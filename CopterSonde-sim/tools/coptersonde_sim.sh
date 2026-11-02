#!/bin/bash
# Start the whole CopterSonde simulator from one terminal: Gazebo (Norman world, launch deck and Skycharge
# pad), CopterSonde SITL, the Skycharge digital twin and the RTK base station twin. Ctrl-C (or closing the
# Gazebo window) stops all of it, and only what this script started.
#
# First it puts ardupilot_gazebo on the CopterSonde branch (main-CopterSonde) and rebuilds its plugins
# when the sources changed. That checkout is shared with RFSonde, so it asks before switching branch and
# never switches or builds under a running Gazebo. SITL rebuilds the firmware of this checkout
# (ArduPilot-4.6-CS-gazebo) as usual.
#
# Usage: coptersonde_sim.sh [options] [-- other sim_vehicle.py options]
#   -I N, --instance N   SITL instance N: MAVLink TCP 5760+10N, Gazebo JSON 9002+10N (default 0)
#   --mavproxy           run MAVProxy here on SERIAL0 instead of waiting for LAAIRS
#   --mavproxy-args "A"  extra MAVProxy options, e.g. "--console --map"
#   --headless           Gazebo server only, no window
#   --no-skycharge       don't start the Skycharge digital twin
#   --no-rtk-base        don't start the RTK base twin (then the drone flies without RTK)
#   --no-build           use the plugins and the firmware as built
#   --switch-branch      switch ardupilot_gazebo to main-CopterSonde without asking
#   --battery-soc S      start the pack at state of charge S (0..1)
#   --battery-ah A       pack capacity in Ah (1 makes charging tests short)
#   --start-hour H, --cycle-hours H, --truth-log FILE, --speed N   see coptersonde_gazebo.sh
#   -h, --help           this text
#
# Paths come from where this ArduPilot checkout is, so it runs on any computer: ardupilot_gazebo must be
# cloned next to it, in the same folder (or set ARDUPILOT_GAZEBO). Gazebo, cmake, iproute2 (ss) and
# ArduPilot's Python tools are needed, as for any ArduPilot SITL with Gazebo.
#
# Logs go to ~/.cache/coptersonde/run/I<N>/ (gazebo.log, sitl.log, skycharge.log, rtk-base.log). This
# terminal shows SITL, the twins' events and Gazebo's errors.

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="$(dirname "$TOOLS")"
REPO="$(dirname "$SIM")"
PARENT="$(dirname "$REPO")"
FW_BRANCH=ArduPilot-4.6-CS-gazebo
GZ_BRANCH=main-CopterSonde

die() {
  echo "coptersonde_sim: $*" >&2
  exit 1
}

usage() {
  sed -n '2,/^$/s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"
  exit "${1:-0}"
}

instance=0
mavproxy=0
mavproxy_args=""
headless=0
twin=1
rtk=1
build=1
switch=0
gz_args=()
sitl_args=()
while [ $# -gt 0 ]; do
  case "$1" in
    -I | --instance) instance="$2"; shift ;;
    -I*) instance="${1#-I}" ;;
    --mavproxy) mavproxy=1 ;;
    --mavproxy-args) mavproxy_args="$2"; shift ;;
    --headless) headless=1; gz_args+=(--headless) ;;
    --no-skycharge) twin=0 ;;
    --no-rtk-base) rtk=0 ;;
    --no-build) build=0 ;;
    --switch-branch) switch=1 ;;
    --battery-soc) export CS_BATTERY_SOC="$2"; shift ;;
    --battery-ah) export CS_BATTERY_AH="$2"; shift ;;
    --start-hour | --cycle-hours | --truth-log | --speed) gz_args+=("$1" "$2"); shift ;;
    -h | --help) usage ;;
    --) shift; sitl_args+=("$@"); break ;;
    *) echo "coptersonde_sim: unknown option $1" >&2; usage 1 >&2 ;;
  esac
  shift
done
[[ $instance =~ ^[0-9]+$ ]] || die "the instance must be a number, not '$instance'"

# ardupilot_gazebo: next to this checkout, under that name or any other
if [ -n "$ARDUPILOT_GAZEBO" ]; then
  APG="$ARDUPILOT_GAZEBO"
else
  APG="$PARENT/ardupilot_gazebo"
  if [ ! -f "$APG/src/ArduPilotPlugin.cc" ]; then
    for d in "$PARENT"/*/; do
      [ -f "$d/src/ArduPilotPlugin.cc" ] && [ -e "$d/.git" ] && APG="${d%/}" && break
    done
  fi
fi
[ -f "$APG/src/ArduPilotPlugin.cc" ] ||
  die "no ardupilot_gazebo checkout next to $REPO; clone it into $PARENT (or set ARDUPILOT_GAZEBO)"
APG="$(cd "$APG" && pwd)"
export ARDUPILOT_GAZEBO="$APG"

RUN="${XDG_CACHE_HOME:-$HOME/.cache}/coptersonde/run/I$instance"
mavlink_port=$((5760 + 10 * instance))
json_port=$((9002 + 10 * instance))
ssh_port=$((2222 + 10 * instance))
http_port=$((8088 + 10 * instance))
rtk_port=$((3101 + 10 * instance))

# ---- environment ----------------------------------------------------------------------------------
command -v gz > /dev/null || die "Gazebo (gz) not found; install Gazebo Jetty"
[ "$mavproxy" = 0 ] || command -v mavproxy.py > /dev/null ||
  die "mavproxy.py not found; activate the Python environment you use for ArduPilot"
if [ -z "$GZ_VERSION" ]; then
  # the ardupilot_gazebo build picks its Gazebo libraries by release name
  case "$(gz sim --versions 2> /dev/null | cut -d. -f1)" in
    8) export GZ_VERSION=harmonic ;;
    9) export GZ_VERSION=ionic ;;
    10) export GZ_VERSION=jetty ;;
  esac
fi
if [ "$headless" = 0 ] && [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
  die "no display for the Gazebo window; use --headless"
fi
if [ "$instance" != 0 ] && [ -z "$GZ_PARTITION" ]; then
  # keep this Gazebo's topics (and the twin's) apart from an instance-0 simulator
  export GZ_PARTITION="coptersonde_I$instance"
fi

gz_running() {
  # gz-sim-main (Jetty) or the older gz sim ruby front end
  pgrep -f '(^|/)gz-sim-|(^|/)gz sim' | tr '\n' ' '
}

# ---- firmware branch -------------------------------------------------------------------------------
fw_branch="$(git -C "$REPO" symbolic-ref --short -q HEAD)"
if [ "$fw_branch" != "$FW_BRANCH" ]; then
  echo "coptersonde_sim: warning: $REPO is on '${fw_branch:-a detached HEAD}', not $FW_BRANCH;" \
    "SITL runs that firmware" >&2
fi

# ---- ardupilot_gazebo branch -----------------------------------------------------------------------
[ -d "$APG/.git" ] || die "$APG is not the ardupilot_gazebo checkout (set ARDUPILOT_GAZEBO)"
gz_branch="$(git -C "$APG" symbolic-ref --short -q HEAD)"
if [ "$gz_branch" != "$GZ_BRANCH" ]; then
  git -C "$APG" rev-parse -q --verify "$GZ_BRANCH" > /dev/null ||
    git -C "$APG" rev-parse -q --verify "origin/$GZ_BRANCH" > /dev/null ||
    die "$APG has no branch $GZ_BRANCH (git -C $APG fetch origin)"
  [ -z "$(git -C "$APG" status --porcelain --untracked-files=no)" ] ||
    die "$APG is on '$gz_branch' with uncommitted changes; commit or stash them there, then switch to $GZ_BRANCH"
  pids="$(gz_running)"
  [ -z "$pids" ] || die "$APG is on '$gz_branch' and Gazebo is running (PID $pids); stop it before switching to $GZ_BRANCH"
  if [ "$switch" = 0 ]; then
    [ -t 0 ] || die "$APG is on '$gz_branch'; rerun with --switch-branch to switch it to $GZ_BRANCH"
    read -r -p "ardupilot_gazebo (shared with RFSonde) is on '${gz_branch:-a detached HEAD}'. Switch it to $GZ_BRANCH? [y/N] " ans
    [[ $ans =~ ^[Yy] ]] || die "not switching; the CopterSonde world needs $GZ_BRANCH"
  fi
  git -C "$APG" checkout "$GZ_BRANCH" || die "could not switch $APG to $GZ_BRANCH"
  echo "coptersonde_sim: switched ardupilot_gazebo to $GZ_BRANCH; back with: git -C $APG checkout $gz_branch"
fi

# ---- ardupilot_gazebo plugins ----------------------------------------------------------------------
STAMP="$APG/build/.coptersonde-sources"

plugin_sources() {
  # what the plugins were built from: the commit, local changes and untracked sources
  ( cd "$APG" || exit 1
    git rev-parse HEAD
    git diff HEAD
    git ls-files -z --others --exclude-standard | xargs -0 -r sha1sum
    echo "GZ_VERSION=$GZ_VERSION" ) | sha1sum | cut -d' ' -f1
}

mkdir -p "$RUN" || die "cannot make $RUN"
plugins="as built (--no-build)"
if [ "$build" = 1 ]; then
  sources="$(plugin_sources)"
  if [ "$sources" != "$(cat "$STAMP" 2> /dev/null)" ] || [ ! -f "$APG/build/libCopterSondePlugin.so" ]; then
    pids="$(gz_running)"
    [ -z "$pids" ] || die "the ardupilot_gazebo plugins may need a rebuild, but Gazebo is running (PID $pids)." \
      "Stop it, or pass --no-build to use the plugins as built"
    echo "coptersonde_sim: building the ardupilot_gazebo plugins ($GZ_BRANCH) ..."
    if [ ! -f "$APG/build/CMakeCache.txt" ]; then
      cmake -S "$APG" -B "$APG/build" > "$RUN/plugins-build.log" 2>&1 ||
        { tail -20 "$RUN/plugins-build.log"; die "cmake failed, see $RUN/plugins-build.log"; }
    fi
    cmake --build "$APG/build" -j"$(nproc)" >> "$RUN/plugins-build.log" 2>&1 ||
      { tail -20 "$RUN/plugins-build.log"; die "the plugin build failed, see $RUN/plugins-build.log"; }
    echo "$sources" > "$STAMP"
    plugins="rebuilt"
  else
    plugins="up to date"
  fi
fi
[ -f "$APG/build/libCopterSondePlugin.so" ] || die "$APG/build/libCopterSondePlugin.so is missing; run without --no-build"

# ---- is this instance free? ------------------------------------------------------------------------
tcp_busy() { ss -Hltn "sport = :$1" 2> /dev/null | grep -q .; }
udp_busy() { ss -Hlun "sport = :$1" 2> /dev/null | grep -q .; }

tcp_busy "$mavlink_port" && die "TCP $mavlink_port is in use: SITL instance $instance is already running. Use -I with another number"
udp_busy "$json_port" && die "UDP $json_port is in use: a Gazebo vehicle for instance $instance is already running. Use -I with another number"
if [ "$twin" = 1 ]; then
  for p in "$ssh_port" "$http_port"; do
    tcp_busy "$p" && die "TCP $p is in use (another Skycharge twin?); pass --no-skycharge or use another -I"
  done
fi
if [ "$rtk" = 1 ] && tcp_busy "$rtk_port"; then
  die "TCP $rtk_port is in use (another RTK base twin?); pass --no-rtk-base or use another -I"
fi
if [ "$instance" = 0 ] && pgrep -x arducopter > /dev/null && ! python3 -c 'import psutil' 2> /dev/null; then
  # without psutil, sim_vehicle.py -I0 kills every arducopter by name
  die "another arducopter is running and python3 has no psutil, so SITL instance 0 would kill it." \
    "Activate the Python environment you use for ArduPilot (it has psutil) or use -I 1"
fi

# ---- start -----------------------------------------------------------------------------------------
pgids=()
names=()
viewer=""

stop_all() {
  trap - INT TERM HUP EXIT
  [ -n "$viewer" ] && kill -- "-$viewer" 2> /dev/null
  [ ${#pgids[@]} -gt 0 ] || return
  echo
  echo "coptersonde_sim: stopping ${names[*]} ..."
  local g n
  for g in "${pgids[@]}"; do
    kill -TERM -- "-$g" 2> /dev/null
  done
  for n in $(seq 50); do
    for g in "${pgids[@]}"; do
      kill -0 -- "-$g" 2> /dev/null && break
      g=""
    done
    [ -z "$g" ] && break
    sleep 0.1
  done
  for g in "${pgids[@]}"; do
    kill -KILL -- "-$g" 2> /dev/null
  done
  wait 2> /dev/null
}
trap 'stop_all; exit 130' INT TERM HUP
trap stop_all EXIT

start() {
  # start "name" log command...: in its own session, so it gets no Ctrl-C and stops as one group
  local name="$1" log="$2"
  shift 2
  : > "$log"
  setsid "$@" > "$log" 2>&1 < /dev/null &
  pgids+=($!)
  names+=("$name")
}

export PYTHONUNBUFFERED=1
rm -f "$RUN"/ritw-*
start gazebo "$RUN/gazebo.log" "$TOOLS/coptersonde_gazebo.sh" --instance "$instance" "${gz_args[@]}"

sitl=(-I "$instance")
[ "$build" = 0 ] && sitl+=(--no-rebuild)
# instance 0 keeps its eeprom.bin and logs/ in the repo root, as coptersonde_sitl.sh does; others get their own
[ "$instance" != 0 ] && sitl+=(--use-dir "${XDG_CACHE_HOME:-$HOME/.cache}/coptersonde/sitl/I$instance")
# SITL_RITW_TERMINAL=bash keeps the arducopter console in sitl.log (sim_vehicle.py would open an xterm)
start sitl "$RUN/sitl.log" env SITL_RITW_TERMINAL=bash TMPDIR="$RUN" "$TOOLS/coptersonde_sitl.sh" "${sitl[@]}" "${sitl_args[@]}"

if [ "$twin" = 1 ]; then
  start skycharge "$RUN/skycharge.log" "$TOOLS/skycharge_twin.sh" --gazebo \
    --ssh "127.0.0.1:$ssh_port" --http "127.0.0.1:$http_port"
fi
if [ "$rtk" = 1 ]; then
  start rtk-base "$RUN/rtk-base.log" "$TOOLS/rtk_base_twin.sh" --listen "127.0.0.1:$rtk_port"
fi

show_logs() {
  # one live view: everything from SITL, the twins' events (not LAAIRS's polls) and Gazebo's errors
  local src="" line
  tail -n +1 -F "$RUN/gazebo.log" "$RUN/sitl.log" ${twin:+"$RUN/skycharge.log"} ${rtk:+"$RUN/rtk-base.log"} \
    2> /dev/null |
    while IFS= read -r line; do
      case "$line" in
        "==> $RUN/gazebo.log <==") src=gazebo; continue ;;
        "==> $RUN/sitl.log <==") src=sitl; continue ;;
        "==> $RUN/skycharge.log <==") src=skycharge; continue ;;
        "==> $RUN/rtk-base.log <==") src=rtk-base; continue ;;
        "") continue ;;
      esac
      case "$src" in
        gazebo)
          while [[ $line =~ $'\e'\[[0-9\;]*m ]]; do
            line="${line/"${BASH_REMATCH[0]}"/}"
          done
          case "$line" in
            *"Loaded level [default]"*) line="world loaded" ;;
            *"[error]"* | *"[critical]"* | *"[CopterSonde]"* | "CopterSonde:"*)
              [[ $line =~ ^\([^\)]*\)\ \[([a-z]+)\]\ \[[^]]*\]\ (.*)$ ]] &&
                line="${BASH_REMATCH[2]}" && [ "${BASH_REMATCH[1]}" != info ] && line="${BASH_REMATCH[1]}: $line" ;;
            *) continue ;;
          esac ;;
        skycharge)
          case "$line" in
            "ssh: exec skycharge-cli show-"* | "http: "*) continue ;;
          esac ;;
      esac
      printf '%-9s | %s\n' "$src" "$line"
    done
}

[ "$twin" = 1 ] || twin=""
[ "$rtk" = 1 ] || rtk=""
export -f show_logs
RUN="$RUN" twin="$twin" rtk="$rtk" setsid bash -c show_logs < /dev/null &
viewer=$!

gcs="LAAIRS \"WSL SITL\" (tcp 127.0.0.1:5760); SITL waits for it, then tcp $((mavlink_port + 2)) is free for MAVProxy"
[ "$instance" != 0 ] && gcs="a ground station on tcp 127.0.0.1:$mavlink_port (SITL waits for it)"
[ "$mavproxy" = 1 ] && gcs="MAVProxy in this terminal, on tcp 127.0.0.1:$mavlink_port"
cat << EOF
CopterSonde simulator, instance $instance${GZ_PARTITION:+ (GZ_PARTITION=$GZ_PARTITION)}
  firmware   ${fw_branch:-detached HEAD}, $REPO
  gazebo     $GZ_BRANCH, plugins $plugins, $APG$([ "$headless" = 1 ] && echo ", headless")
  connect    $gcs
EOF
if [ -n "$twin" ]; then
  echo "  skycharge  digital twin on ssh 127.0.0.1:$ssh_port; LAAIRS Skycharge console:" \
    "$([ "$ssh_port" = 2222 ] && echo '"Digital twin (simulator)"' || echo "Custom host 127.0.0.1 port $ssh_port")"
fi
if [ -n "$rtk" ]; then
  echo "  rtk base   u-blox F9P twin on socket://127.0.0.1:$rtk_port; LAAIRS RTK Injection:" \
    "$([ "$rtk_port" = 3101 ] && echo '"Digital twin (simulator)", then Survey-In' || echo "port $rtk_port")"
fi
echo "  logs       $RUN"
echo "Ctrl-C stops everything this script started."
echo

if [ "$mavproxy" = 1 ]; then
  # wait for SITL to build and listen, then hand this terminal to MAVProxy
  until tcp_busy "$mavlink_port"; do
    kill -0 "${pgids[1]}" 2> /dev/null || die "SITL stopped before it was ready, see $RUN/sitl.log"
    sleep 1
  done
  kill -- "-$viewer" 2> /dev/null
  viewer=""
  echo "coptersonde_sim: SITL is ready; starting MAVProxy (logs in $RUN/sitl.log, $RUN/gazebo.log)"
  # shellcheck disable=SC2086
  (cd "$RUN" && mavproxy.py --master "tcp:127.0.0.1:$mavlink_port" $mavproxy_args)
  exit 0
fi

# poll rather than wait -n, so that Ctrl-C always reaches the trap at once
while :; do
  for i in "${!pgids[@]}"; do
    kill -0 "${pgids[$i]}" 2> /dev/null && continue
    wait "${pgids[$i]}"
    status=$?
    sleep 0.5
    echo
    echo "coptersonde_sim: ${names[$i]} exited (status $status); last lines of its log:"
    tail -5 "$RUN/${names[$i]}.log"
    exit "$status"
  done
  sleep 1
done
