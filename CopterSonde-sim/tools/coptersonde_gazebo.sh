#!/bin/bash
# Start Gazebo with the CopterSonde Norman world (then start SITL with coptersonde_sitl.sh).
#
# The CopterSonde-sim models and the atmosphere table come first on the resource path, so the
# RFSonde-sim models that ardupilot_gazebo/env.sh adds can't shadow them.
#
# Usage: coptersonde_gazebo.sh [--start-hour H] [--cycle-hours H] [--truth-log FILE] [--headless]
#                              [--instance N] [--speed N] [other gz sim options]
#   --start-hour H   local time of day when the simulation starts (default: the model's 9)
#   --cycle-hours H  simulation hours for one day of the atmosphere (default 24 = real time)
#   --truth-log FILE write the air and battery at the vehicle at 10 Hz (CSV)
#   --headless       server only, no GUI (gz sim -s)
#   --instance N     talk to SITL instance N (coptersonde_sitl.sh -I N); default 0
#   --speed N        run N times faster than real time, if the computer keeps up (gz sim -z)
#   ARDUPILOT_GAZEBO the ardupilot_gazebo checkout with the built plugins (default ~/Github/ardupilot_gazebo)

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="$(dirname "$TOOLS")"
APG="${ARDUPILOT_GAZEBO:-$HOME/Github/ardupilot_gazebo}"

extra=()
server=()
instance=0
while [ $# -gt 0 ]; do
  case "$1" in
    --start-hour) export CS_START_HOUR="$2"; shift ;;
    --cycle-hours) export CS_CYCLE_HOURS="$2"; shift ;;
    --truth-log) export CS_TRUTH_LOG="$2"; shift ;;
    --headless) server=(-s) ;;
    --instance) instance="$2"; shift ;;
    --speed) extra+=(-z "$(python3 -c "print(int(1000 * float('$2')))")"); shift ;;
    *) extra+=("$1") ;;
  esac
  shift
done

if [ ! -f "$SIM/worlds/norman_coptersonde.sdf" ]; then
  echo "CopterSonde: generating the Norman world (once) ..."
  python3 "$SIM/tools/make_norman_world.py" || exit 1
fi
if [ ! -f "$APG/build/libCopterSondePlugin.so" ]; then
  echo "CopterSonde: $APG/build/libCopterSondePlugin.so not found; build ardupilot_gazebo (branch main-CopterSonde)"
  exit 1
fi

models="$SIM/models"
if [ "$instance" != 0 ]; then
  # a copy of the vehicle that talks to SITL instance N (JSON port 9002 + 10 N), first on the path
  models="${TMPDIR:-/tmp}/coptersonde_I$instance"
  rm -rf "$models"
  mkdir -p "$models"
  cp -r "$SIM/models/coptersonde" "$models/"
  sed -i "s#<fdm_port_in>9002</fdm_port_in>#<fdm_port_in>$((9002 + 10 * instance))</fdm_port_in>#" \
    "$models/coptersonde/model.sdf"
  models="$models:$SIM/models"
  echo "CopterSonde: Gazebo talks to SITL instance $instance (JSON port $((9002 + 10 * instance)))"
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$APG/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$models:$SIM/worlds:$SIM:$APG/models:$APG/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

exec gz sim -v4 -r "${server[@]}" "${extra[@]}" "$SIM/worlds/norman_coptersonde.sdf"
