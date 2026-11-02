#!/bin/bash
# Start the Skycharge digital twin (CopterSonde-sim/skycharge): the charge source of the BOLOGNINI S1 pad,
# serving skycharge-cli over SSH (127.0.0.1:2222, the vendor key) and the JSON API (127.0.0.1:8088).
#
# Usage: skycharge_twin.sh [--gazebo] [other options, see --help]
#   --gazebo   the CopterSonde in the Gazebo world is the plant (start coptersonde_gazebo.sh too, in the
#              same GZ_PARTITION); without it a battery inside the twin is (skytwin place/remove over SSH).
#
# The twin needs paramiko 4 (the pad, like the twin, only takes SHA-1 ssh-rsa signatures). A venv with
# it is made on first use in SKYCHARGE_VENV (default ~/.cache/coptersonde/venv-skycharge); it sees the
# system packages, which hold the Gazebo Python bindings.

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="$(dirname "$TOOLS")"
VENV="${SKYCHARGE_VENV:-$HOME/.cache/coptersonde/venv-skycharge}"

if [ ! -x "$VENV/bin/python" ] || ! "$VENV/bin/python" -c 'import paramiko' 2>/dev/null; then
  echo "Skycharge twin: making $VENV (paramiko 4) ..."
  /usr/bin/python3 -m venv --system-site-packages "$VENV" && "$VENV/bin/pip" install -q "paramiko>=4,<5" || exit 1
fi
cd "$SIM/skycharge" || exit 1
exec "$VENV/bin/python" -m skycharge_twin "$@"
