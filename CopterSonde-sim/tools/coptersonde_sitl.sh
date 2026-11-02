#!/bin/bash
# Start CopterSonde SITL for the norman_coptersonde Gazebo world (start Gazebo first with
# coptersonde_gazebo.sh).
#
# Runs this checkout's sim_vehicle.py (not the one on PATH, which may belong to another
# ArduPilot tree) with the CopterSonde_Norman location, config/coptersonde.parm and no MAVProxy.
# Connect LAAIRS ("WSL SITL") or another ground station to TCP 127.0.0.1:5760; SITL waits for
# that connection before it starts. TCP 5762 is free for MAVProxy or the flight-test script.
#
# Usage: coptersonde_sitl.sh [other sim_vehicle.py options]
#   The Gazebo model talks to SITL instance 0 (fdm_port_in 9002 in models/coptersonde/model.sdf).

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM="$(dirname "$TOOLS")"
REPO="$(dirname "$SIM")"

cd "$REPO" || exit 1
exec Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -L CopterSonde_Norman \
  --add-param-file="$SIM/config/coptersonde.parm" \
  --no-mavproxy "$@"
