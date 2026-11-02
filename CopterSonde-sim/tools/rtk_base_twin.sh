#!/bin/bash
# Start the RTK base station twin (CopterSonde-sim/rtk_base): a u-blox ZED-F9P base that LAAIRS's RTK
# Injection window connects to as the serial port socket://127.0.0.1:3101, surveys in, and injects its RTCM3
# corrections into the drone. SITL turns RTK fixed only while they arrive (SIM_GPS_RTK 1).
#
# Usage: rtk_base_twin.sh [--listen HOST:PORT] [--location NAME] [--offset EAST NORTH] [--help]
#   It needs only Python 3.

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$(dirname "$TOOLS")/rtk_base" || exit 1
exec python3 -m rtk_base_twin "$@"
