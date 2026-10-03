#!/bin/bash
# Start the 3D boat simulator: Gazebo (waves + buoy) and ArduPilot Rover SITL.
# Mission Planner on Windows then connects with UDP, port 14550.
#
#   bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/run_boat.sh [--headless] [--wipe]
#
#   --headless  Gazebo without its window (physics and sensors only)
#   --wipe      reset ArduPilot's stored parameters to the defaults + params/vedc_buoy.parm
#
# Home position: HOME_LOCATION=lat,lon,alt,heading. Default: the middle of Ho da 01,
# the largest of the Ho Da quarry lakes in the VNU-HCM campus (OpenStreetMap
# relation 3634964), 217 m from the nearest shore. The Gazebo bow faces east,
# about 265 m of open water that way, so keep heading 90.
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

HOME_LOCATION="${HOME_LOCATION:-10.883438,106.796019,10,90}"
WORLD="${WORLD:-vedc_lake.sdf}"
HEADLESS=0
WIPE=""
for arg in "$@"; do
    case "$arg" in
        --headless) HEADLESS=1 ;;
        --wipe)     WIPE="--wipe-eeprom" ;;
        *) echo "unknown option $arg" >&2; exit 2 ;;
    esac
done

GZ_ARGS=(-v 3 -r "$WORLD")
MAVPROXY_ARGS=()
if [ "$HEADLESS" = 1 ]; then
    GZ_ARGS=(-s "${GZ_ARGS[@]}")
    # No terminal to type into: without --daemon MAVProxy reads EOF and quits.
    MAVPROXY_ARGS=(--mavproxy-args="--daemon")
fi

echo "Starting Gazebo: gz sim ${GZ_ARGS[*]}"
gz sim "${GZ_ARGS[@]}" &
GZ_PID=$!
trap 'echo "Stopping Gazebo"; kill $GZ_PID 2>/dev/null; wait $GZ_PID 2>/dev/null' EXIT

# SITL waits in lock-step for Gazebo's ArduPilotPlugin on UDP 9002.
# MAVProxy already sends to 127.0.0.1:14550, which Windows (Mission Planner)
# sees because .wslconfig uses networkingMode=mirrored. --no-wsl2-network stops
# sim_vehicle.py adding a second output to a guessed "Windows host", which in
# mirrored mode is the home router. 14551 feeds the dashboard bridge
# (esp32-lora-station/tools/mavlink_bridge.py) or tools/smoke_test.py.
mkdir -p "$SIM_DIR/sitl_run"
cd "$SIM_DIR/sitl_run"
sim_vehicle.py -v Rover -f rover-skid --model JSON \
    --add-param-file="$VEDC_SIM_REPO/params/vedc_buoy.parm" \
    --custom-location="$HOME_LOCATION" \
    --no-wsl2-network \
    --out=udp:127.0.0.1:14551 \
    --no-rebuild $WIPE "${MAVPROXY_ARGS[@]}"
