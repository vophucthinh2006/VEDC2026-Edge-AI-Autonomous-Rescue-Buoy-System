#!/bin/bash
# Start the 3D boat simulator: Gazebo (waves + buoy) and ArduPilot Rover SITL.
# Mission Planner on Windows then connects with UDP, port 14550.
#
#   bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/run_boat.sh [--headless] [--wipe] [--pi[=simulated]] [--world=flood]
#
#   --headless  Gazebo without its window (physics and sensors only)
#   --wipe      reset ArduPilot's stored parameters to the defaults + params/vedc_buoy.parm
#   --pi        also run the Pi's rescue code (Rasp_Pi/rescue_main.py) on the Gazebo camera and
#               load params/avoidance.parm. --pi=simulated uses the geometric detector instead
#               of the TFLite model. Its log goes to ~/vedc_sim/sitl_run/pi.log.
#   --world=    lake (default): open water, marker buoys, one person in the water.
#               flood: a street under water, houses, obstacles on the way, one person in the
#               water and one on a roof (worlds/vedc_flood.sdf).
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
PI_DETECTOR=""
PI_OVERLAYS=(--overlay config/sim.yaml)
for arg in "$@"; do
    case "$arg" in
        --headless) HEADLESS=1 ;;
        --wipe)     WIPE="--wipe-eeprom" ;;
        --pi)       PI_DETECTOR="camera" ;;
        --pi=*)     PI_DETECTOR="${arg#--pi=}" ;;
        --world=lake)  ;;
        --world=flood) WORLD="vedc_flood.sdf"; PI_OVERLAYS+=(--overlay config/sim_flood.yaml) ;;
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
PI_PID=""
trap 'echo "Stopping Gazebo"; [ -n "$PI_PID" ] && kill $PI_PID 2>/dev/null; kill $GZ_PID 2>/dev/null; wait $GZ_PID 2>/dev/null' EXIT

# Let Gazebo load the world before SITL starts. When both start together and OA_TYPE is set
# (--pi), BendyRuler runs once at boot on an empty request and trips a SITL-only panic
# ("Location cannot be (0, 0, 0)"). The panic spins the avoidance thread forever, so the
# boat holds zero throttle in AUTO. Seen on every boot without this wait, on none with it.
echo "Waiting for Gazebo to load the world"
for _ in $(seq 60); do
    gz topic -l 2>/dev/null | grep -q "vedc_buoy/lidar" && break
    sleep 1
done
sleep 2

mkdir -p "$SIM_DIR/sitl_run"
PARAM_FILES=(--add-param-file="$VEDC_SIM_REPO/params/vedc_buoy.parm")
if [ -n "$PI_DETECTOR" ]; then
    # The avoidance parameters need the Pi's obstacle data to pass the pre-arm check.
    PARAM_FILES+=(--add-param-file="$VEDC_SIM_REPO/params/avoidance.parm")
    # 25 s gives SITL time to open its simulated TELEM2 (SERIAL2, TCP 5763) before the Pi
    # code connects; it then waits for the autopilot's heartbeat.
    ( sleep 25
      cd "$VEDC_SIM_REPO/../Rasp_Pi"
      exec python3 -u rescue_main.py "${PI_OVERLAYS[@]}" --detector "$PI_DETECTOR"
    ) > "$SIM_DIR/sitl_run/pi.log" 2>&1 &
    PI_PID=$!
    echo "Pi rescue code starts in 25 s (detector: $PI_DETECTOR), log: $SIM_DIR/sitl_run/pi.log"
fi

# SITL waits in lock-step for Gazebo's ArduPilotPlugin on UDP 9002.
# MAVProxy already sends to 127.0.0.1:14550, which Windows (Mission Planner)
# sees because .wslconfig uses networkingMode=mirrored. --no-wsl2-network stops
# sim_vehicle.py adding a second output to a guessed "Windows host", which in
# mirrored mode is the home router. 14551 feeds the dashboard bridge
# (esp32-lora-station/tools/mavlink_bridge.py). The Pi code
# (Rasp_Pi/rescue_main.py --overlay config/sim.yaml) connects to SITL's
# SERIAL2 on TCP 5763, the simulated TELEM2.
cd "$SIM_DIR/sitl_run"
sim_vehicle.py -v Rover -f rover-skid --model JSON \
    "${PARAM_FILES[@]}" \
    --custom-location="$HOME_LOCATION" \
    --no-wsl2-network \
    --out=udp:127.0.0.1:14551 \
    --no-rebuild $WIPE "${MAVPROXY_ARGS[@]}"
