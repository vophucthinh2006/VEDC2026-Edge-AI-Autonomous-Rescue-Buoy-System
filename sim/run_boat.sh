#!/bin/bash
# Start the 3D boat simulator: Gazebo (waves + buoy) and ArduPilot Rover SITL.
# Mission Planner on Windows then connects with UDP, port 14550.
#
#   bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/run_boat.sh [--headless] [--wipe] [--pi[=simulated]] [--hw] [--oneway] [--world=flood|village|two|central]
#
#   --headless  Gazebo without its window (physics and sensors only)
#   --wipe      reset ArduPilot's stored parameters to the defaults + params/vedc_buoy.parm
#   --pi        also run the Pi's rescue code (Rasp_Pi/rescue_main.py) on the Gazebo camera and
#               load params/avoidance.parm. --pi=simulated uses the geometric detector instead
#               of the TFLite model. Its log goes to ~/vedc_sim/sitl_run/pi.log, and what it
#               sees (camera with the detector's box, LiDAR from above) to http://127.0.0.1:8090/
#   --hw        with --pi: the Pi code reaches the simulator the way it reaches the boat's hardware.
#               The LiDAR comes as the LD14's serial stream (tools/ld14_emulator.py) and the
#               camera servo, alarm and LoRa beacon go through a simulated STM32F103 helper board
#               running the firmware's own logic (tools/helper_board_sim.py). What the board does
#               is logged in ~/vedc_sim/sitl_run/helper_events.log; a file named "estop" there
#               presses the E-stop button. With VEDC_INGEST_TOKEN set, its LoRa beacons are posted
#               to VEDC_INGEST_URL (default http://127.0.0.1:8787/api/ingest) as buoy PHAO-01.
#   --oneway    rear motors forward only, like the boat's ESCs (params/oneway_esc.parm)
#   --world=    lake (default): open water, marker buoys, one person in the water.
#               flood: a street under water, houses, obstacles on the way, one person in the
#               water and one on a roof (worlds/vedc_flood.sdf).
#               village: the flood scene with every case in it: one person in the water, one on a
#               roof, three in the water together, three on a roof together (worlds/vedc_village.sdf).
#               central: central Vietnam, November 2025, from press reports: a village by the lagoon
#               after Typhoon Kalmaegi (roofs gone, wrecked boats, 1.2 m of water), then a hamlet
#               flooded to the eaves with people on the roofs (worlds/vedc_central.sdf).
#               two: the lake with two people 3 m apart, 32 m east and 6.5 / 9.5 m north of the
#               start (worlds/vedc_two.sdf).
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
HW=0
ONEWAY=0
for arg in "$@"; do
    case "$arg" in
        --headless) HEADLESS=1 ;;
        --wipe)     WIPE="--wipe-eeprom" ;;
        --pi)       PI_DETECTOR="camera" ;;
        --pi=*)     PI_DETECTOR="${arg#--pi=}" ;;
        --hw)       HW=1 ;;
        --oneway)   ONEWAY=1 ;;
        --world=lake)  ;;
        --world=flood) WORLD="vedc_flood.sdf"; PI_OVERLAYS+=(--overlay config/sim_flood.yaml) ;;
        --world=two)   WORLD="vedc_two.sdf";   PI_OVERLAYS+=(--overlay config/sim_two.yaml) ;;
        --world=central) WORLD="vedc_central.sdf"; PI_OVERLAYS+=(--overlay config/sim_central.yaml) ;;
        --world=village) WORLD="vedc_village.sdf"; PI_OVERLAYS+=(--overlay config/sim_village.yaml) ;;
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
HW_PIDS=""
trap 'echo "Stopping Gazebo"; [ -n "$PI_PID" ] && kill $PI_PID 2>/dev/null; [ -n "$HW_PIDS" ] && kill $HW_PIDS 2>/dev/null; kill $GZ_PID 2>/dev/null; wait $GZ_PID 2>/dev/null' EXIT

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
if [ -n "$PI_DETECTOR" ] && [ "$HW" = 1 ]; then
    # The two stand-ins for the boat's hardware, each behind a pseudo-terminal the Pi code opens
    # like a UART. The paths go into a small overlay on top of config/sim_hw.yaml.
    RUN="$SIM_DIR/sitl_run"
    rm -f "$RUN/estop"
    source "$VEDC_SIM_REPO/tools/hw_standins.sh"
    start_hw_standins "$RUN" /vedc_buoy/lidar /model/vedc_buoy/joint/camera_pan_joint/cmd_pos
    PI_OVERLAYS+=(--overlay config/sim_hw.yaml --overlay "$RUN/hw_ports.yaml")
    echo "Hardware stand-ins: ${VEDC_LIDAR_MODEL:-ld14} LiDAR on $RUN/lidar_pty, helper board on $RUN/helper_pty (log: $RUN/helper_events.log)"
fi
if [ -n "$PI_DETECTOR" ]; then
    # The avoidance parameters need the Pi's obstacle data to pass the pre-arm check.
    PARAM_FILES+=(--add-param-file="$VEDC_SIM_REPO/params/avoidance.parm")
    # 25 s gives SITL time to open its simulated TELEM2 (SERIAL2, TCP 5763) before the Pi
    # code connects; it then waits for the autopilot's heartbeat.
    ( sleep 25
      cd "$VEDC_SIM_REPO/../Rasp_Pi"
      exec python3 -u rescue_main.py "${PI_OVERLAYS[@]}" --detector "$PI_DETECTOR" --viewer 8090
    ) > "$SIM_DIR/sitl_run/pi.log" 2>&1 &
    PI_PID=$!
    echo "Pi rescue code starts in 25 s (detector: $PI_DETECTOR), log: $SIM_DIR/sitl_run/pi.log"
    echo "What the Pi sees (camera + LiDAR): http://127.0.0.1:8090/"
fi

# SITL waits in lock-step for Gazebo's ArduPilotPlugin on UDP 9002.
# MAVProxy already sends to 127.0.0.1:14550, which Windows (Mission Planner)
# sees because .wslconfig uses networkingMode=mirrored. --no-wsl2-network stops
# sim_vehicle.py adding a second output to a guessed "Windows host", which in
# mirrored mode is the home router. 14551 feeds the dashboard bridge
# (esp32-lora-station/tools/mavlink_bridge.py). The Pi code
# (Rasp_Pi/rescue_main.py --overlay config/sim.yaml) connects to SITL's
# SERIAL2 on TCP 5763, the simulated TELEM2.
if [ "$ONEWAY" = 1 ]; then
    PARAM_FILES+=(--add-param-file="$VEDC_SIM_REPO/params/oneway_esc.parm")
fi
cd "$SIM_DIR/sitl_run"
sim_vehicle.py -v Rover -f rover-skid --model JSON \
    "${PARAM_FILES[@]}" \
    --custom-location="$HOME_LOCATION" \
    --no-wsl2-network \
    --out=udp:127.0.0.1:14551 \
    --no-rebuild $WIPE "${MAVPROXY_ARGS[@]}"
