#!/bin/bash
# A fleet of STM32 boats (Raspberry Pi + STM32F407) in Gazebo, all on one shore station and one LoRa channel
# (docs/fleet-control.md). Boat n is the model vedc_buoy_n (vedc_buoy for PHAO-01) with its own simulated F407
# (tools/stm32_board_sim.py --model --buoy-id), LD14 and Rasp_Pi/main.py; the station (tools/shore_station_sim.py)
# hears them all on the shared channel (tools/lora_radio.py Air, frames that overlap are lost) and takes routes,
# commands and formations from the Worker, as on the dashboard.
#
#   VEDC_INGEST_TOKEN=<INGEST_TOKEN> bash sim/tools/fleet_sim.sh [--boats=3] [--world=central] [--headless]
#                                         [--no-tdma] [--vision=all|first] [--viewer] [--run=<dir>] [--duration=<s>]
#   --boats=      how many boats, 1..4 (default 3): PHAO-01 .. PHAO-0n
#   --world=      central (default), lake, flood, village, two
#   --no-tdma     every boat beacons on its own clock, as a lone boat does (the channel then loses frames)
#   --no-claims   the station tells no boat to leave a person alone (docs/victim-claim.md): every boat stays
#                 with every person it sees, as before the claims
#   --vision=     all (default): every boat runs the detector; first: only PHAO-01, the others main.py --no-camera
#   --viewer      main.py's viewer for each boat, http://127.0.0.1:8090 + n - 1
#   --run=        where everything goes (default ~/vedc_sim/fleet_run): boat_n/{pi.log,stm32_sim.log,track.csv,...},
#                 station_sim.log, air_station.json
#   --duration=   stop after this many seconds (default: run until Ctrl+C)
#   --hold=       seconds a boat stays with a person before it goes on (follow.max_hold_s; default the boat's, 180)
#   VEDC_CLOUD_URL   the Worker, default http://127.0.0.1:8787
#   VEDC_STATION_POS the shore station, lat,lon; default 30 m west of the world origin, on the bank
#   FLEET_STARTS     "east,north,heading" per boat, space separated; default side by side west of the origin
set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_fleet
[ -n "$VEDC_INGEST_TOKEN" ] || { echo "set VEDC_INGEST_TOKEN to the Worker's INGEST_TOKEN" >&2; exit 2; }
CLOUD="${VEDC_CLOUD_URL:-http://127.0.0.1:8787}"

BOATS=3 WORLD=central HEADLESS=0 TDMA=1 CLAIMS=1 VISION=all VIEWER=0 RUN="$SIM_DIR/fleet_run" DURATION="" HOLD=""
for arg in "$@"; do
    case "$arg" in
        --boats=*)    BOATS="${arg#--boats=}" ;;
        --world=*)    WORLD="${arg#--world=}" ;;
        --headless)   HEADLESS=1 ;;
        --no-tdma)    TDMA=0 ;;
        --no-claims)  CLAIMS=0 ;;
        --vision=*)   VISION="${arg#--vision=}" ;;
        --viewer)     VIEWER=1 ;;
        --run=*)      RUN="${arg#--run=}" ;;
        --duration=*) DURATION="${arg#--duration=}" ;;
        --hold=*)     HOLD="${arg#--hold=}" ;;
        *) echo "unknown option $arg" >&2; exit 2 ;;
    esac
done
[ "$BOATS" -ge 1 ] && [ "$BOATS" -le 4 ] || { echo "--boats is 1 to 4 (App/lora_slot.h LORA_SLOT_COUNT)" >&2; exit 2; }
case "$WORLD" in
    lake) SDF=vedc_lake.sdf; OVERLAY="" ;;
    *)    SDF="vedc_$WORLD.sdf"; OVERLAY="config/sim_$WORLD.yaml" ;;
esac
[ -f "$VEDC_SIM_REPO/worlds/$SDF" ] || { echo "no world $SDF" >&2; exit 2; }

HOME_LATLON="10.883438,106.796019"   # run_boat.sh HOME_LOCATION: the world origin
export VEDC_STATION_POS="${VEDC_STATION_POS:-10.883438,106.795745}"
# Side by side west of the origin, bows east: PHAO-01 at the origin as in run_boat.sh, then 4 m north, south, ...
read -r -a STARTS <<< "${FLEET_STARTS:-0,0,90 -3,4,90 -3,-4,90 -6,0,90}"

rm -rf "$RUN"; mkdir -p "$RUN/models" "$RUN/worlds"
# The buoy without its ArduPilot link, once per boat: vedc_buoy for PHAO-01, vedc_buoy_n for the others,
# each with topics of its own. The meshes stay those of model://vedc_buoy.
cp -r "$VEDC_SIM_REPO/models/vedc_buoy" "$RUN/models/"
sed -i '/<plugin name="ArduPilotPlugin"/,/<\/plugin>/d' "$RUN/models/vedc_buoy/model.sdf"
model_of() { [ "$1" = 1 ] && echo vedc_buoy || echo "vedc_buoy_$1"; }
for n in $(seq 2 "$BOATS"); do
    m="$(model_of "$n")"
    mkdir -p "$RUN/models/$m"
    # Every name tied to the model: the sensors' topics, the thrusters' namespace, the pods' topics, and the
    # link the wave forces act on (without it the hull gets no buoyancy or drag and is thrown about)
    sed -e "s#<model name=\"vedc_buoy\">#<model name=\"$m\">#" -e "s#<topic>vedc_buoy/#<topic>$m/#g" \
        -e "s#/model/vedc_buoy/#/model/$m/#g" -e "s#<namespace>vedc_buoy</namespace>#<namespace>$m</namespace>#g" \
        -e "s#<enable>vedc_buoy::#<enable>$m::#g" "$RUN/models/vedc_buoy/model.sdf" > "$RUN/models/$m/model.sdf"
    if grep -q ">vedc_buoy[<:/]" "$RUN/models/$m/model.sdf"; then
        echo "model $m still names vedc_buoy:" >&2; grep -n ">vedc_buoy[<:/]" "$RUN/models/$m/model.sdf" >&2; exit 1
    fi
    sed "s#<name>vedc_buoy</name>#<name>$m</name>#" "$VEDC_SIM_REPO/models/vedc_buoy/model.config" > "$RUN/models/$m/model.config"
done
python3 - "$VEDC_SIM_REPO/worlds/$SDF" "$RUN/worlds/$SDF" "$BOATS" "${STARTS[@]}" <<'EOF'
import math, sys
src, dst, boats, starts = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4:]
text = open(src).read()
marker = "<uri>model://vedc_buoy</uri>"
assert marker in text, "the world has no vedc_buoy"
extra = ""
for n in range(2, boats + 1):
    east, north, heading = (float(v) for v in starts[n - 1].split(","))
    yaw = math.radians(90.0 - heading)
    extra += (f"\n    <include><name>vedc_buoy_{n}</name><pose>{east} {north} 0 0 0 {yaw:.4f}</pose>"
              f"<uri>model://vedc_buoy_{n}</uri></include>")
end = text.index("</include>", text.index(marker)) + len("</include>")
open(dst, "w").write(text[:end] + extra + text[end:])
EOF
export GZ_SIM_RESOURCE_PATH="$RUN/models:$RUN/worlds:$GZ_SIM_RESOURCE_PATH"
WORLD_NAME="$(sed -n 's/.*<world name="\([^"]*\)".*/\1/p' "$RUN/worlds/$SDF" | head -n 1)"

PIDS=""
GZ=""
cleanup() {
    trap - EXIT INT TERM
    echo "stopping the fleet"
    [ -n "$PIDS" ] && kill -INT $PIDS 2>/dev/null
    sleep 2
    [ -n "$PIDS" ] && kill $PIDS 2>/dev/null
    [ -n "$GZ" ] && kill -- -"$GZ" 2>/dev/null     # gz sim's server runs in its process group
    sleep 1
    [ -n "$GZ" ] && kill -9 -- -"$GZ" 2>/dev/null
}
trap cleanup EXIT
trap 'exit 130' INT TERM

GZ_ARGS=(-r -v 1)
[ "$HEADLESS" = 1 ] && GZ_ARGS=(-s -r -v 1)
cd "$RUN"
setsid gz sim "${GZ_ARGS[@]}" "$RUN/worlds/$SDF" > "$RUN/gz.log" 2>&1 < /dev/null &
GZ=$!
echo "Gazebo ($WORLD, $BOATS boats) loading"
last="$(model_of "$BOATS")"
for _ in $(seq 120); do
    gz topic -l 2>/dev/null | grep -q "$last/lidar" && break
    sleep 1
done
sleep 3

# The route, shore-link and slot code of both firmwares, for this machine (tools/hw_standins.sh).
gcc -std=c11 -O1 -Wall -shared -fPIC -I"$REPO/STM32F4/App" -I"$REPO/esp32-lora-station/main" \
    "$VEDC_SIM_REPO/tools/route_radio_shim.c" "$REPO/STM32F4/App/route_link.c" "$REPO/STM32F4/App/shore_link.c" \
    "$REPO/STM32F4/App/event_queue.c" "$REPO/STM32F4/App/lora_slot.c" "$REPO/esp32-lora-station/main/route_downlink.c" \
    "$REPO/esp32-lora-station/main/lora_payload.c" "$REPO/esp32-lora-station/main/victim_claims.c" -lm -o "$RUN/librouteradio.so" || exit 1

STATION_ARGS=()
[ "$CLAIMS" = 1 ] || STATION_ARGS+=(--no-claims)
python3 -u "$VEDC_SIM_REPO/tools/shore_station_sim.py" --radio-lib "$RUN/librouteradio.so" --cloud "$CLOUD" --collisions \
    --dir "$RUN" --station-pos "$VEDC_STATION_POS" "${STATION_ARGS[@]}" > "$RUN/station_sim.log" 2>&1 &
PIDS="$PIDS $!"

BOARD_ARGS=(--collisions)
[ "$TDMA" = 1 ] && BOARD_ARGS+=(--tdma)
for n in $(seq 1 "$BOATS"); do
    m="$(model_of "$n")" id="PHAO-0$n" dir="$RUN/boat_$n"
    mkdir -p "$dir/logs"
    python3 -u "$VEDC_SIM_REPO/tools/ld14_emulator.py" --pty "$dir/lidar_pty" --topic "/$m/lidar" > "$dir/lidar_sim.log" 2>&1 &
    PIDS="$PIDS $!"
    python3 -u "$VEDC_SIM_REPO/tools/stm32_board_sim.py" --pty "$dir/stm32_pty" --dir "$dir" --radio-lib "$RUN/librouteradio.so" \
        --home "$HOME_LATLON" --world "$WORLD_NAME" --model "$m" --buoy-id "$id" "${BOARD_ARGS[@]}" > "$dir/stm32_sim.log" 2>&1 &
    PIDS="$PIDS $!"
    printf 'serial:\n  stm32_port: %s\n  lidar_port: %s\n  lidar_model: ld14\ncamera:\n  gazebo_topic: /%s/camera\n  pan_topic: /model/%s/joint/camera_pan_joint/cmd_pos\nlidar:\n  gazebo_topic: /%s/lidar\nvehicle:\n  lidar_yaw_offset_deg: 0.0\n  lidar_mask: []\n  gps_required: true\nlogging:\n  directory: %s\n' \
        "$dir/stm32_pty" "$dir/lidar_pty" "$m" "$m" "$m" "$dir/logs" > "$dir/boat.yaml"
    [ -n "$HOLD" ] && printf 'follow:\n  max_hold_s: %s\n' "$HOLD" >> "$dir/boat.yaml"
done
for n in $(seq 1 "$BOATS"); do
    for _ in $(seq 40); do [ -e "$RUN/boat_$n/lidar_pty" ] && [ -e "$RUN/boat_$n/stm32_pty" ] && break; sleep 0.5; done
done

cd "$REPO/Rasp_Pi"
for n in $(seq 1 "$BOATS"); do
    dir="$RUN/boat_$n" pi_args=()
    [ "$VISION" = first ] && [ "$n" != 1 ] && pi_args+=(--no-camera)
    [ "$VIEWER" = 1 ] && pi_args+=(--viewer "$((8089 + n))")
    overlays=(--overlay config/boat_stm32.yaml --overlay config/sim.yaml)
    [ -n "$OVERLAY" ] && [ -f "$OVERLAY" ] && overlays+=(--overlay "$OVERLAY")
    setsid python3 -u main.py "${overlays[@]}" --overlay "$dir/boat.yaml" "${pi_args[@]}" > "$dir/pi.log" 2>&1 < /dev/null &
    PIDS="$PIDS $!"
done
echo "fleet up: $BOATS boats (PHAO-01..PHAO-0$BOATS), $([ "$TDMA" = 1 ] && echo 'time slots' || echo 'no time slots'), logs in $RUN"
echo "  dashboard: $CLOUD/admin   station: $VEDC_STATION_POS"
[ "$VIEWER" = 1 ] && for n in $(seq 1 "$BOATS"); do echo "  viewer PHAO-0$n: http://127.0.0.1:$((8089 + n))/"; done
touch "$RUN/ready"
if [ -n "$DURATION" ]; then
    sleep "$DURATION"
else
    wait $GZ
fi
