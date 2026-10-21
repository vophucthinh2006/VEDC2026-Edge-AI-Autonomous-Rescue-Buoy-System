#!/bin/bash
# The STM32F407 boat (Raspberry Pi + STM32F407) in Gazebo, driven from the dashboard: sim/run_boat.sh
# --stm32 in the central-Vietnam scene (or the --world given), its simulated shore station reading the
# route from the Worker, Rasp_Pi/main.py with the viewer. Prints the dashboard's and the viewer's links
# once main.py serves the viewer. Started by scripts/stm32/sim_central.bat, which starts the Worker.
#
#   VEDC_INGEST_TOKEN=<INGEST_TOKEN of the Worker> bash sim/tools/stm32_web_sim.sh [--world=central] [--headless]
#   VEDC_CLOUD_URL   the Worker, default http://127.0.0.1:8787
#   VIEWER_PORT      main.py's viewer, default 8090
#   VEDC_STATION_POS the shore station, lat,lon; default on the bank of Ho Da 01
#   BEACON_BUOYS     buoys that only beacon (tools/beacon_buoy_sim.py), "id,lat,lon,heading,battery" separated by
#                    spaces; default PHAO-02 and PHAO-03 next to PHAO-01 on Ho Da 01; "" for none
#
# Ctrl+C stops the simulator. Logs in ~/vedc_sim/sitl_run (pi.log, stm32_sim.log, station_sim.log).
set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
[ -n "$VEDC_INGEST_TOKEN" ] || { echo "set VEDC_INGEST_TOKEN to the Worker's INGEST_TOKEN" >&2; exit 2; }
WEB="${VEDC_CLOUD_URL:-http://127.0.0.1:8787}"
VIEWER_PORT="${VIEWER_PORT:-8090}"
export VEDC_CLOUD_URL="$WEB"

ARGS=()
WORLD_GIVEN=0
for arg in "$@"; do
    case "$arg" in
        --world=*) WORLD_GIVEN=1 ;;
    esac
    ARGS+=("$arg")
done
[ "$WORLD_GIVEN" = 1 ] || ARGS+=(--world=central)

# Two at once share the board's pseudo-terminal, the LoRa ports, the viewer's port and pi.log: both break.
if pgrep -f "sim/run_boat.sh --stm32" > /dev/null; then
    echo "a simulation of the STM32 boat is running already (sim/run_boat.sh --stm32): stop it first" >&2
    exit 1
fi

# The shore station on the bank of Ho Da 01, 15 m inland from the shore nearest to the boat's start (217 m
# south-south-east of it, OpenStreetMap relation 3634964), as it would stand: the boat and the buoys are then
# well inside its LoRa coverage (web/js/state.js LORA_GOOD_M). Its position goes to the dashboard and, in every
# ACK, to the boat as the way home.
export VEDC_STATION_POS="${VEDC_STATION_POS:-10.881602,106.797024}"
# PHAO-01 is the simulated boat, at the home position (run_boat.sh HOME_LOCATION, the middle of Ho Da 01);
# PHAO-02 and PHAO-03 stand about 70 m north-east and south-east of it, on the same lake.
BEACON_BUOYS="${BEACON_BUOYS-PHAO-02,10.883888,106.796469,90,72 PHAO-03,10.882988,106.796469,270,64}"

RUN="$SIM_DIR/sitl_run"
PI_LOG="$RUN/pi.log"
rm -f "$PI_LOG"   # the wait below must see this run's main.py, not the last one's

links() {
    echo
    echo "=================================================================="
    echo "  $1"
    echo "  Relatives : $WEB/  (the guest page, no sign-in)"
    echo "  Admin     : $WEB/admin  (the console)"
    echo "  Viewer    : http://127.0.0.1:$VIEWER_PORT/"
    echo "  Sign-in   : on /admin, 'Dang nhap admin (thu nghiem)' (npm run dev:idp)"
    echo "  Buoys     : PHAO-01 the simulated boat${BEACON_BUOYS:+; beacon only: $(for b in $BEACON_BUOYS; do printf '%s ' "${b%%,*}"; done)}"
    echo "  Route     : select PHAO-01, Lo trinh tab, tap the map for points, Gui toi phao"
    echo "  Station   : $VEDC_STATION_POS (on the bank), LoRa circles 2 km good, 5 km weak"
    echo "  Logs      : $RUN"
    echo "=================================================================="
    echo
}

# The links once the Pi is up, from the side: run_boat.sh stays in the foreground so that Ctrl+C
# reaches it and it stops Gazebo, the stand-ins and main.py the way it always does.
(
    for _ in $(seq 300); do
        grep -q "viewer: http" "$PI_LOG" 2>/dev/null && { links "READY: the boat waits for a route from the dashboard"; exit 0; }
        sleep 1
    done
    echo "main.py did not start its viewer within 300 s: see $PI_LOG" >&2
) &
WATCH=$!

BEACONS=""
if [ -n "$BEACON_BUOYS" ]; then
    BUOY_ARGS=()
    for buoy in $BEACON_BUOYS; do BUOY_ARGS+=(--buoy "$buoy"); done
    mkdir -p "$RUN"
    python3 -u "$REPO/sim/tools/beacon_buoy_sim.py" "${BUOY_ARGS[@]}" > "$RUN/beacon_buoys.log" 2>&1 &
    BEACONS=$!
fi

STM32_PI_ARGS="--viewer $VIEWER_PORT ${STM32_PI_ARGS:-}" bash "$REPO/sim/run_boat.sh" --stm32 "${ARGS[@]}"
RC=$?
kill "$WATCH" $BEACONS 2>/dev/null
exit $RC
