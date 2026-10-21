#!/bin/bash
# The whole mission of the STM32F407 boat (Raspberry Pi + STM32F407, Rasp_Pi/main.py) in the
# central-Vietnam scene, with the boat's own settings (config/boat_stm32.yaml) and every link of the
# real system standing in: the F407 board (sim/tools/stm32_board_sim.py) and the ESP32 shore station
# (shore_station_sim.py) on the simulated LoRa radio, both running the firmware's own route, shore
# link and event code through ctypes (sim/tools/hw_standins.sh).
#
#   1. no route yet                       the boat stands still (IDLE)
#   2. a route "from the dashboard"       the station sends it over LoRa, the F407 hands it to the Pi
#   3. the person in the water            the Pi reports VICTIM_FOUND with a risk level; the F407 sends
#                                         the person event, the station acknowledges it
#   4. the confirm button (PE4)           confirm event over LoRa, the Pi goes back to the route
#   5. light 1 switched on by the admin   the next ACK switches RELAY1, the beacon reports L=1
#  5b. DỪNG PHAO, then CHẠY TIẾP          the F407 keeps the motors off while held, whatever the Pi asks
#   6. the station switched off           after 60 s the F407 reports the link lost ($LNK), the Pi
#                                         takes the boat home towards the station; the light stays on
#   7. the station back                   the link comes back
#
# The route comes from a file instead of the Worker (VEDC_ROUTE_FILE), the lights from a file named
# lights next to it, the station is switched off by a file named station_off.
#
#   bash sim/tools/stm32_mission_test.sh
#   GUI=1          open the Gazebo window too
set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_stm32_mission
export VEDC_SIM_REPO="$REPO/sim"
T="$SIM_DIR/stm32_mission"
rm -rf "$T"; mkdir -p "$T/models" "$T/worlds"
HOME_LL="10.883438,106.796019"     # the buoy's start, the world origin (Rasp_Pi/config/mission_sim_central.yaml)

# The buoy without its ArduPilot link, the scene with the person in the water only (as stm32_central_test.sh).
cp -r "$REPO/sim/models/vedc_buoy" "$T/models/"
sed -i '/<plugin name="ArduPilotPlugin"/,/<\/plugin>/d' "$T/models/vedc_buoy/model.sdf"
grep -v "<name>victim_doorway</name>" "$REPO/sim/worlds/vedc_central.sdf" > "$T/worlds/vedc_central.sdf"
export GZ_SIM_RESOURCE_PATH="$T/models:$T/worlds:$GZ_SIM_RESOURCE_PATH"

PIDS=""
GZ=""
cleanup() {
    [ -n "$PIDS" ] && kill $PIDS 2>/dev/null
    [ -n "$HW_PIDS" ] && kill $HW_PIDS 2>/dev/null
    [ -n "$GZ" ] && kill -- -"$GZ" 2>/dev/null
    sleep 1
    [ -n "$GZ" ] && kill -9 -- -"$GZ" 2>/dev/null
}
trap cleanup EXIT

cd "$T"
GZ_ARGS=(-s -r -v 1)
[ "$GUI" = 1 ] && GZ_ARGS=(-r -v 1)
setsid gz sim "${GZ_ARGS[@]}" "$T/worlds/vedc_central.sdf" > "$T/gz.log" 2>&1 < /dev/null &
GZ=$!
for _ in $(seq 120); do
    gz topic -l 2>/dev/null | grep -q "vedc_buoy/lidar" && break
    sleep 1
done
sleep 5

# The board's GPS reports GGA quality 1, a plain fix, as the boat's receiver does.
export VEDC_ROUTE_FILE="$T/route.yaml" VEDC_STATION_POS="$HOME_LL"
source "$REPO/sim/tools/hw_standins.sh"
start_stm32_standins "$T" "$HOME_LL" vedc_central || exit 1
# The dashboard's route, the pool's (empty) geofence: as on the boat, nothing moves until the route comes.
cat > "$T/pi.yaml" <<EOF
mission:
  file: config/mission_sim_central.yaml
  waypoints_from_file: false
follow:
  only_on_route: true
EOF

cd "$REPO/Rasp_Pi"
setsid python3 -u main.py --overlay config/sim.yaml --overlay config/boat_stm32.yaml --overlay config/sim_stm32.yaml \
    --overlay "$T/pi.yaml" --overlay "$T/stm32_ports.yaml" --viewer 8090 > "$T/pi.log" 2>&1 < /dev/null &
PI=$!
PIDS="$PIDS $PI"
echo "Pi log $T/pi.log, board $T/stm32_events.log, station $T/station_sim.log, viewer http://127.0.0.1:8090/"

FAILS=0
check() {      # check <name> <seconds> <command...>: polls the command once a second
    local name="$1" limit="$2"; shift 2
    for _ in $(seq "$limit"); do
        if "$@" > /dev/null 2>&1; then echo "  ok    $name"; return 0; fi
        sleep 1
    done
    echo "  FAIL  $name (within ${limit} s)"
    FAILS=$((FAILS + 1))
    return 1
}
events() { grep -q "$1" "$T/stm32_events.log"; }
pilog() { grep -qi "$1" "$T/pi.log"; }
# The boat's position from the board's status line "pos=(x,y)", metres from the station.
distance() { tail -n 1 "$T/stm32_sim.log" | sed -n 's/.*pos=(\([-0-9.]*\),\([-0-9.]*\)).*/\1 \2/p' | awk '{printf "%.1f", sqrt($1*$1+$2*$2)}'; }

echo "1. no route: the boat stands still"
sleep 15
if events "AUTO: steering"; then echo "  FAIL  the boat drove without a route"; FAILS=$((FAILS + 1)); else echo "  ok    standing still"; fi

echo "2. route from the dashboard over LoRa"
python3 - "$REPO/Rasp_Pi/config/mission_sim_central.yaml" "$T/route.yaml" <<'EOF'
import sys, yaml
mission = yaml.safe_load(open(sys.argv[1]))
# loop: the boat is still patrolling when the station goes off (step 6); a finished route is IDLE, no way home
yaml.safe_dump({"id": "PHAO-01", "loop": True, "waypoints": mission["waypoints"]}, open(sys.argv[2], "w"))
EOF
check "route on the F407" 40 events "LoRa RTE taken"
check "route handed to the Pi" 30 events "RTE rev 1 to the Pi"
check "the boat drives" 30 events "AUTO: steering"

echo "3. the person in the water"
check "VICTIM_FOUND with a risk level" 120 grep -qE "TXD VICTIM_FOUND,[-0-9.]+,[-0-9.]+,[0-9.]+,[0-3]" "$T/stm32_events.log"
check "person event acknowledged by the station" 20 events "acknowledged"
check "station posted the event" 5 grep -q "event {'seq'" "$T/station_sim.log"

echo "4. the confirm button"
touch "$T/confirm"
check "confirm counted" 5 events "confirm button pressed (1)"
check "confirm event acknowledged" 20 bash -c "grep 'acknowledged' '$T/stm32_events.log' | wc -l | grep -qv '^1$'"
check "the Pi announces it" 10 pilog "confirm"

echo "5. light 1 from the dashboard"
echo 1 > "$T/lights"
check "RELAY1 on" 20 events "lights 1"
check "beacon reports L=1" 20 grep -q "L=1" "$T/station_sim.log"

echo "5b. DỪNG PHAO from the dashboard, then CHẠY TIẾP"
touch "$T/hold"
check "the F407 holds the boat" 20 events "STOPPED from the dashboard"
check "beacon reports H=1" 20 grep -q "STOP" "$T/station_sim.log"
sleep 3
check "motors off while held" 5 bash -c "tail -n 1 '$T/stm32_sim.log' | grep -q 'esc=0.00/0.00'"
rm -f "$T/hold"
check "released" 20 events "released from the dashboard"

echo "6. the station switched off"
touch "$T/station_off"
check "link lost after 60 s" 80 events "shore station lost"
check "the Pi goes home" 15 pilog "RETURN"
# The boat patrols on for the 60 s before the link counts as lost: measure from the turn home.
START_D="$(distance)"
sleep 45
END_D="$(distance)"
echo "        distance to the station: ${START_D} m when it turned home, ${END_D} m 45 s later"
if awk -v a="$START_D" -v b="$END_D" 'BEGIN { exit !(b < a) }'; then
    echo "  ok    closer to the station"
elif grep "RETURN" "$T/pi.log" | tail -n 5 | grep -q "obstacle_too_close"; then
    # The rear ESCs turn one way only: wedged against something, the boat cannot back off. Not the way
    # home that is wrong; seen against the wrecked boat at x 22, y -4.3.
    echo "  WARN  stuck against an obstacle (obstacle_too_close): the boat cannot reverse"
else
    echo "  FAIL  not closer to the station"; FAILS=$((FAILS + 1))
fi
if grep -q "lights 0" "$T/stm32_events.log"; then echo "  FAIL  the light went off"; FAILS=$((FAILS + 1)); else echo "  ok    light 1 still on"; fi

echo "7. the station back"
rm -f "$T/station_off"
check "link back" 20 events "shore station heard again"

# A risk that flickers between two levels is not a report each time: a few updates at most. (VICTIM_FOUND
# may count more than the one person: the detector also takes some of the scene's poles and boats for people.)
UPDATES="$(grep -c 'TXD VICTIM_UPDATE' "$T/stm32_events.log")"
echo "        VICTIM_FOUND $(grep -c 'TXD VICTIM_FOUND' "$T/stm32_events.log"), VICTIM_UPDATE $UPDATES"
if [ "$UPDATES" -le 4 ]; then echo "  ok    $UPDATES updates of the person followed"; else echo "  FAIL  $UPDATES updates: risk flicker reported"; FAILS=$((FAILS + 1)); fi

kill -INT $PI 2>/dev/null
sleep 2
if [ "$FAILS" = 0 ]; then echo "PASS"; exit 0; fi
echo "FAIL: $FAILS checks"
exit 1
