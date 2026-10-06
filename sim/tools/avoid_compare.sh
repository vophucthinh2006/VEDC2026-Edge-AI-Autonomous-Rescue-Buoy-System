#!/bin/bash
# One run of one avoidance method over one route of the flood scene, on SITL instance 1, with the
# same measurements for all of them. No person detection: only getting past the obstacles.
#
#   bash sim/tools/avoid_compare.sh <bendyruler|bapf|paper|vo> [street|gap]
#     bendyruler  what the boat does today: mission in AUTO, the Pi feeds the LiDAR, the autopilot avoids
#     bapf        Pi steers (GUIDED): potential field of the STM32 runtime, settings.yaml values
#     paper       Pi steers: biased potential field of Jo et al. 2022, with the CPA gate
#     vo          Pi steers: velocity obstacle of the same paper
#   street  down the street to 38.5 m and back, past the tree, the car and the pole (default)
#   gap     waypoints that send the boat between the car and the north row, 4.3 m wide
#
# Prints one RESULT line. Rasp_Pi/pi_steer.py holds the Pi-steered navigators' coefficients.
set -o pipefail
METHOD="${1:?method: bendyruler, bapf, paper or vo}"
ROUTE="${2:-street}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_compare
T="$SIM_DIR/avoid_compare"
rm -rf "$T"; mkdir -p "$T/models" "$T/run"
cp -r "$REPO/sim/models/vedc_buoy" "$T/models/"
sed -i 's#<fdm_port_in>9002</fdm_port_in>#<fdm_port_in>9012</fdm_port_in>#' "$T/models/vedc_buoy/model.sdf"
export GZ_SIM_RESOURCE_PATH="$T/models:$GZ_SIM_RESOURCE_PATH"
case "$ROUTE" in
    street) LEGS="0,38.5;0,2" ;;
    gap)    LEGS="4,13;4,23;0,30" ;;
    *) echo "unknown route $ROUTE" >&2; exit 2 ;;
esac

cd "$T"
setsid gz sim -s -r -v 1 vedc_flood.sdf > "$T/gz.log" 2>&1 < /dev/null &
GZ=$!
sleep 15
cd "$T/run"
PARAMS=(--add-param-file="$REPO/sim/params/vedc_buoy.parm")
# The autopilot's avoidance (and its pre-arm check for obstacle data) only for the method that uses it.
[ "$METHOD" = bendyruler ] && PARAMS+=(--add-param-file="$REPO/sim/params/avoidance.parm")
setsid sim_vehicle.py -v Rover -f rover-skid --model JSON -I1 "${PARAMS[@]}" \
    --custom-location=10.883438,106.796019,10,90 \
    --no-wsl2-network --no-rebuild --wipe-eeprom \
    --out=udp:127.0.0.1:14561 --mavproxy-args="--daemon" \
    > "$T/sitl.log" 2>&1 < /dev/null &
SITL=$!
sleep 28
cd "$REPO/Rasp_Pi"
if [ "$METHOD" = bendyruler ]; then
    # The rescue runtime with nobody to find and the camera still: the LiDAR feed and the blocked-waypoint rules.
    printf 'sim:\n  victims: []\ncamera:\n  scan:\n    enabled: false\n' > "$T/nobody.yaml"
    setsid python3 -u rescue_main.py --overlay config/sim.yaml --overlay "$T/nobody.yaml" --detector simulated \
        --mavlink tcp:127.0.0.1:5773 > "$T/pi.log" 2>&1 < /dev/null &
else
    setsid python3 -u pi_steer.py --navigator "$METHOD" --route "$LEGS" --overlay config/sim.yaml \
        --mavlink tcp:127.0.0.1:5773 > "$T/pi.log" 2>&1 < /dev/null &
fi
PI=$!
cleanup() {
    kill -- -$PI -$SITL -$GZ 2>/dev/null
    sleep 2
    pkill -f "ardurover.*-I1"; pkill -f "mavproxy.*5770"
}
trap cleanup EXIT

timeout 330 python3 -u - "$METHOD" "$ROUTE" "$LEGS" "$T/track.csv" <<'EOF'
import math
import sys
import time

from pymavlink import mavutil

METHOD, ROUTE, LEGS, TRACK = sys.argv[1], sys.argv[2], [tuple(float(x) for x in leg.split(",")) for leg in sys.argv[3].split(";")], sys.argv[4]
M = mavutil.mavlink
HOME = (10.883438, 106.796019)
K = 111320 * math.cos(math.radians(HOME[0]))
# sim/worlds/vedc_flood.sdf, as (east, north). The person in the water counts: the boat must not run them over.
CIRCLES = {"tree": (10.0, -1.0, 0.9), "pole": (31.0, 0.8, 0.2), "person": (27.0, -0.5, 0.3)}
RECTS = {"car": (16.1, 19.9, 0.15, 1.85), "end house": (43.2, 49.8, -3.8, 3.8),
         "north row": (2.5, 39.5, 6.2, 12.0), "south row": (1.0, 37.5, -12.0, -6.2)}
LENGTH_M = 1.1                     # the boat: Jo et al. count the time spent closer than 1 L and 2 L
LIMIT_S = 240

m = mavutil.mavlink_connection("udpin:0.0.0.0:14561", source_system=250)
while True:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=30)
    if hb is None:
        sys.exit("FAIL: no heartbeat")
    if hb.autopilot == M.MAV_AUTOPILOT_ARDUPILOTMEGA and hb.get_srcComponent() == 1:
        m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
        break

if METHOD == "bendyruler":
    items = [HOME] + [(HOME[0] + n / 111320, HOME[1] + e / K) for n, e in LEGS]
    m.mav.mission_count_send(m.target_system, m.target_component, len(items), 0)
    while True:
        r = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=10)
        if r is None:
            sys.exit("FAIL: mission upload timed out")
        if r.get_type() == "MISSION_ACK":
            break
        la, lo = items[r.seq]
        m.mav.mission_item_int_send(m.target_system, m.target_component, r.seq, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                    M.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, int(la * 1e7), int(lo * 1e7), 0, 0)
t0 = time.time()
while time.time() - t0 < 90:
    m.mav.command_long_send(m.target_system, m.target_component, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    a = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if a and a.command == M.MAV_CMD_COMPONENT_ARM_DISARM and a.result == 0:
        break
    time.sleep(2)
else:
    sys.exit("FAIL: could not arm")
if METHOD == "bendyruler":
    m.set_mode("AUTO")              # the Pi-steered methods put the boat in GUIDED themselves once it is armed

def to_circle(e, n, c):
    return math.hypot(e - c[0], n - c[1]) - c[2]

def to_rect(e, n, r):
    return math.hypot(max(r[0] - e, 0.0, e - r[1]), max(r[2] - n, 0.0, n - r[3]))

track, notes, skipped = [], [], 0
start, done, mode = time.time(), None, ""
while time.time() - start < LIMIT_S and done is None:
    msg = m.recv_match(type=["GLOBAL_POSITION_INT", "STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=2)
    if msg is None:
        continue
    kind = msg.get_type()
    if kind == "HEARTBEAT":
        if msg.get_srcSystem() == m.target_system and msg.get_srcComponent() == 1:
            mode = mavutil.mode_string_v10(msg)
    elif kind == "STATUSTEXT":
        if any(word in msg.text for word in ("Reached", "REACHED", "Complete", "DONE", "BLOCKED")):
            notes.append(msg.text)
            print(f"  [{time.time() - start:5.1f} s] {msg.text}")
        skipped += "BLOCKED" in msg.text
        # "BLOCKED, HOLD": the Pi gave the last waypoint up as blocked and stopped the boat there.
        if "ROUTE DONE" in msg.text or "Mission Complete" in msg.text or "BLOCKED, HOLD" in msg.text:
            done = time.time() - start
    else:
        e, n = (msg.lon / 1e7 - HOME[1]) * K, (msg.lat / 1e7 - HOME[0]) * 111320
        clear = min([to_circle(e, n, c) for c in CIRCLES.values()] + [to_rect(e, n, r) for r in RECTS.values()])
        track.append((time.time() - start, e, n, math.hypot(msg.vx, msg.vy) / 100.0, msg.hdg / 100.0, clear))

with open(TRACK, "w") as f:
    f.write("t,east,north,speed,heading,clearance\n" + "".join(f"{t:.2f},{e:.2f},{n:.2f},{v:.2f},{h:.0f},{c:.2f}\n" for t, e, n, v, h, c in track))
moving = [row for row in track if row[0] >= 3.0]              # the first seconds are the start from rest
length = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(track, track[1:]))
steps = list(zip(moving, moving[1:]))
under_1l = sum(b[0] - a[0] for a, b in steps if a[5] < LENGTH_M)
under_2l = sum(b[0] - a[0] for a, b in steps if a[5] < 2 * LENGTH_M)
stuck = sum(b[0] - a[0] for a, b in steps if a[3] < 0.1)
wobble = sum(abs((b[4] - a[4] + 180.0) % 360.0 - 180.0) for a, b in steps)
closest = min((row[5] for row in track), default=float("nan"))
last = track[-1] if track else (0, 0, 0, 0, 0, 0)
goal_n, goal_e = LEGS[-1]
end_gap = math.hypot(last[1] - goal_e, last[2] - goal_n)
outcome = "NOT finished" if done is None else "reached" if end_gap <= 3.0 and not skipped else f"finished, {skipped} waypoint(s) given up"
print(f"RESULT {METHOD:10s} {ROUTE:6s} | {outcome} | time {done if done is not None else LIMIT_S:5.0f} s"
      f" | path {length:5.1f} m | closest {closest:4.2f} m | under 1L {under_1l:5.1f} s | under 2L {under_2l:5.1f} s"
      f" | stopped {stuck:5.1f} s | turning {wobble:5.0f} deg | skipped {skipped} | ended {end_gap:4.1f} m from the goal in {mode}")
EOF
RC=$?
grep -E "Traceback|Error|navigator|obstacle_too_close|boxed_in" "$T/pi.log" | cut -c1-160 | sort | uniq -c | sort -rn | head -5
exit $RC
