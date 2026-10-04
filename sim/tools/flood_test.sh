#!/bin/bash
# The flood scene end to end, on SITL instance 1 so it can run beside a run_boat.sh session:
# the boat searches down the flooded street (sim/worlds/vedc_flood.sdf) and comes back. The Pi
# code must feed the LiDAR to the autopilot so it goes around the tree, the car and the pole
# and returns to the route, find the person in the water and the one on the roof, and never
# touch anything.
#
#   bash sim/tools/flood_test.sh [simulated|camera] [noscan]
#     simulated  geometric detector (default): tests the avoidance and the rescue logic
#     camera     the real TFLite model on the Gazebo camera: tests the whole perception chain
set -o pipefail
DETECTOR="${1:-simulated}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_flood_test
T="$SIM_DIR/flood_test"
rm -rf "$T"; mkdir -p "$T/models" "$T/run"
cp -r "$REPO/sim/models/vedc_buoy" "$T/models/"
sed -i 's#<fdm_port_in>9002</fdm_port_in>#<fdm_port_in>9012</fdm_port_in>#' "$T/models/vedc_buoy/model.sdf"
export GZ_SIM_RESOURCE_PATH="$T/models:$GZ_SIM_RESOURCE_PATH"

cd "$T"
setsid gz sim -s -r -v 1 vedc_flood.sdf > "$T/gz.log" 2>&1 < /dev/null &
GZ=$!
sleep 15
cd "$T/run"
setsid sim_vehicle.py -v Rover -f rover-skid --model JSON -I1 \
    --add-param-file="$REPO/sim/params/vedc_buoy.parm" \
    --add-param-file="$REPO/sim/params/avoidance.parm" \
    --custom-location=10.883438,106.796019,10,90 \
    --no-wsl2-network --no-rebuild --wipe-eeprom \
    --out=udp:127.0.0.1:14561 --mavproxy-args="--daemon" \
    > "$T/sitl.log" 2>&1 < /dev/null &
SITL=$!
sleep 28
cd "$REPO/Rasp_Pi"
# Instance 1 shifts SITL's ports by 10: its SERIAL2 (TELEM2) is TCP 5773.
NO_SCAN=()
[ "$2" = noscan ] && NO_SCAN=(--no-scan)       # camera fixed ahead, for comparison
setsid python3 -u rescue_main.py --overlay config/sim.yaml --overlay config/sim_flood.yaml \
    --detector "$DETECTOR" --mavlink tcp:127.0.0.1:5773 "${NO_SCAN[@]}" > "$T/pi.log" 2>&1 < /dev/null &
PI=$!
cleanup() {
    kill -- -$PI -$SITL -$GZ 2>/dev/null
    sleep 2
    pkill -f "ardurover.*-I1"; pkill -f "mavproxy.*5770"
}
trap cleanup EXIT

timeout 560 python3 -u - "$T/track.csv" <<'EOF'
import math
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink
HOME = (10.883438, 106.796019)
K = 111320 * math.cos(math.radians(HOME[0]))
# sim/worlds/vedc_flood.sdf, as (east, north): Gazebo x, y.
CIRCLES = {"tree": (10.0, -1.0, 0.9), "pole": (31.0, 0.8, 0.2)}
RECTS = {"car": (16.1, 19.9, 0.15, 1.85), "end house": (43.2, 49.8, -3.8, 3.8),
         "north row": (2.5, 39.5, 6.2, 12.0), "south row": (1.0, 37.5, -12.0, -6.2)}
VICTIMS = {"water": (27.0, -0.5), "roof": (43.5, 0.5)}
# North, east: down the street, then RTL. The waypoint stays 4.7 m short of the end house:
# BendyRuler also probes 2 m past its target, and with a wall there it does not settle on the waypoint.
LEGS = [(0.0, 38.5)]
LOOK_S = 10                        # wait at the end of the street, still searching, before turning home
CLEARANCE_M = 0.8                  # from the boat's GPS position; the hulls reach 0.55 m from it

m = mavutil.mavlink_connection("udpin:0.0.0.0:14561", source_system=250)
while True:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=30)
    if hb is None:
        sys.exit("FAIL: no heartbeat")
    if hb.autopilot == M.MAV_AUTOPILOT_ARDUPILOTMEGA and hb.get_srcComponent() == 1:
        m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
        break

def latlon(n, e):
    return HOME[0] + n / 111320, HOME[1] + e / K

items = [(M.MAV_CMD_NAV_WAYPOINT, 0, *HOME)] + [(M.MAV_CMD_NAV_WAYPOINT, LOOK_S, *latlon(n, e)) for n, e in LEGS]
items.append((M.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0))
m.mav.mission_count_send(m.target_system, m.target_component, len(items), 0)
while True:
    r = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=10)
    if r is None:
        sys.exit("FAIL: mission upload timed out")
    if r.get_type() == "MISSION_ACK":
        break
    cmd, delay, la, lo = items[r.seq]
    m.mav.mission_item_int_send(m.target_system, m.target_component, r.seq, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                cmd, 0, 1, delay, 0, 0, 0, int(la * 1e7), int(lo * 1e7), 0, 0)
print("mission uploaded")

t0 = time.time()
while time.time() - t0 < 90:
    m.mav.command_long_send(m.target_system, m.target_component, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    a = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if a and a.command == M.MAV_CMD_COMPONENT_ARM_DISARM and a.result == 0:
        break
    time.sleep(2)
else:
    sys.exit("FAIL: could not arm")
m.set_mode("AUTO")
print("armed, AUTO")

def to_circle(e, n, c):
    return math.hypot(e - c[0], n - c[1]) - c[2]

def to_rect(e, n, r):
    return math.hypot(max(r[0] - e, 0.0, e - r[1]), max(r[2] - n, 0.0, n - r[3]))

modes, texts, track = [], [], []
clearance = {name: 1e9 for name in list(CIRCLES) + list(RECTS)}
victim_range = {name: 1e9 for name in VICTIMS}
mode, far_east, last_print, start, home_at, returning = "", 0.0, 0.0, time.time(), None, False
while time.time() - start < 480:
    msg = m.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT"], blocking=True, timeout=2)
    if msg is None:
        continue
    kind = msg.get_type()
    if kind == "HEARTBEAT" and msg.get_srcSystem() == m.target_system and msg.get_srcComponent() == 1:
        new = mavutil.mode_string_v10(msg)
        if new != mode:
            mode = new
            modes.append(mode)
            print(f"  [{time.time() - start:5.1f} s] mode {mode}")
    elif kind == "STATUSTEXT" and any(word in msg.text for word in ("VICTIM", "RESUMING", "Reached", "Mission", "BLOCKED")):
        if "BLOCKED, HOLD" in msg.text:
            returning = True      # a blocked last waypoint ends the run where the boat is
        texts.append(msg.text)
        returning = returning or "RTL" in msg.text
        print(f"  [{time.time() - start:5.1f} s] {msg.text}")
    elif kind == "GLOBAL_POSITION_INT":
        n = (msg.lat / 1e7 - HOME[0]) * 111320
        e = (msg.lon / 1e7 - HOME[1]) * K
        track.append((time.time() - start, e, n, mode, returning))
        far_east = max(far_east, e)
        for name, c in CIRCLES.items():
            clearance[name] = min(clearance[name], to_circle(e, n, c))
        for name, r in RECTS.items():
            clearance[name] = min(clearance[name], to_rect(e, n, r))
        for name, v in VICTIMS.items():
            victim_range[name] = min(victim_range[name], math.hypot(e - v[0], n - v[1]))
        if time.time() - last_print >= 10:
            last_print = time.time()
            print(f"  [{last_print - start:5.1f} s] {mode:7s} east {e:5.1f} north {n:5.1f} speed {math.hypot(msg.vx, msg.vy) / 100:.1f}")
        if returning and math.hypot(e, n) < 3.0:
            home_at = time.time() - start
            break

with open(sys.argv[1], "w") as f:
    f.write("t,east,north,mode\n" + "".join(f"{t:.1f},{e:.2f},{n:.2f},{md}{'/home' if back else ''}\n" for t, e, n, md, back in track))

def swing(east_from, east_to, outbound):
    """Largest distance from the centre line while passing an obstacle, on the way out or back."""
    rows = [abs(n) for _t, e, n, _md, back in track if east_from <= e <= east_to and back != outbound]
    return max(rows) if rows else 0.0

print(f"modes: {' > '.join(modes)}")
print("clearance: " + ", ".join(f"{name} {d:.2f} m" for name, d in clearance.items()))
print("closest to the victims: " + ", ".join(f"{name} {d:.2f} m" for name, d in victim_range.items()))
print(f"swing off the centre line, out: tree {swing(8, 12, True):.2f} m, car {swing(16, 20, True):.2f} m, pole {swing(29, 33, True):.2f} m")
print(f"swing off the centre line, back: tree {swing(8, 12, False):.2f} m, car {swing(16, 20, False):.2f} m, pole {swing(29, 33, False):.2f} m")
print(f"furthest east {far_east:.1f} m; home " + (f"after {home_at:.0f} s" if home_at else "NOT reached"))

failures = []
for name, d in clearance.items():
    if d < CLEARANCE_M:
        failures.append(f"{name}: came within {d:.2f} m")
for name, d in victim_range.items():
    if d < CLEARANCE_M:
        failures.append(f"victim {name}: came within {d:.2f} m")
if swing(8, 12, True) < 1.0:
    failures.append("did not go around the tree")
if len([t for t in texts if "VICTIM SEEN" in t]) < 2:
    failures.append("fewer than two VICTIM SEEN reports")
if len([t for t in texts if "VICTIM REACHED" in t or "VICTIM UNREACHABLE" in t]) < 2:
    failures.append("fewer than two people attended (VICTIM REACHED / UNREACHABLE)")
if any("BLOCKED" in t for t in texts):
    failures.append("skipped a waypoint that was not blocked")
# Where the Pi says the people are: the last two numbers of a REACHED / UNREACHABLE report.
for name, v in VICTIMS.items():
    errors = []
    for t in texts:
        words = t.split()
        if ("REACHED" in t or "UNREACHABLE" in t) and len(words) >= 5:
            lat, lon = float(words[-3]), float(words[-2])
            errors.append(math.hypot((lon - HOME[1]) * K - v[0], (lat - HOME[0]) * 111320 - v[1]))
    best = min(errors, default=None)
    print(f"reported position of the person ({name}): " + ("none" if best is None else f"{best:.1f} m off"))
    if best is None or best > 3.0:
        failures.append(f"victim {name}: no report within 3 m of where they are")
# Reports that are not at either person: the boat went to something that is not there.
false_alarms = 0
for t in texts:
    words = t.split()
    if ("REACHED" in t or "UNREACHABLE" in t) and len(words) >= 5:
        e, n = (float(words[-2]) - HOME[1]) * K, (float(words[-3]) - HOME[0]) * 111320
        if all(math.hypot(e - v[0], n - v[1]) > 3.0 for v in VICTIMS.values()):
            false_alarms += 1
            print(f"false alarm: {t}  (east {e:.1f}, north {n:.1f})")
takeovers = modes.count("GUIDED")
print(f"takeovers {takeovers}, false alarms {false_alarms}, lost on the way {len([t for t in texts if 'VICTIM LOST' in t])}")
if false_alarms:
    failures.append(f"{false_alarms} false alarm(s)")
if far_east < 37.0:
    failures.append(f"only got {far_east:.1f} m down the street")
if home_at is None:
    failures.append("did not come home")
print("PASS" if not failures else "FAIL: " + "; ".join(failures))
sys.exit(1 if failures else 0)
EOF
RC=$?
echo "== Pi log"
grep -E "phase [a-z]+ ->|VICTIM|RESUMING|BLOCKED|ERROR|Traceback|Error|autopilot heard|detector" "$T/pi.log" | cut -c1-170 | tail -24
exit $RC
