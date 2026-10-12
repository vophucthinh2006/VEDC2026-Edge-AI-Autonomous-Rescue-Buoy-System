#!/bin/bash
# The central-Vietnam scene (sim/worlds/vedc_central.sdf), on SITL instance 1: a village by the
# lagoon after Typhoon Kalmaegi, then a hamlet flooded to the eaves. The boat runs the 66 m of the
# channel and comes back. Counted: how many of the six people or groups it stops for, whether it
# stops twice for the same ones, how close it comes to the wrecks, the bamboo, the poles and the
# houses, and how far the bow is off the people while it holds next to them.
#
#   bash sim/tools/central_test.sh [camera|simulated]
#   HOLD_MODE=LOITER|FACE overrides rescue.hold_mode; the run reports how far the bow was off the
#   people while the boat held next to them.
#     camera     the real TFLite model on the Gazebo camera (default)
#     simulated  geometric detector
set -o pipefail
# SCAN_LIMIT=<deg>: the camera sweeps between -deg and +deg instead of the configured 60
# EXTRA_PARM=<file>: one more parameter file on top, e.g. sim/params/oneway_esc.parm
DETECTOR="${1:-camera}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_central_test
T="$SIM_DIR/central_test"
rm -rf "$T"; mkdir -p "$T/models" "$T/run"
cp -r "$REPO/sim/models/vedc_buoy" "$T/models/"
sed -i 's#<fdm_port_in>9002</fdm_port_in>#<fdm_port_in>9012</fdm_port_in>#' "$T/models/vedc_buoy/model.sdf"
export GZ_SIM_RESOURCE_PATH="$T/models:$GZ_SIM_RESOURCE_PATH"

cd "$T"
setsid gz sim -s -r -v 1 vedc_central.sdf > "$T/gz.log" 2>&1 < /dev/null &
GZ=$!
sleep 15
cd "$T/run"
setsid sim_vehicle.py -v Rover -f rover-skid --model JSON -I1 \
    --add-param-file="$REPO/sim/params/vedc_buoy.parm" \
    --add-param-file="$REPO/sim/params/avoidance.parm" ${EXTRA_PARM:+--add-param-file="$EXTRA_PARM"} \
    --custom-location=10.883438,106.796019,10,90 \
    --no-wsl2-network --no-rebuild --wipe-eeprom \
    --out=udp:127.0.0.1:14561 --mavproxy-args="--daemon" \
    > "$T/sitl.log" 2>&1 < /dev/null &
SITL=$!
sleep 28
cd "$REPO/Rasp_Pi"
# Instance 1 shifts SITL's ports by 10: its SERIAL2 (TELEM2) is TCP 5773.
NO_SCAN=()
if [ -n "$HOLD_MODE" ]; then
    printf 'rescue:
  hold_mode: %s
' "$HOLD_MODE" > "$T/hold_mode.yaml"
    NO_SCAN=(--overlay "$T/hold_mode.yaml")
fi
setsid python3 -u rescue_main.py --overlay config/sim.yaml --overlay config/sim_central.yaml \
    --detector "$DETECTOR" --mavlink tcp:127.0.0.1:5773 "${NO_SCAN[@]}" ${SCAN_LIMIT:+--scan-limit "$SCAN_LIMIT"} > "$T/pi.log" 2>&1 < /dev/null &
PI=$!
cleanup() {
    kill -- -$PI -$SITL -$GZ 2>/dev/null
    sleep 2
    pkill -f "ardurover.*-I1"; pkill -f "mavproxy.*5770"
}
trap cleanup EXIT

timeout 1000 python3 -u - "$T/track.csv" <<'EOF'
import math
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink
HOME = (10.883438, 106.796019)
K = 111320 * math.cos(math.radians(HOME[0]))
# sim/worlds/vedc_central.sdf, as (east, north). People standing together are one group.
GROUPS = {"in the water, part one": [(14.0, -1.0)], "in a doorway": [(19.0, 6.55)], "on a roof": [(52.0, -6.5)],
          "in the water, part two": [(59.5, -1.2)], "three on a roof": [(63.0, 6.5), (64.2, 6.5), (65.4, 6.5)],
          "on the end house": [(72.0, 0.5)]}
# What the boat must keep clear of. Circles: (east, north, radius). Boxes: (east, north of the
# centre, half length, half width, yaw): each house by its own footprint, eaves included, so that
# the alleys between them are open water, as they are in the scene.
CIRCLES = {"basket boat 1": (11.0, 1.6, 0.95), "basket boat 2": (29.5, 2.4, 0.95), "pole 1": (34.0, -1.6, 0.25),
           "bamboo 1": (47.0, -1.6, 1.0), "pole 2": (56.0, 1.4, 0.25), "bamboo 2": (66.0, -2.0, 1.0)}
BOXES = {"wrecked boat": (22.3, -4.3, 4.9, 1.3, 0.22), "end house": (74.5, 0.0, 2.8, 3.8, 0.0)}
for i, x in enumerate((8.0, 19.0, 30.0)):
    BOXES[f"storm house north {i + 1}"] = (x, 9.0, 3.0, 2.25, 0.0)
for i, x in enumerate((6.0, 16.5, 27.0)):
    BOXES[f"storm house south {i + 1}"] = (x, -9.0, 3.0, 2.25, 0.0)
for i, (x, tiled) in enumerate(((44.0, True), (54.0, False), (64.0, True))):
    BOXES[f"house north {i + 1}"] = (x, 9.0, 3.8 if tiled else 3.3, 2.8 if tiled else 2.55, 0.0)
for i, (x, tiled) in enumerate(((42.0, False), (52.0, True), (62.0, False))):
    BOXES[f"house south {i + 1}"] = (x, -9.0, 3.8 if tiled else 3.3, 2.8 if tiled else 2.55, 0.0)
CLEARANCE_M = 0.8                  # from the boat's GPS position; the hulls reach 0.55 m from it
LEGS = [(0.0, 66.5)]               # north, east: down the channel, then RTL. 5.5 m short of the end house.
LOOK_S = 10
SAME_M = 3.0                       # two stops closer together than this are for the same people

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

modes, texts, mode, start, home_at, returning, far_east = [], [], "", time.time(), None, False, 0.0
held_at, yaw, bow_off, held_range = None, None, [], []     # between a REACHED report and RESUMING: the place reported
clearance = {name: (1e9, 0.0, 0.0) for name in list(CIRCLES) + list(BOXES)}      # nearest, and where the boat was then
track = []

def to_box(e, n, box):
    ce, cn, half_l, half_w, yaw = box
    along = (e - ce) * math.cos(yaw) + (n - cn) * math.sin(yaw)
    across = -(e - ce) * math.sin(yaw) + (n - cn) * math.cos(yaw)
    return math.hypot(max(abs(along) - half_l, 0.0), max(abs(across) - half_w, 0.0))

while time.time() - start < 900:
    msg = m.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT", "ATTITUDE"], blocking=True, timeout=2)
    if msg is None:
        continue
    kind = msg.get_type()
    if kind == "HEARTBEAT" and msg.get_srcSystem() == m.target_system and msg.get_srcComponent() == 1:
        new = mavutil.mode_string_v10(msg)
        if new != mode:
            mode = new
            modes.append(mode)
    elif kind == "ATTITUDE":
        yaw = math.degrees(msg.yaw) % 360.0
    elif kind == "STATUSTEXT" and any(word in msg.text for word in ("VICTIM", "RESUMING", "PEOPLE", "BLOCKED", "Mission", "Reached", "SAME")):
        texts.append(msg.text)
        words = msg.text.split()
        if ("REACHED" in msg.text or "UNREACHABLE" in msg.text) and len(words) >= 5:
            held_at = ((float(words[-2]) - HOME[1]) * K, (float(words[-3]) - HOME[0]) * 111320)
        elif "RESUMING" in msg.text:
            held_at = None
        returning = returning or "RTL" in msg.text or "BLOCKED, HOLD" in msg.text
        print(f"  [{time.time() - start:5.1f} s] {msg.text}")
    elif kind == "GLOBAL_POSITION_INT":
        n, e = (msg.lat / 1e7 - HOME[0]) * 111320, (msg.lon / 1e7 - HOME[1]) * K
        far_east = max(far_east, e)
        track.append((time.time() - start, e, n, mode))
        for name, (ce, cn, r) in CIRCLES.items():
            clearance[name] = min(clearance[name], (math.hypot(e - ce, n - cn) - r, e, n))
        for name, box in BOXES.items():
            clearance[name] = min(clearance[name], (to_box(e, n, box), e, n))
        if held_at is not None and yaw is not None:
            to_them = math.degrees(math.atan2(held_at[0] - e, held_at[1] - n)) % 360.0
            bow_off.append(abs((to_them - yaw + 180.0) % 360.0 - 180.0))
            held_range.append(math.hypot(held_at[0] - e, held_at[1] - n))
        if returning and math.hypot(e, n) < 3.0:
            home_at = time.time() - start
            break

# Where the boat stopped for people: the position in each REACHED / UNREACHABLE report.
stops = []
for t in texts:
    words = t.split()
    if ("REACHED" in t or "UNREACHABLE" in t) and len(words) >= 5:
        stops.append(((float(words[-2]) - HOME[1]) * K, (float(words[-3]) - HOME[0]) * 111320))
def group_of(stop):
    name, d = min(((g, min(math.hypot(stop[0] - e, stop[1] - n) for e, n in people)) for g, people in GROUPS.items()), key=lambda x: x[1])
    return name if d <= 4.0 else f"nobody ({d:.1f} m from the {name})"
named = [group_of(s) for s in stops]
print(f"takeovers {modes.count('GUIDED')}, stops for people {len(stops)}, lost on the way {len([t for t in texts if 'VICTIM LOST' in t])}")
for s, g in zip(stops, named):
    print(f"  stop at east {s[0]:5.1f}, north {s[1]:5.1f}: {g}")
repeats = [g for g in set(named) if named.count(g) > 1]
close = sum(1 for i in range(len(stops)) for j in range(i) if math.hypot(stops[i][0] - stops[j][0], stops[i][1] - stops[j][1]) < SAME_M)
counts = [t for t in texts if t.startswith("PEOPLE")]
print(f"groups attended {len(set(g for g in named if not g.startswith('nobody')))} of {len(GROUPS)}; stopped again for: {', '.join(repeats) or 'nobody'}; "
      f"last count: {counts[-1] if counts else 'none'}")
if bow_off:
    ordered = sorted(bow_off)
    print(f"bow off the people while holding: mean {sum(bow_off) / len(bow_off):.0f} deg, median {ordered[len(ordered) // 2]:.0f} deg, "
          f"largest {ordered[-1]:.0f} deg, within 15 deg {100 * len([b for b in bow_off if b <= 15.0]) // len(bow_off)} % of the time; "
          f"range to the place reported {min(held_range):.1f} to {max(held_range):.1f} m")
print(f"furthest east {far_east:.1f} m; home " + (f"after {home_at:.0f} s" if home_at else "NOT reached"))
with open(sys.argv[1], "w") as f:
    f.write("t,east,north,mode\n" + "".join(f"{t:.1f},{e:.2f},{n:.2f},{md}\n" for t, e, n, md in track))
nearest = sorted(clearance.items(), key=lambda item: item[1][0])
print("closest the boat came to: " + ", ".join(f"{name} {d:.2f} m" for name, (d, _e, _n) in nearest[:6]))
failures = []
for name, (d, e, n) in nearest:
    if d < CLEARANCE_M:
        failures.append(f"{name}: came within {d:.2f} m (boat at east {e:.1f}, north {n:.1f})")
if far_east < 60.0:
    failures.append(f"only got {far_east:.1f} m down the channel")
if home_at is None:
    failures.append("did not come home")
if repeats or close:
    failures.append(f"stopped more than once for the same people ({', '.join(repeats) or 'two stops within 3 m'})")
if not stops:
    failures.append("stopped for nobody")
if any(g.startswith("nobody") for g in named):
    failures.append("stopped where nobody is")
print("PASS" if not failures else "FAIL: " + "; ".join(failures))
sys.exit(1 if failures else 0)
EOF
RC=$?
echo "== Pi log"
grep -E "phase [a-z]+ ->|VICTIM|RESUMING|PEOPLE|BLOCKED|Traceback|Error" "$T/pi.log" | cut -c1-170 | tail -30
exit $RC
