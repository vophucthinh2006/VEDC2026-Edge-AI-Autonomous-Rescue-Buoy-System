#!/bin/bash
# What the boat does with two people in the same camera frame. Lake scene with a second person
# added, on SITL instance 1; the boat runs a straight leg towards them with the Pi's rescue code.
#
#   bash sim/tools/two_person_test.sh [side|behind] [camera|simulated]
#     side    the two are 3 m apart, side by side across the boat's course: equally far, so
#             equally tall in the frame, 3 m either side of... 1.5 m either side of the leg
#     behind  one on the leg, the other 4.5 m further and 1.5 m to the side: both in the frame,
#             the far one smaller
#
# It reports how many people were attended, how far each report is from either person, and how
# much the camera servo swung back and forth while the Pi had the boat. Pass: both people attended,
# every report within 3 m of one of them, neither run closer than 0.8 m.
set -o pipefail
SCENE="${1:-side}"
DETECTOR="${2:-camera}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_two
T="$SIM_DIR/two_person_test"
rm -rf "$T"; mkdir -p "$T/models" "$T/run"
cp -r "$REPO/sim/models/vedc_buoy" "$T/models/"
sed -i 's#<fdm_port_in>9002</fdm_port_in>#<fdm_port_in>9012</fdm_port_in>#' "$T/models/vedc_buoy/model.sdf"
export GZ_SIM_RESOURCE_PATH="$T/models:$GZ_SIM_RESOURCE_PATH"
case "$SCENE" in            # east north of each person (Gazebo x y), the leg runs east along north 8
    side)   P1="32 6.5"; P2="32 9.5" ;;
    behind) P1="30 8";   P2="34.5 9.5" ;;
    *) echo "unknown scene $SCENE" >&2; exit 2 ;;
esac
# The lake world with its one person moved and a second one added.
python3 - "$REPO/sim/worlds/vedc_lake.sdf" "$T/vedc_two.sdf" "$P1" "$P2" <<'EOF'
import sys
src, out, p1, p2 = sys.argv[1:5]
s = open(src, encoding="utf-8").read()
old = "<pose>32 8 0 0 0 -1.5708</pose>"
assert s.count(old) == 1, "the person's pose in vedc_lake.sdf has changed"
s = s.replace(old, f"<pose>{p1} 0 0 0 -1.5708</pose>")
second = f"""    <include>
      <name>victim_2</name>
      <pose>{p2} 0 0 0 -1.5708</pose>
      <uri>model://victim_person</uri>
    </include>
  </world>"""
s = s.replace("  </world>", second)
open(out, "w", encoding="utf-8", newline="\n").write(s)
EOF
k=$(python3 -c "import math; print(111320 * math.cos(math.radians(10.883438)))")
victims=$(python3 -c "
for p in ('$P1', '$P2'):
    e, n = map(float, p.split())
    print(f'    - [{10.883438 + n / 111320:.7f}, {106.796019 + e / $k:.7f}]')")
printf 'sim:\n  victims:\n%s\n' "$victims" > "$T/two.yaml"

cd "$T"
setsid gz sim -s -r -v 1 "$T/vedc_two.sdf" > "$T/gz.log" 2>&1 < /dev/null &
GZ=$!
sleep 12
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
setsid python3 -u rescue_main.py --overlay config/sim.yaml --overlay "$T/two.yaml" --detector "$DETECTOR" \
    --mavlink tcp:127.0.0.1:5773 --viewer 8092 > "$T/pi.log" 2>&1 < /dev/null &
PI=$!
cleanup() {
    kill -- -$PI -$SITL -$GZ 2>/dev/null
    sleep 2
    pkill -f "ardurover.*-I1"; pkill -f "mavproxy.*5770"
}
trap cleanup EXIT

timeout 330 python3 -u - "$SCENE" "$P1" "$P2" "$T" <<'EOF'
import math
import sys
import time
import urllib.request

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from pymavlink import mavutil

SCENE, T = sys.argv[1], sys.argv[4]
PEOPLE = {"1": tuple(float(x) for x in sys.argv[2].split()), "2": tuple(float(x) for x in sys.argv[3].split())}   # east, north
M = mavutil.mavlink
HOME = (10.883438, 106.796019)
K = 111320 * math.cos(math.radians(HOME[0]))
LEGS = [(8.0, 17.0), (8.0, 50.0)]      # north, east

# Camera pan from Gazebo (as in scan_test.sh), with the time, to count how often it swings back.
pan_log, yaw = [], {}
def on_poses(msg):
    for pose in msg.pose:
        if pose.name in ("camera_link", "base_link"):
            q = pose.orientation
            yaw[pose.name] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    if len(yaw) == 2:
        pan_log.append((time.time(), -math.degrees((yaw["camera_link"] - yaw["base_link"] + math.pi) % (2 * math.pi) - math.pi)))
node = Node()
node.subscribe(Pose_V, "/world/vedc_lake/dynamic_pose/info", on_poses)

m = mavutil.mavlink_connection("udpin:0.0.0.0:14561", source_system=250)
while True:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=30)
    if hb is None:
        sys.exit("FAIL: no heartbeat")
    if hb.autopilot == M.MAV_AUTOPILOT_ARDUPILOTMEGA and hb.get_srcComponent() == 1:
        m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
        break
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
m.set_mode("AUTO")
print(f"armed, AUTO; scene {SCENE}: person 1 at east {PEOPLE['1'][0]:g} north {PEOPLE['1'][1]:g}, person 2 at east {PEOPLE['2'][0]:g} north {PEOPLE['2'][1]:g}")

def nearest_person(lat, lon):
    e, n = (lon - HOME[1]) * K, (lat - HOME[0]) * 111320
    name = min(PEOPLE, key=lambda p: math.hypot(e - PEOPLE[p][0], n - PEOPLE[p][1]))
    return name, math.hypot(e - PEOPLE[name][0], n - PEOPLE[name][1]), e, n

start, mode, reports, spans, closest, shots = time.time(), "", [], [], {p: 1e9 for p in PEOPLE}, 0
guided_since = None
while time.time() - start < 230:
    msg = m.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT"], blocking=True, timeout=2)
    if msg is None:
        continue
    kind = msg.get_type()
    if kind == "HEARTBEAT" and msg.get_srcSystem() == m.target_system and msg.get_srcComponent() == 1:
        new = mavutil.mode_string_v10(msg)
        if new != mode:
            if new == "GUIDED":
                guided_since = time.time()
            elif mode == "GUIDED" and guided_since:
                spans.append((guided_since, time.time()))
            mode = new
            print(f"  [{time.time() - start:5.1f} s] mode {mode}")
    elif kind == "STATUSTEXT":
        if any(word in msg.text for word in ("VICTIM", "RESUMING", "BLOCKED", "PEOPLE")):
            print(f"  [{time.time() - start:5.1f} s] {msg.text}")
            words = msg.text.split()
            if ("SEEN" in msg.text or "REACHED" in msg.text or "UNREACHABLE" in msg.text) and len(words) >= 5:
                reports.append((words[1], *nearest_person(float(words[-3]), float(words[-2]))))
            if "SEEN" in msg.text and shots < 3:          # what the Pi saw at that moment
                shots += 1
                try:
                    urllib.request.urlretrieve("http://127.0.0.1:8092/frame.jpg", f"{T}/seen_{shots}.jpg")
                except OSError:
                    pass
        if "Complete" in msg.text:
            break
    elif kind == "GLOBAL_POSITION_INT":
        e, n = (msg.lon / 1e7 - HOME[1]) * K, (msg.lat / 1e7 - HOME[0]) * 111320
        for p in PEOPLE:
            closest[p] = min(closest[p], math.hypot(e - PEOPLE[p][0], n - PEOPLE[p][1]))

# Servo swings while the Pi had the boat (GUIDED): reversals of more than 8 deg.
def swings(t_from, t_to):
    series = [a for t, a in pan_log if t_from <= t <= t_to]
    count, anchor, direction = 0, None, 0
    for a in series:
        if anchor is None:
            anchor = a
        elif direction >= 0 and a < anchor - 8.0:
            count, direction, anchor = count + (direction > 0), -1, a
        elif direction <= 0 and a > anchor + 8.0:
            count, direction, anchor = count + (direction < 0), 1, a
        elif (direction > 0 and a > anchor) or (direction < 0 and a < anchor):
            anchor = a
    return count, (min(series), max(series)) if series else (0.0, 0.0)

attended = [r for r in reports if r[0] in ("REACHED", "UNREACHABLE")]
people_attended = sorted({r[1] for r in attended if r[2] <= 3.0})
for kind_, name, off, e, n in reports:
    print(f"report {kind_:11s} at east {e:5.1f} north {n:5.1f}: nearest is person {name}, {off:.1f} m off")
for index, (a, b) in enumerate(spans, 1):
    count, (low, high) = swings(a, b)
    print(f"approach {index}: {b - a:4.1f} s in GUIDED, camera between {low:+.0f} and {high:+.0f} deg, {count} reversals of more than 8 deg")
print(f"RESULT {SCENE}: takeovers {len(spans)}, people attended {len(people_attended)} of 2 ({', '.join(people_attended) or 'none'}), "
      f"reports off by {', '.join(f'{r[2]:.1f}' for r in attended) or '-'} m, "
      f"closest the boat came: person 1 {closest['1']:.1f} m, person 2 {closest['2']:.1f} m")
failures = []
if len(people_attended) < 2:
    failures.append(f"only {len(people_attended)} of 2 people attended")
if any(r[2] > 3.0 for r in attended):
    failures.append("a report more than 3 m from either person")
if min(closest.values()) < 0.8:
    failures.append(f"came within {min(closest.values()):.2f} m of a person")
print("PASS" if not failures else "FAIL: " + "; ".join(failures))
sys.exit(1 if failures else 0)
EOF
RC=$?
grep -E "ranged as|Traceback|Error" "$T/pi.log" | cut -c1-12,40-130 | tail -8
exit $RC
