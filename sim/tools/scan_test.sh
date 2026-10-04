#!/bin/bash
# How far to the side the camera's search sweep finds a person. On SITL instance 1, lake scene:
# the boat runs a straight leg that passes the victim OFFSET metres to starboard of them (the
# victim is to port), with the sweep or with the camera fixed ahead.
#
#   bash sim/tools/scan_test.sh [camera|simulated] [offset_m] [scan|noscan] [wait_s]
#     offset_m  how far the leg passes from the victim (default 3.5). The simulated detector
#               sees to 5 m (config/sim.yaml), so keep it below that.
#     scan      search sweep on (default). noscan: camera fixed ahead, for comparison
#     wait_s    instead of passing by, stop for this long at a waypoint abeam of the victim
#               (a waypoint with a Delay, as set in Mission Planner): the boat sits still and sweeps
#
# With the sweep the person must be found and reported within 3 m of where they are. With noscan
# the script only reports what happened.
set -o pipefail
DETECTOR="${1:-simulated}"
OFFSET="${2:-3.5}"
SCAN="${3:-scan}"
WAIT_S="${4:-0}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_scan
T="$SIM_DIR/scan_test"
rm -rf "$T"; mkdir -p "$T/models" "$T/run"
cp -r "$REPO/sim/models/vedc_buoy" "$T/models/"
sed -i 's#<fdm_port_in>9002</fdm_port_in>#<fdm_port_in>9012</fdm_port_in>#' "$T/models/vedc_buoy/model.sdf"
export GZ_SIM_RESOURCE_PATH="$T/models:$GZ_SIM_RESOURCE_PATH"

cd "$T"
setsid gz sim -s -r -v 1 vedc_lake.sdf > "$T/gz.log" 2>&1 < /dev/null &
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
PI_ARGS=(--overlay config/sim.yaml --detector "$DETECTOR" --mavlink tcp:127.0.0.1:5773)
[ "$SCAN" = noscan ] && PI_ARGS+=(--no-scan)
setsid python3 -u rescue_main.py "${PI_ARGS[@]}" > "$T/pi.log" 2>&1 < /dev/null &
PI=$!
cleanup() {
    kill -- -$PI -$SITL -$GZ 2>/dev/null
    sleep 2
    pkill -f "ardurover.*-I1"; pkill -f "mavproxy.*5770"
}
trap cleanup EXIT

timeout 260 python3 -u - "$OFFSET" "$SCAN" "$WAIT_S" "$DETECTOR" <<'EOF'
import math
import sys
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node
from pymavlink import mavutil

OFFSET, SCAN, WAIT_S = float(sys.argv[1]), sys.argv[2] == "scan", float(sys.argv[3])
CAMERA = sys.argv[4] == "camera"     # the simulated detector turns its field of view, not the Gazebo servo
M = mavutil.mavlink
HOME = (10.883438, 106.796019)
K = 111320 * math.cos(math.radians(HOME[0]))
VICTIM_NE = (8.0, 32.0)                       # north, east of home: sim/worlds/vedc_lake.sdf
track_n = VICTIM_NE[0] - OFFSET               # the leg runs east, the victim to port of it
# (north, east, delay). A waypoint is reached WP_RADIUS (2 m) before it: the waiting one is put
# 2 m past the victim's east so the boat stops abeam of them.
LEGS = [(track_n, 17.0, 0)] + ([(track_n, 34.0, WAIT_S)] if WAIT_S else []) + [(track_n, 50.0, 0)]

# The camera's pan as Gazebo has it: yaw of camera_link relative to the hull, positive right.
pans, yaw, held = [], {}, [None, 0.0]
def on_poses(msg):
    for pose in msg.pose:
        if pose.name in ("camera_link", "base_link"):
            q = pose.orientation
            yaw[pose.name] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    if len(yaw) == 2:
        pan = -math.degrees((yaw["camera_link"] - yaw["base_link"] + math.pi) % (2 * math.pi) - math.pi)
        step = round(pan / 10.0) * 10
        # Only where the camera rests: the same reading for 0.15 s, not the angles it passes through.
        if abs(pan - step) >= 2.0 or step != held[0]:
            held[0], held[1] = (step if abs(pan - step) < 2.0 else None), time.time()
        elif time.time() - held[1] >= 0.15 and (not pans or pans[-1] != step):
            pans.append(step)
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

def latlon(n, e):
    return HOME[0] + n / 111320, HOME[1] + e / K

items = [(M.MAV_CMD_NAV_WAYPOINT, 0, *HOME)] + [(M.MAV_CMD_NAV_WAYPOINT, delay, *latlon(n, e)) for n, e, delay in LEGS]
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
pans.clear()
m.set_mode("AUTO")
print(f"armed, AUTO; leg {OFFSET:g} m from the victim, sweep {'on' if SCAN else 'off'}" + (f", {WAIT_S:g} s wait abeam" if WAIT_S else ""))

texts, seen_at, closest, start, done = [], None, 1e9, time.time(), False
position = (0.0, 0.0)
while time.time() - start < 170 and not done:
    msg = m.recv_match(type=["GLOBAL_POSITION_INT", "STATUSTEXT"], blocking=True, timeout=2)
    if msg is None:
        continue
    if msg.get_type() == "STATUSTEXT":
        if any(word in msg.text for word in ("VICTIM", "RESUMING", "BLOCKED", "Reached", "Complete")):
            texts.append(msg.text)
            print(f"  [{time.time() - start:5.1f} s] {msg.text}")
            if "VICTIM SEEN" in msg.text and seen_at is None:
                seen_at = math.hypot(position[0] - VICTIM_NE[0], position[1] - VICTIM_NE[1])
            done = "RESUMING" in msg.text or "Complete" in msg.text
    else:
        position = ((msg.lat / 1e7 - HOME[0]) * 111320, (msg.lon / 1e7 - HOME[1]) * K)
        closest = min(closest, math.hypot(position[0] - VICTIM_NE[0], position[1] - VICTIM_NE[1]))

print("camera pan seen in Gazebo (deg, first steps): " + " ".join(f"{p:+d}" for p in pans[:14]))
errors = []
for t in texts:
    words = t.split()
    if ("REACHED" in t or "UNREACHABLE" in t) and len(words) >= 5:
        lat, lon = float(words[-3]), float(words[-2])
        errors.append(math.hypot((lat - HOME[0]) * 111320 - VICTIM_NE[0], (lon - HOME[1]) * K - VICTIM_NE[1]))
found = seen_at is not None
print(f"RESULT offset {OFFSET:g} m, sweep {'on' if SCAN else 'off'}: "
      + (f"seen from {seen_at:.1f} m" if found else "NOT seen")
      + (f", reported {min(errors):.1f} m off" if errors else "")
      + f", closest the boat came {closest:.1f} m")
if not SCAN:
    sys.exit(0)
failures = []
if CAMERA and pans[:4] != [0, -60, 0, 60]:
    failures.append("the camera did not step 0, -60, 0, +60")
if not found:
    failures.append("person not seen")
elif not errors or min(errors) > 3.0:
    failures.append("no report within 3 m of where the person is")
if closest < 0.8:
    failures.append(f"came within {closest:.2f} m of the person")
print("PASS" if not failures else "FAIL: " + "; ".join(failures))
sys.exit(1 if failures else 0)
EOF
RC=$?
grep -E "Traceback|Error|sweep" "$T/pi.log" | cut -c1-170 | tail -4
exit $RC
