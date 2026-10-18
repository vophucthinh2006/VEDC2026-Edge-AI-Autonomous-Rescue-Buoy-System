#!/bin/bash
# The rescue scenario with the Pi code reaching the simulator the way it reaches the boat's
# hardware, on SITL instance 1 so it can run beside a run_boat.sh session:
#
#   LiDAR          Gazebo -> LD14 serial stream (ld14_emulator.py) -> SerialLidar -> autopilot
#   helper board   Pi -> $CAM/$VIC/$POS on a serial port -> the STM32F103 firmware's logic
#                  (helper_board_sim.py) -> camera joint, alarm relays, LoRa beacon
#
# The real TFLite model looks at the Gazebo camera, which only turns if the board turns it.
# Checked: the rescue itself; the camera joint sweeping; the alarm and a beacon with the count
# and the boat's position; the E-stop button putting the autopilot in HOLD; the autopilot
# refusing to arm once the LiDAR goes silent; the Pi carrying on without the board.
#
#   bash sim/tools/hw_path_test.sh
set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_hw_path
T="$SIM_DIR/hw_path_test"
rm -rf "$T"; mkdir -p "$T/models" "$T/run" "$T/hw"
cp -r "$REPO/sim/models/vedc_buoy" "$T/models/"
sed -i 's#<fdm_port_in>9002</fdm_port_in>#<fdm_port_in>9012</fdm_port_in>#' "$T/models/vedc_buoy/model.sdf"
export GZ_SIM_RESOURCE_PATH="$T/models:$GZ_SIM_RESOURCE_PATH"

cd "$T"
setsid gz sim -s -r -v 1 vedc_lake.sdf > "$T/gz.log" 2>&1 < /dev/null &
GZ=$!
sleep 10
cd "$T/run"
setsid sim_vehicle.py -v Rover -f rover-skid --model JSON -I1 \
    --add-param-file="$REPO/sim/params/vedc_buoy.parm" \
    --add-param-file="$REPO/sim/params/avoidance.parm" \
    --custom-location=10.883438,106.796019,10,90 \
    --no-wsl2-network --no-rebuild --wipe-eeprom \
    --out=udp:127.0.0.1:14561 --mavproxy-args="--daemon" \
    > "$T/sitl.log" 2>&1 < /dev/null &
SITL=$!
source "$REPO/sim/tools/hw_standins.sh"
start_hw_standins "$T/hw" /vedc_buoy/lidar /model/vedc_buoy/joint/camera_pan_joint/cmd_pos || exit 1
sleep 28
cd "$REPO/Rasp_Pi"
# Instance 1 shifts SITL's ports by 10: its SERIAL2 (TELEM2) is TCP 5773.
setsid python3 -u rescue_main.py --overlay config/sim.yaml --overlay config/sim_hw.yaml --overlay "$T/hw/hw_ports.yaml" \
    --detector camera --mavlink tcp:127.0.0.1:5773 > "$T/pi.log" 2>&1 < /dev/null &
PI=$!
cleanup() {
    kill $HW_PIDS 2>/dev/null
    kill -- -$PI -$SITL -$GZ 2>/dev/null
    sleep 2
    pkill -f "ardurover.*-I1"; pkill -f "mavproxy.*5770"
}
trap cleanup EXIT

timeout 420 python3 -u - "$T" "$LIDAR_SIM_PID" "$HELPER_SIM_PID" "$PI" <<'EOF'
import math
import os
import signal
import sys
import time

from gz.msgs10.model_pb2 import Model
from gz.transport13 import Node
from pymavlink import mavutil

M = mavutil.mavlink
T, LIDAR_PID, HELPER_PID, PI_PID = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
HOME = (10.883438, 106.796019)
K = 111320 * math.cos(math.radians(HOME[0]))
VICTIM_NE = (8.0, 32.0)            # north, east of home: sim/worlds/vedc_lake.sdf
LEGS = [(8, 12), (8, 46), (-12, 46)]   # the second leg runs through the victim's position

# The camera joint as Gazebo has it: it only moves if the simulated board turned it.
pan = {"min": 0.0, "max": 0.0, "samples": 0}
def on_joints(msg):
    for joint in msg.joint:
        if joint.name == "camera_pan_joint":
            pan["min"], pan["max"] = min(pan["min"], joint.axis1.position), max(pan["max"], joint.axis1.position)
            pan["samples"] += 1
node = Node()
node.subscribe(Model, "/world/vedc_lake/model/vedc_buoy/joint_state", on_joints)

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

items = [(M.MAV_CMD_NAV_WAYPOINT, *HOME)] + [(M.MAV_CMD_NAV_WAYPOINT, *latlon(n, e)) for n, e in LEGS]
items.append((M.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0))
m.mav.mission_count_send(m.target_system, m.target_component, len(items), 0)
while True:
    r = m.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=10)
    if r is None:
        sys.exit("FAIL: mission upload timed out")
    if r.get_type() == "MISSION_ACK":
        break
    cmd, la, lo = items[r.seq]
    m.mav.mission_item_int_send(m.target_system, m.target_component, r.seq, M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                cmd, 0, 1, 0, 0, 0, 0, int(la * 1e7), int(lo * 1e7), 0, 0)
print("mission uploaded")

# Arming needs the proximity sensor healthy: it only is once the Pi feeds it from the serial LiDAR.
t0 = time.time()
while time.time() - t0 < 90:
    m.mav.command_long_send(m.target_system, m.target_component, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    a = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if a and a.command == M.MAV_CMD_COMPONENT_ARM_DISARM and a.result == 0:
        break
    time.sleep(2)
else:
    sys.exit("FAIL: could not arm (no LiDAR data through the serial path?)")
m.set_mode("AUTO")
print("armed, AUTO")

state = {"mode": "", "victims": None}
modes, texts, track = [], [], []
closest, hold_range = 1e9, []
start = time.time()

def pump(seconds, until=lambda: False):
    """Read telemetry for up to `seconds`, or until the condition holds."""
    global closest
    end = time.time() + seconds
    while time.time() < end and not until():
        msg = m.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT", "NAMED_VALUE_INT"], blocking=True, timeout=1)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "HEARTBEAT" and msg.get_srcSystem() == m.target_system and msg.get_srcComponent() == 1:
            new = mavutil.mode_string_v10(msg)
            if new != state["mode"]:
                state["mode"] = new
                modes.append(new)
                print(f"  [{time.time() - start:5.1f} s] mode {new}")
        elif kind == "STATUSTEXT":
            if any(word in msg.text for word in ("VICTIM", "RESUMING", "ESTOP", "PRX", "roximity")):
                texts.append(msg.text)
                print(f"  [{time.time() - start:5.1f} s] report: {msg.text}")
        elif kind == "NAMED_VALUE_INT":
            name = msg.name if isinstance(msg.name, str) else bytes(msg.name).decode("ascii", "replace")
            if name.strip(chr(0)) == "VICTIMS":
                state["victims"] = max(state["victims"] or 0, msg.value)
        elif kind == "GLOBAL_POSITION_INT":
            n = (msg.lat / 1e7 - HOME[0]) * 111320
            e = (msg.lon / 1e7 - HOME[1]) * K
            track.append((time.time(), msg.lat / 1e7, msg.lon / 1e7))
            d = math.hypot(n - VICTIM_NE[0], e - VICTIM_NE[1])
            closest = min(closest, d)
            if state["mode"] == "LOITER":
                hold_range.append(d)

def pi_log():
    with open(T + "/pi.log", errors="replace") as f:
        return f.read()

def events():
    with open(T + "/hw/helper_events.log") as f:
        return [(float(t), kind, value.strip()) for t, kind, value in (line.split(" ", 2) for line in f if line.count(" ") >= 2)]

failures = []

# 1. The rescue, with the LiDAR and the camera servo on their serial paths.
pump(240, lambda: any("RESUMING" in t for t in texts) and state["mode"] == "AUTO")
order = [x for x in modes if x in ("AUTO", "GUIDED", "LOITER")]
print(f"modes: {' > '.join(modes)}")
print(f"closest approach to the victim: {closest:.2f} m; range while holding: "
      + (f"{min(hold_range):.2f} to {max(hold_range):.2f} m" if hold_range else "never held"))
if order[:4] != ["AUTO", "GUIDED", "LOITER", "AUTO"]:
    failures.append("mode sequence is not AUTO > GUIDED > LOITER > AUTO")
if not any("VICTIM REACHED" in t for t in texts):
    failures.append("no VICTIM REACHED report")
if closest < 0.8:
    failures.append(f"came within {closest:.2f} m of the victim")

# 2. The camera joint swept: only the simulated board publishes to it in this mode.
print(f"camera joint in Gazebo: {math.degrees(pan['min']):+.0f} to {math.degrees(pan['max']):+.0f} deg over {pan['samples']} samples")
if pan["max"] - pan["min"] < math.radians(60.0):
    failures.append("the camera joint did not sweep: the servo command did not go through the board")

# 3. The board's alarm and its LoRa beacon.
log = events()
alarm = [t for t, kind, value in log if kind == "relays" and value == "1"]
beacons = [(t, value) for t, kind, value in log if kind == "beacon" and value.endswith(",v=1")]
print(f"board: alarm switched on {len(alarm)} time(s), {len([1 for _, k, _ in log if k == 'beacon'])} beacons, {len(beacons)} with v=1")
if not alarm:
    failures.append("the board never switched the alarm relays on")
if not beacons:
    failures.append("no LoRa beacon with v=1")
else:
    t, text = beacons[0]
    print(f"first beacon with the count: {text}")
    fields = text.split(",")
    try:
        lat, lon = float(fields[1]), float(fields[2])
        near = min((math.hypot((lat - la) * 111320, (lon - lo) * K) for when, la, lo in track if abs(when - t) <= 2.5), default=None)
        print("beacon position against the boat's track: " + ("no track point at that time" if near is None else f"{near:.1f} m off"))
        if near is None or near > 3.0:
            failures.append("the beacon's position is not where the boat was")
    except (ValueError, IndexError):
        failures.append("the beacon with v=1 carries no position")
print(f"VICTIMS as seen on the ground station port: {state['victims']}")
if not state["victims"]:
    failures.append("the autopilot did not forward the Pi's VICTIMS count to the ground station port")

# 4. E-stop button on the board: the Pi asks the autopilot for HOLD.
open(T + "/hw/estop", "w").close()
pressed = time.time()
pump(8, lambda: state["mode"] == "HOLD")
if state["mode"] == "HOLD":
    print(f"E-stop: HOLD {time.time() - pressed:.1f} s after the button")
else:
    failures.append("E-stop button did not put the autopilot in HOLD")
os.remove(T + "/hw/estop")

# 5. The LiDAR goes silent: the Pi must stop the feed, and the autopilot, left without proximity
# data, must refuse to arm (its pre-arm check "PRX1: No Data") rather than run on blind.
os.kill(LIDAR_PID, signal.SIGKILL)
pump(6, lambda: "obstacle feed stopped" in pi_log())
if "obstacle feed stopped" not in pi_log():
    failures.append("the Pi kept feeding obstacles without LiDAR scans")
m.mav.command_long_send(m.target_system, m.target_component, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0)   # disarm, forced
pump(3)
refused = False
for _ in range(4):
    m.mav.command_long_send(m.target_system, m.target_component, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    pump(3, lambda: any("PRX" in t for t in texts))
    if any("PRX" in t for t in texts):
        refused = True
        break
hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
print(f"LiDAR unplugged: Pi stopped the feed: {'obstacle feed stopped' in pi_log()}; autopilot refuses to arm for want of proximity data: {refused}")
if not refused:
    failures.append("the autopilot did not refuse to arm without LiDAR data")

# 6. The board is pulled out: the Pi carries on and says so.
os.kill(HELPER_PID, signal.SIGKILL)
pump(12, lambda: "board off" in pi_log())
alive = True
try:
    os.kill(PI_PID, 0)
except OSError:
    alive = False
print(f"board unplugged: Pi program {'still running' if alive else 'DIED'}, log says 'board off': {'board off' in pi_log()}")
if not alive:
    failures.append("the Pi program stopped when the board went away")
if "board off" not in pi_log():
    failures.append("the Pi did not notice the board was gone")

print("PASS" if not failures else "FAIL: " + "; ".join(failures))
sys.exit(1 if failures else 0)
EOF
RC=$?
echo "== Pi log"
grep -E "phase [a-z]+ ->|VICTIM|RESUMING|ERROR|WARNING|Traceback|Error|autopilot heard|rescue logic|UART|E-stop|LiDAR" "$T/pi.log" | cut -c1-190 | tail -24
echo "== LiDAR stand-in"; tail -3 "$T/hw/lidar_sim.log"
echo "== board stand-in"; tail -3 "$T/hw/helper_sim.log"
exit $RC
