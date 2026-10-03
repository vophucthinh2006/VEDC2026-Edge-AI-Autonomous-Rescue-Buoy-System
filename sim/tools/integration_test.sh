#!/bin/bash
# Full Gazebo + ArduPilot check on SITL instance 1, alongside a running
# run_boat.sh session without touching it:
#   ports 9012 (JSON physics), 5770 (MAVProxy master), 14561 (this test),
#   private Gazebo partition, private copy of the model on port 9012.
# Checks: servo parameters, IMU noise, arming, GUIDED 25 m east, and the
# outputs of a right turn (left motor > right, rear pod > 1500, front pod < 1500).
#
#   bash /mnt/d/EMBEDDED/Competition/TKDT_2026/VEDC_2026/sim/tools/integration_test.sh
set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO/env.sh"
export GZ_PARTITION=vedc_integration
T="$SIM_DIR/integration_test"
rm -rf "$T"; mkdir -p "$T/models" "$T/run"
cp -r "$REPO/models/vedc_buoy" "$T/models/"
sed -i 's#<fdm_port_in>9002</fdm_port_in>#<fdm_port_in>9012</fdm_port_in>#' "$T/models/vedc_buoy/model.sdf"
export GZ_SIM_RESOURCE_PATH="$T/models:$GZ_SIM_RESOURCE_PATH"

cd "$T"
setsid gz sim -s -r -v 2 vedc_lake.sdf > "$T/gz.log" 2>&1 < /dev/null &
GZ=$!
sleep 8
cd "$T/run"
setsid sim_vehicle.py -v Rover -f rover-skid --model JSON -I1 \
    --add-param-file="$REPO/params/vedc_buoy.parm" \
    --custom-location=10.883438,106.796019,10,90 \
    --no-wsl2-network --no-rebuild --wipe-eeprom \
    --out=udp:127.0.0.1:14561 --mavproxy-args="--daemon" \
    > "$T/sitl.log" 2>&1 < /dev/null &
SITL=$!
cleanup() {
    kill -- -$SITL 2>/dev/null; kill -- -$GZ 2>/dev/null
    sleep 2
    pkill -f "ardurover.*-I1"; pkill -f "mavproxy.*5770"
}
trap cleanup EXIT
sleep 30

timeout 220 python3 - <<'EOF'
import statistics as st
import sys
import time
from pymavlink import mavutil

m = mavutil.mavlink_connection("udpin:0.0.0.0:14561", source_system=250)
while True:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=20)
    if hb is None:
        sys.exit("FAIL: no heartbeat")
    if hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        m.target_system, m.target_component = hb.get_srcSystem(), hb.get_srcComponent()
        break
failures = []

expected = {"FRAME_CLASS": 2, "SERVO1_FUNCTION": 73, "SERVO2_FUNCTION": 26, "SERVO3_FUNCTION": 74,
            "SERVO4_FUNCTION": 70, "SERVO5_FUNCTION": 26, "SERVO5_REVERSED": 1}
for name, want in expected.items():
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    got, t0 = None, time.time()
    while time.time() - t0 < 4:
        p = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if p and p.param_id.rstrip("\x00") == name:
            got = p.param_value
            break
    ok = got is not None and abs(got - want) < 0.01
    print(f"  {name:16s} {got}  {'ok' if ok else 'WRONG, want %s' % want}")
    if not ok:
        failures.append(name)

acc, t0 = [], time.time()
while time.time() - t0 < 5:
    msg = m.recv_match(type="RAW_IMU", blocking=True, timeout=1)
    if msg:
        acc.append((msg.xacc, msg.yacc, msg.zacc))
std = max(st.pstdev(c) for c in zip(*acc)) if len(acc) > 2 else 1e9
print(f"  IMU at rest: worst-axis std {std:.0f} mg over {len(acc)} samples")
if std > 200:
    failures.append("IMU noise")

armed = False
t0 = time.time()
while time.time() - t0 < 60 and not armed:
    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    armed = bool(ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and ack.result == 0)
    if not armed:
        time.sleep(2)
print("  armed" if armed else "  could not arm")
if not armed:
    failures.append("arm")
    print("FAIL:", ", ".join(failures))
    sys.exit(1)

def position():
    msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    return msg.lat / 1e7, msg.lon / 1e7

def go(lat, lon):
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000, int(lat * 1e7), int(lon * 1e7), 0, 0, 0, 0, 0, 0, 0, 0, 0)

def dist(a, b):
    import math
    return math.hypot((b[0] - a[0]) * 111320, (b[1] - a[1]) * 111320 * math.cos(math.radians(a[0])))

m.set_mode("GUIDED")
start = position()
goal = (start[0], start[1] + 25 / (111320 * 0.982))
go(*goal)
t0, reached = time.time(), False
while time.time() - t0 < 90 and not reached:
    reached = dist(position(), goal) < 3.0
moved = dist(start, position())
print(f"  GUIDED 25 m east: {'reached' if reached else 'NOT reached'}, moved {moved:.1f} m")
if not reached:
    failures.append("guided")

# A point 30 m south of an east-facing boat is a right turn.
here = position()
go(here[0] - 30 / 111320, here[1])
turn_ok, t0 = False, time.time()
while time.time() - t0 < 8 and not turn_ok:
    s = m.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=2)
    if s and s.servo1_raw > s.servo3_raw + 20:
        turn_ok = s.servo2_raw > 1500 and s.servo5_raw < 1500
        print(f"  right turn: L {s.servo1_raw} R {s.servo3_raw} rear pod {s.servo2_raw} "
              f"front {s.servo4_raw} front pod {s.servo5_raw}  {'ok' if turn_ok else 'WRONG pod direction'}")
if not turn_ok:
    failures.append("turn outputs")

m.set_mode("HOLD")
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
print("PASS" if not failures else "FAIL: " + ", ".join(failures))
sys.exit(1 if failures else 0)
EOF
