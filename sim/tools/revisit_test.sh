#!/bin/bash
# Does the boat go back to somebody it has already attended? On SITL instance 1. The mission runs
# past the person, turns round and runs past them again: the first pass must take the boat to
# them, the second must leave it on its route.
#
#   bash sim/tools/revisit_test.sh [camera|simulated]
#     camera     the real TFLite model on the Gazebo camera (default): its ranges, read off the
#                height of a box, are what makes a person seen again look like somebody new
#     simulated  geometric detector
set -o pipefail
# EXTRA_PARM=<file>: one more parameter file on top, e.g. sim/params/oneway_esc.parm
DETECTOR="${1:-camera}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_revisit
T="$SIM_DIR/revisit_test"
rm -rf "$T"; mkdir -p "$T/models" "$T/run"
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
    --add-param-file="$REPO/sim/params/avoidance.parm" ${EXTRA_PARM:+--add-param-file="$EXTRA_PARM"} \
    --custom-location=10.883438,106.796019,10,90 \
    --no-wsl2-network --no-rebuild --wipe-eeprom \
    --out=udp:127.0.0.1:14561 --mavproxy-args="--daemon" \
    > "$T/sitl.log" 2>&1 < /dev/null &
SITL=$!
sleep 28
cd "$REPO/Rasp_Pi"
# Instance 1 shifts SITL's ports by 10: its SERIAL2 (TELEM2) is TCP 5773.
setsid python3 -u rescue_main.py --overlay config/sim.yaml --detector "$DETECTOR" --mavlink tcp:127.0.0.1:5773 \
    > "$T/pi.log" 2>&1 < /dev/null &
PI=$!
cleanup() {
    kill -- -$PI -$SITL -$GZ 2>/dev/null
    sleep 2
    pkill -f "ardurover.*-I1"; pkill -f "mavproxy.*5770"
}
trap cleanup EXIT

timeout 460 python3 -u - <<'EOF'
import math
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink
HOME = (10.883438, 106.796019)
K = 111320 * math.cos(math.radians(HOME[0]))
VICTIM_NE = (8.0, 32.0)            # north, east of home: sim/worlds/vedc_lake.sdf
LEGS = [(8, 12), (8, 46), (8, 12)]     # out through the victim's position, and back through it

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

t0 = time.time()
while time.time() - t0 < 60:
    m.mav.command_long_send(m.target_system, m.target_component, M.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    a = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    if a and a.command == M.MAV_CMD_COMPONENT_ARM_DISARM and a.result == 0:
        break
    time.sleep(2)
else:
    sys.exit("FAIL: could not arm")
m.set_mode("AUTO")
print("armed, AUTO")

modes, texts = [], []
closest, hold_range, mode, seq, back = 1e9, [], "", 0, False
start = time.time()
while time.time() - start < 380:
    msg = m.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT", "MISSION_CURRENT"], blocking=True, timeout=2)
    if msg is None:
        continue
    kind = msg.get_type()
    if kind == "HEARTBEAT" and msg.get_srcSystem() == m.target_system and msg.get_srcComponent() == 1:
        new = mavutil.mode_string_v10(msg)
        if new != mode:
            mode = new
            modes.append(mode)
            print(f"  [{time.time() - start:5.1f} s] mode {mode}")
    elif kind == "STATUSTEXT" and ("VICTIM" in msg.text or "RESUMING" in msg.text):
        texts.append(msg.text)
        print(f"  [{time.time() - start:5.1f} s] report: {msg.text}")
    elif kind == "MISSION_CURRENT":
        seq = msg.seq
    elif kind == "GLOBAL_POSITION_INT":
        n = (msg.lat / 1e7 - HOME[0]) * 111320
        e = (msg.lon / 1e7 - HOME[1]) * K
        d = math.hypot(n - VICTIM_NE[0], e - VICTIM_NE[1])
        closest = min(closest, d)
        if mode == "LOITER":
            hold_range.append(d)
    if "RTL" in modes and len(modes) > 3:
        break
    back = back or seq >= 4        # item 4 is the waypoint back past the person

print(f"modes: {' > '.join(modes)}")
print(f"closest approach to the victim: {closest:.2f} m; range while holding: "
      + (f"{min(hold_range):.2f} to {max(hold_range):.2f} m" if hold_range else "never held"))
failures = []
takeovers = modes.count("GUIDED")
reached = len([t for t in texts if "VICTIM REACHED" in t or "VICTIM UNREACHABLE" in t])
print(f"takeovers {takeovers}, people reported reached {reached}, VICTIM SEEN {len([t for t in texts if 'VICTIM SEEN' in t])}, "
      f"lost on the way {len([t for t in texts if 'VICTIM LOST' in t])}")
if reached < 1:
    failures.append("the person was never attended")
if takeovers > 1:
    failures.append(f"took the boat over {takeovers} times for one person")
if reached > 1:
    failures.append(f"reported the same person reached {reached} times")
if not back:
    failures.append("the mission never got to its leg back past the person")
if closest < 0.8:
    failures.append(f"came within {closest:.2f} m of the victim")
print("PASS" if not failures else "FAIL: " + "; ".join(failures))
sys.exit(1 if failures else 0)
EOF
RC=$?
echo "== Pi log"
grep -E "phase [a-z]+ ->|VICTIM|RESUMING|ERROR|Traceback|Error|autopilot heard|detector" "$T/pi.log" | cut -c1-170 | tail -20
exit $RC
