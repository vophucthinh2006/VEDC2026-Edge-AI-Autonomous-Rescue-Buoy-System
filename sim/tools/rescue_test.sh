#!/bin/bash
# The whole rescue scenario, on SITL instance 1 so it can run beside a run_boat.sh session:
# the boat runs a mission whose leg leads to the victim; Rasp_Pi/rescue_main.py must confirm the
# person, take over, stop at the stand-off, hold, report and give the boat back to the mission.
#
#   bash sim/tools/rescue_test.sh [simulated|camera]
#     simulated  geometric detector (default): tests the rescue logic and the MAVLink layer
#     camera     the real TFLite model on the Gazebo camera: tests the whole perception chain
set -o pipefail
# EXTRA_PARM=<file>: one more parameter file on top, e.g. sim/params/oneway_esc.parm
# HOLD_MODE=LOITER|FACE: overrides rescue.hold_mode, to compare the two ways of holding next to the person
DETECTOR="${1:-simulated}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_rescue
T="$SIM_DIR/rescue_test"
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
HOLD_OVERLAY=()
if [ -n "$HOLD_MODE" ]; then
    printf 'rescue:
  hold_mode: %s
' "$HOLD_MODE" > "$T/hold_mode.yaml"
    HOLD_OVERLAY=(--overlay "$T/hold_mode.yaml")
fi
# Instance 1 shifts SITL's ports by 10: its SERIAL2 (TELEM2) is TCP 5773.
# PI_VIEWER=8091 also serves what the Pi sees on that port (sim/tools/capture_view.py records it).
setsid python3 -u rescue_main.py --overlay config/sim.yaml "${HOLD_OVERLAY[@]}" --detector "$DETECTOR" --mavlink tcp:127.0.0.1:5773 \
    ${PI_VIEWER:+--viewer "$PI_VIEWER"} > "$T/pi.log" 2>&1 < /dev/null &
PI=$!
cleanup() {
    kill -- -$PI -$SITL -$GZ 2>/dev/null
    sleep 2
    pkill -f "ardurover.*-I1"; pkill -f "mavproxy.*5770"
}
trap cleanup EXIT

timeout 330 python3 -u - <<'EOF'
import math
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink
HOME = (10.883438, 106.796019)
K = 111320 * math.cos(math.radians(HOME[0]))
VICTIM_NE = (8.0, 32.0)            # north, east of home: sim/worlds/vedc_lake.sdf
LEGS = [(8, 12), (8, 46), (-12, 46)]   # the second leg runs through the victim's position

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
closest, hold_range, mode, seq = 1e9, [], "", 0
holding, yaw, bow_off = False, None, []      # between VICTIM REACHED and RESUMING; how far the bow is off the person then
start = time.time()
while time.time() - start < 240:
    msg = m.recv_match(type=["HEARTBEAT", "GLOBAL_POSITION_INT", "STATUSTEXT", "MISSION_CURRENT", "ATTITUDE"], blocking=True, timeout=2)
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
        holding = "REACHED" in msg.text or "UNREACHABLE" in msg.text or (holding and "RESUMING" not in msg.text)
        print(f"  [{time.time() - start:5.1f} s] report: {msg.text}")
    elif kind == "ATTITUDE":
        yaw = math.degrees(msg.yaw) % 360.0
    elif kind == "MISSION_CURRENT":
        seq = msg.seq
    elif kind == "GLOBAL_POSITION_INT":
        n = (msg.lat / 1e7 - HOME[0]) * 111320
        e = (msg.lon / 1e7 - HOME[1]) * K
        d = math.hypot(n - VICTIM_NE[0], e - VICTIM_NE[1])
        closest = min(closest, d)
        if holding:
            hold_range.append(d)
            if yaw is not None:
                to_victim = math.degrees(math.atan2(VICTIM_NE[1] - e, VICTIM_NE[0] - n)) % 360.0
                bow_off.append(abs((to_victim - yaw + 180.0) % 360.0 - 180.0))
    if "RTL" in modes and len(modes) > 3:
        break

print(f"modes: {' > '.join(modes)}")
print(f"closest approach to the victim: {closest:.2f} m; range while holding: "
      + (f"{min(hold_range):.2f} to {max(hold_range):.2f} m" if hold_range else "never held"))
if bow_off:
    ordered = sorted(bow_off)
    print(f"bow off the person while holding: mean {sum(bow_off) / len(bow_off):.0f} deg, median {ordered[len(ordered) // 2]:.0f} deg, "
          f"largest {ordered[-1]:.0f} deg, within 15 deg {100 * len([b for b in bow_off if b <= 15.0]) // len(bow_off)} % of the time")
failures = []
order = [x for x in modes if x in ("AUTO", "GUIDED", "LOITER")]
# The autopilot holds (LOITER), or the Pi does, bow on the person, staying in GUIDED (hold_mode FACE).
if order[:4] != ["AUTO", "GUIDED", "LOITER", "AUTO"] and order[:3] != ["AUTO", "GUIDED", "AUTO"]:
    failures.append("mode sequence is neither AUTO > GUIDED > LOITER > AUTO nor AUTO > GUIDED > AUTO")
if not any("RESUMING" in t for t in texts):
    failures.append("the boat was never given back to the mission")
if not any("VICTIM SEEN" in t for t in texts):
    failures.append("no VICTIM SEEN report")
if not any("VICTIM REACHED" in t for t in texts):
    failures.append("no VICTIM REACHED report")
if not hold_range or min(hold_range) < 0.8:
    failures.append("held closer than 0.8 m (or never held)")
if hold_range and max(hold_range) > 5.0:
    failures.append("drifted more than 5 m from the victim while holding")
if closest < 0.8:
    failures.append(f"came within {closest:.2f} m of the victim (ran over them)")
print("PASS" if not failures else "FAIL: " + "; ".join(failures))
sys.exit(1 if failures else 0)
EOF
RC=$?
echo "== Pi log"
grep -E "phase [a-z]+ ->|VICTIM|RESUMING|ERROR|Traceback|Error|autopilot heard|detector" "$T/pi.log" | cut -c1-170 | tail -20
exit $RC
