#!/bin/bash
# Both ways from the Pixhawk boat to the dashboard, end to end, on SITL instance 1:
#
#   LoRa      Pi -> helper board (its firmware logic, helper_board_sim.py) -> the beacon line,
#             posted as the ESP32 shore station posts it          -> buoy PHAO-01
#   MAVLink   Pi -> autopilot -> ground station port -> esp32-lora-station/tools/mavlink_bridge.py
#                                                                 -> buoy SIM-01
#
# The boat runs the rescue scenario; afterwards the cloud must hold both buoys, with a position
# and with the count of people the Pi found.
#
# Needs the Worker running (cd esp32-lora-station/cloud && npx wrangler dev --port 8787) and its
# ingest token:
#   VEDC_INGEST_TOKEN=... bash sim/tools/dashboard_test.sh [worker url, default http://127.0.0.1:8787]
set -o pipefail
WORKER="${1:-http://127.0.0.1:8787}"
[ -n "$VEDC_INGEST_TOKEN" ] || { echo "set VEDC_INGEST_TOKEN to the Worker's INGEST_TOKEN" >&2; exit 2; }
export VEDC_INGEST_URL="$WORKER/api/ingest"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_dashboard
T="$SIM_DIR/dashboard_test"
rm -rf "$T"; mkdir -p "$T/models" "$T/run" "$T/hw"
cp -r "$REPO/sim/models/vedc_buoy" "$T/models/"
sed -i 's#<fdm_port_in>9002</fdm_port_in>#<fdm_port_in>9012</fdm_port_in>#' "$T/models/vedc_buoy/model.sdf"
export GZ_SIM_RESOURCE_PATH="$T/models:$GZ_SIM_RESOURCE_PATH"

cd "$T"
setsid gz sim -s -r -v 1 vedc_lake.sdf > "$T/gz.log" 2>&1 < /dev/null &
GZ=$!
sleep 10
cd "$T/run"
# 14561 for this test, 14562 for the bridge.
setsid sim_vehicle.py -v Rover -f rover-skid --model JSON -I1 \
    --add-param-file="$REPO/sim/params/vedc_buoy.parm" \
    --add-param-file="$REPO/sim/params/avoidance.parm" \
    --custom-location=10.883438,106.796019,10,90 \
    --no-wsl2-network --no-rebuild --wipe-eeprom \
    --out=udp:127.0.0.1:14561 --out=udp:127.0.0.1:14562 --mavproxy-args="--daemon" \
    > "$T/sitl.log" 2>&1 < /dev/null &
SITL=$!
source "$REPO/sim/tools/hw_standins.sh"
start_hw_standins "$T/hw" /vedc_buoy/lidar /model/vedc_buoy/joint/camera_pan_joint/cmd_pos || exit 1
sleep 28
python3 -u "$REPO/esp32-lora-station/tools/mavlink_bridge.py" --link udpin:0.0.0.0:14562 --url "$VEDC_INGEST_URL" --id SIM-01 \
    > "$T/bridge.log" 2>&1 &
BRIDGE=$!
cd "$REPO/Rasp_Pi"
setsid python3 -u rescue_main.py --overlay config/sim.yaml --overlay config/sim_hw.yaml --overlay "$T/hw/hw_ports.yaml" \
    --detector simulated --mavlink tcp:127.0.0.1:5773 > "$T/pi.log" 2>&1 < /dev/null &
PI=$!
cleanup() {
    kill $HW_PIDS $BRIDGE 2>/dev/null
    kill -- -$PI -$SITL -$GZ 2>/dev/null
    sleep 2
    pkill -f "ardurover.*-I1"; pkill -f "mavproxy.*5770"
}
trap cleanup EXIT

timeout 400 python3 -u - "$WORKER" <<'EOF'
import http.cookiejar
import json
import math
import sys
import time
import urllib.request

from pymavlink import mavutil

M = mavutil.mavlink
WORKER = sys.argv[1]
HOME = (10.883438, 106.796019)
K = 111320 * math.cos(math.radians(HOME[0]))
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

seen_on_port, texts, start = None, [], time.time()
while time.time() - start < 240 and not any("RESUMING" in t for t in texts):
    msg = m.recv_match(type=["STATUSTEXT", "NAMED_VALUE_INT"], blocking=True, timeout=2)
    if msg is None:
        continue
    if msg.get_type() == "STATUSTEXT" and ("VICTIM" in msg.text or "RESUMING" in msg.text):
        texts.append(msg.text)
        print(f"  [{time.time() - start:5.1f} s] report: {msg.text}")
    elif msg.get_type() == "NAMED_VALUE_INT":
        name = msg.name if isinstance(msg.name, str) else bytes(msg.name).decode("ascii", "replace")
        if name.strip(chr(0)) == "VICTIMS":
            seen_on_port = max(seen_on_port or 0, msg.value)
print(f"VICTIMS forwarded by the autopilot to the ground station port: {seen_on_port}")
time.sleep(7)        # one more beacon period and bridge post

# What a browser gets: a guest session, then the state.
jar = http.cookiejar.CookieJar()
opener = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(jar))
opener.open(urllib.request.Request(WORKER + "/api/auth/guest", data=b"", method="POST", headers={"Origin": WORKER}), timeout=5).close()
buoys = {b["id"]: b for b in json.load(opener.open(WORKER + "/api/state", timeout=5))}

failures = []
if not any("VICTIM REACHED" in t for t in texts):
    failures.append("the rescue did not happen")
if not seen_on_port:
    failures.append("the autopilot does not forward NAMED_VALUE_INT VICTIMS to the ground station port")
for name, link in (("PHAO-01", "LoRa from the helper board"), ("SIM-01", "MAVLink bridge")):
    b = buoys.get(name)
    if b is None:
        failures.append(f"{name} ({link}) is not in the cloud")
        continue
    summary = {k: b.get(k) for k in ("fix", "lat", "lon", "yaw", "mode", "victims", "rssi", "link", "age_s")}
    print(f"{name} ({link}): {summary}")
    if b.get("fix") != 1:
        failures.append(f"{name}: no position")
    elif math.hypot((b["lat"] - HOME[0]) * 111320, (b["lon"] - HOME[1]) * K) > 80.0:
        failures.append(f"{name}: position is not near the boat")
    if not b.get("victims"):
        failures.append(f"{name}: no victim count")
    if b.get("age_s", 99) > 15:
        failures.append(f"{name}: last update {b.get('age_s')} s ago")
    if "yaw" not in b:
        failures.append(f"{name}: no attitude")
print("PASS" if not failures else "FAIL: " + "; ".join(failures))
sys.exit(1 if failures else 0)
EOF
RC=$?
echo "== bridge"; tail -2 "$T/bridge.log"
echo "== board stand-in"; tail -3 "$T/hw/helper_sim.log"
exit $RC
