#!/bin/bash
# The STM32F407 boat (Raspberry Pi + STM32F407, Rasp_Pi/main.py) in the central-Vietnam scene
# (sim/worlds/vedc_central.sdf), with no ArduPilot: the F407 is sim/tools/f407_board_sim.py, which
# runs the firmware's heading PID and drives the buoy's thrusters and pods itself. The Pi code is
# main.py as on the boat, with the boat's own settings (config/boat_stm32.yaml), its UART and LD14
# behind pseudo-terminals.
#
# The route (Rasp_Pi/config/mission_sim_central.yaml) runs straight down the channel past the
# person in the water at x 14, y -1; the person in the doorway at x 19 is left out of the test's
# copy of the scene, so that is the only person near the route. Passes when the Pi reports VICTIM_FOUND and the boat comes to
# rest within 1.2 to 4 m of the person.
#
#   bash sim/tools/stm32_central_test.sh
#   DURATION=<s>   how long the Pi runs (default 120)
#   MEDIA=<dir>    also record the run (sim/tools/record_media.py) and write stills to <dir>/img
#                  and a 30 s video to <dir>/video
#   GUI=1          open the Gazebo window too
set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_stm32_central
DURATION="${DURATION:-120}"
T="$SIM_DIR/stm32_central"
rm -rf "$T"; mkdir -p "$T/models" "$T/worlds"

# The buoy without its ArduPilot link: the F407 stand-in publishes the thrust and pod topics.
cp -r "$REPO/sim/models/vedc_buoy" "$T/models/"
sed -i '/<plugin name="ArduPilotPlugin"/,/<\/plugin>/d' "$T/models/vedc_buoy/model.sdf"
# The scene as the test uses it, in its own copy. Two fixed cameras for the record: one over the start of the channel looking down it, one close to
# the person in the water looking back at the boat coming.
python3 - "$REPO/sim/worlds/vedc_central.sdf" "$T/worlds/vedc_central.sdf" <<'EOF'
import sys
cameras = """
    <model name="cam_bank"><static>true</static><pose>-10 -3 5.5 0 0.2662 0.1244</pose><link name="link">
      <sensor name="cam_bank" type="camera"><always_on>1</always_on><update_rate>15</update_rate>
        <topic>vedc_central/cam_bank</topic>
        <camera><horizontal_fov>1.0</horizontal_fov><image><width>960</width><height>540</height></image>
          <clip><near>0.3</near><far>200</far></clip></camera></sensor></link></model>
    <model name="cam_close"><static>true</static><pose>17.5 -2.5 2.5 0 0.4036 2.7928</pose><link name="link">
      <sensor name="cam_close" type="camera"><always_on>1</always_on><update_rate>5</update_rate>
        <topic>vedc_central/cam_close</topic>
        <camera><horizontal_fov>1.2</horizontal_fov><image><width>960</width><height>540</height></image>
          <clip><near>0.3</near><far>200</far></clip></camera></sensor></link></model>
"""
text = open(sys.argv[1]).read()
marker = "    <!-- Part one"
assert marker in text
# Only the person in the water on the way: the one in the doorway of north house 2 is left out, so
# the boat has one person to find and go to.
doorway = next(line for line in text.splitlines(keepends=True) if "<name>victim_doorway</name>" in line)
text = text.replace(doorway, "")
open(sys.argv[2], "w").write(text.replace(marker, cameras + "\n" + marker, 1))
EOF
export GZ_SIM_RESOURCE_PATH="$T/models:$T/worlds:$GZ_SIM_RESOURCE_PATH"

PIDS=""
GZ=""
cleanup() {
    [ -n "$PIDS" ] && kill $PIDS 2>/dev/null
    [ -n "$GZ" ] && kill -- -"$GZ" 2>/dev/null     # gz sim's server runs in its process group
    sleep 1
    [ -n "$GZ" ] && kill -9 -- -"$GZ" 2>/dev/null
}
trap cleanup EXIT

cd "$T"
GZ_ARGS=(-s -r -v 1)
[ "$GUI" = 1 ] && GZ_ARGS=(-r -v 1)
setsid gz sim "${GZ_ARGS[@]}" "$T/worlds/vedc_central.sdf" > "$T/gz.log" 2>&1 < /dev/null &
GZ=$!
for _ in $(seq 120); do
    gz topic -l 2>/dev/null | grep -q "vedc_buoy/lidar" && break
    sleep 1
done
sleep 5

# The firmware's heading controller and camera glide, built for this machine.
FW="$REPO/STM32F4/App"
gcc -std=c11 -O1 -shared -fPIC -I"$FW" "$FW/heading_controller.c" "$FW/camera_pan.c" -lm -o "$T/libf407.so" || exit 1
python3 -u "$REPO/sim/tools/ld14_emulator.py" --pty "$T/lidar_pty" --topic /vedc_buoy/lidar > "$T/lidar_sim.log" 2>&1 &
PIDS="$PIDS $!"
python3 -u "$REPO/sim/tools/f407_board_sim.py" --pty "$T/f407_pty" --lib "$T/libf407.so" --log "$T/f407.csv" > "$T/f407.log" 2>&1 &
PIDS="$PIDS $!"
for _ in $(seq 20); do [ -e "$T/lidar_pty" ] && [ -e "$T/f407_pty" ] && break; sleep 0.5; done
printf 'serial:\n  stm32_port: %s\n  lidar_port: %s\n' "$T/f407_pty" "$T/lidar_pty" > "$T/ports.yaml"

if [ -n "$MEDIA" ]; then
    python3 -u "$REPO/sim/tools/record_media.py" record --out "$T/rec" --duration "$((DURATION + 5))" \
        --topic bank=/vedc_central/cam_bank:15 --topic close=/vedc_central/cam_close:5 \
        --topic buoy=/vedc_buoy/camera:4 --topic overview=/vedc_central/overview:1 --viewer 8090:15 \
        > "$T/record.log" 2>&1 &
    RECORDER=$!
fi

cd "$REPO/Rasp_Pi"
setsid python3 -u main.py --overlay config/sim.yaml --overlay config/boat_stm32.yaml --overlay config/sim_stm32.yaml --overlay "$T/ports.yaml" \
    --viewer 8090 > "$T/pi.log" 2>&1 < /dev/null &
PI=$!
PIDS="$PIDS $PI"
echo "running ${DURATION} s: Pi log $T/pi.log, F407 log $T/f407.log, viewer http://127.0.0.1:8090/"
sleep "$DURATION"
kill -INT $PI 2>/dev/null
sleep 2
[ -n "$RECORDER" ] && wait $RECORDER

python3 - "$T/f407.csv" "$T/pi.log" <<'EOF'
import csv, math, sys
rows = list(csv.DictReader(open(sys.argv[1])))
vx, vy = 14.0, -1.0
d = lambda r: math.hypot(float(r["x"]) - vx, float(r["y"]) - vy)
pi_log = open(sys.argv[2]).read()
found = int(float(rows[-1]["victims"])) if rows else 0
final = d(rows[-1]) if rows else float("nan")
closest = min(map(d, rows)) if rows else float("nan")
travel = sum(math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"])) for a, b in zip(rows, rows[1:]))
print(f"VICTIM_FOUND sent: {found}   following: {'following a person' in pi_log}")
print(f"distance to the person: closest {closest:.2f} m, at the end {final:.2f} m; travelled {travel:.1f} m")
ok = found > 0 and 1.2 <= final <= 4.0 and closest >= 0.8
print("PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
EOF
RESULT=$?

if [ -n "$MEDIA" ]; then
    mkdir -p "$MEDIA/img" "$MEDIA/video"
    python3 "$REPO/sim/tools/record_media.py" compose --rec "$T/rec" --log "$T/f407.csv" --victim 14,-1 \
        --img-out "$MEDIA/img" --video-out "$T/run_raw.mp4" --length 30
fi
exit $RESULT
