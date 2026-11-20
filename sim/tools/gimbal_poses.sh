#!/bin/bash
# Pictures of the camera gimbal (two SG90, pan + tilt) in the central-Vietnam scene: the buoy is set
# 8 m off the person on a roof (x 52, y -6.5), bow towards them, and the gimbal is put at pan
# -45 / 0 / +45 and tilt 0 / 20 (the two rows of the raster sweep). For each pose, a still from a
# fixed camera next to the bow and one from the buoy's own camera; then a contact sheet of each.
#
#   bash sim/tools/gimbal_poses.sh <out dir>
set -o pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="$(realpath -m "${1:?usage: gimbal_poses.sh <out dir>}")"
source "$REPO/sim/env.sh"
export GZ_PARTITION=vedc_gimbal_poses
T="$SIM_DIR/gimbal_poses"
rm -rf "$T"; mkdir -p "$T/models" "$T/worlds" "$OUT"

# No ArduPilot and nobody driving: the buoy floats where it is put.
cp -r "$REPO/sim/models/vedc_buoy" "$T/models/"
sed -i '/<plugin name="ArduPilotPlugin"/,/<\/plugin>/d' "$T/models/vedc_buoy/model.sdf"
python3 - "$REPO/sim/worlds/vedc_central.sdf" "$T/worlds/vedc_central.sdf" <<'EOF'
import math, sys
bx, by, yaw = 52.0, 1.5, -math.pi / 2          # 8 m north of the person on the roof, facing them
def world(lx, ly, lz):                          # buoy frame to world
    return bx + lx * math.cos(yaw) - ly * math.sin(yaw), by + lx * math.sin(yaw) + ly * math.cos(yaw), lz
cam, look = world(0.62, 0.24, 0.45), world(0.40, 0.0, 0.31)   # ahead of the bow, a little to port
dx, dy, dz = (l - c for l, c in zip(look, cam))
cyaw, cpitch = math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))
camera = f"""
    <model name="cam_gimbal"><static>true</static><pose>{cam[0]:.3f} {cam[1]:.3f} {cam[2]:.3f} 0 {cpitch:.4f} {cyaw:.4f}</pose><link name="link">
      <sensor name="cam_gimbal" type="camera"><always_on>1</always_on><update_rate>15</update_rate>
        <topic>vedc_central/cam_gimbal</topic>
        <camera><horizontal_fov>0.6</horizontal_fov><image><width>960</width><height>720</height></image>
          <clip><near>0.05</near><far>100</far></clip></camera></sensor></link></model>
"""
text = open(sys.argv[1]).read()
old = "<name>vedc_buoy</name>\n      <pose>0 0 0 0 0 0</pose>"
assert old in text
text = text.replace(old, f"<name>vedc_buoy</name>\n      <pose>{bx} {by} 0 0 0 {yaw:.4f}</pose>")
marker = "    <!-- Part one"
assert marker in text
open(sys.argv[2], "w").write(text.replace(marker, camera + "\n" + marker, 1))
EOF
export GZ_SIM_RESOURCE_PATH="$T/models:$T/worlds:$GZ_SIM_RESOURCE_PATH"

GZ=""
cleanup() {
    [ -n "$GZ" ] && kill -- -"$GZ" 2>/dev/null
    sleep 1
    [ -n "$GZ" ] && kill -9 -- -"$GZ" 2>/dev/null
}
trap cleanup EXIT
cd "$T"
setsid gz sim -s -r -v 1 "$T/worlds/vedc_central.sdf" > "$T/gz.log" 2>&1 < /dev/null &
GZ=$!
for _ in $(seq 120); do
    gz topic -l 2>/dev/null | grep -q "vedc_central/cam_gimbal" && break
    sleep 1
done
sleep 5

JOINT=/model/vedc_buoy/joint
for tilt in 0 20; do
    for pan in -45 0 45; do
        # cmd_pos in radians: pan positive right, tilt positive up (the CAM packet's convention)
        gz topic -t "$JOINT/camera_pan_joint/cmd_pos" -m gz.msgs.Double -p "data: $(python3 -c "import math; print(math.radians($pan))")"
        gz topic -t "$JOINT/camera_tilt_joint/cmd_pos" -m gz.msgs.Double -p "data: $(python3 -c "import math; print(math.radians($tilt))")"
        sleep 3
        # world_snapshot.py may crash in gz-transport on its way out, after the picture is written
        for shot in "/vedc_central/cam_gimbal gimbal" "/vedc_buoy/camera buoycam"; do
            set -- $shot
            rm -f "$OUT/$2_pan${pan}_tilt${tilt}.png"
            python3 "$REPO/sim/tools/world_snapshot.py" "$1" "$OUT/$2_pan${pan}_tilt${tilt}.png"
            [ -s "$OUT/$2_pan${pan}_tilt${tilt}.png" ] || exit 1
        done
    done
done

# Two contact sheets: rows tilt 0 / 20, columns pan -45 / 0 / +45.
python3 - "$OUT" <<'EOF'
import sys
from pathlib import Path
import cv2
import numpy as np
out = Path(sys.argv[1])
for kind, width in (("gimbal", 480), ("buoycam", 427)):
    rows = []
    for tilt in (0, 20):
        cells = []
        for pan in (-45, 0, 45):
            img = cv2.imread(str(out / f"{kind}_pan{pan}_tilt{tilt}.png"))
            img = cv2.resize(img, (width, int(img.shape[0] * width / img.shape[1])))
            label = f"pan {pan:+d}  tilt {tilt}"
            cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
            cells.append(img)
        rows.append(np.hstack(cells))
    cv2.imwrite(str(out / f"{kind}_sheet.jpg"), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 90])
    print(out / f"{kind}_sheet.jpg")
EOF
