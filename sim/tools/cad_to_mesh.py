#!/usr/bin/env python3
"""Turn the team's CAD export into the visual meshes of the simulated buoy.

    python3 sim/tools/cad_to_mesh.py sim/Thuyen.ETL.STL sim/models/vedc_buoy/meshes

The CAD file is in millimetres with the length along its Z axis (bow at low Z), the beam along X
and up along Y. Gazebo wants metres, x forward, y to port, z up. The model is centred on the
length and the beam, and placed so that the bottom of the hulls sits HULL_BOTTOM_M below the
model origin, where the collision boxes of model.sdf (which carry the buoyancy) have theirs.

The CAD is one fused shape. It is cut into the parts that move, each written in the frame of the
link that carries it in model.sdf, so the pods steer and the camera pans on screen:

    hull.stl            everything else, in the model frame
    pod_rear_left.stl   strut, motor and propeller under the port hull
    pod_rear_right.stl  the same under the starboard hull
    pod_front.stl       the motor in the tunnel between the hulls
    camera.stl          camera body and stem on the bow box (its base plates stay on the hull)

One change to the CAD, agreed with the team: the front motor is lowered by FRONT_DROP_M. As
drawn, the centre of its 0.12 m propeller is 0.02 m below the waterline, half of it in the air.

Only the look comes from these meshes. Mass, buoyancy and thrust stay in model.sdf, and the link
positions below must match the <pose> of the same links there.
"""
import pathlib
import struct
import sys

import numpy as np

HULL_BOTTOM_MM = 115.0     # CAD height of the flat bottom of the hulls (the thrusters reach down to 0)
HULL_BOTTOM_M = -0.05      # the same surface in the model frame: bottom of the collision boxes
FRONT_DROP_M = 0.09        # front propeller centre from 0.02 m to 0.11 m below the origin

# Link origins in the model frame (m), as in model.sdf.
POD_REAR_LEFT = (-0.40, 0.234, -0.115)
POD_REAR_RIGHT = (-0.40, -0.234, -0.115)
POD_FRONT = (0.19, 0.0, -0.02 - FRONT_DROP_M)
CAMERA = (0.41, 0.0, 0.295)

RECORD = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])


def load(path: str) -> np.ndarray:
    data = open(path, "rb").read()
    count = struct.unpack("<I", data[80:84])[0]
    if len(data) != 84 + 50 * count:
        raise SystemExit("not a binary STL")
    cad = np.frombuffer(data, dtype=RECORD, count=count, offset=84)["v"].astype(np.float64)   # (count, 3, 3), mm
    flat = cad.reshape(-1, 3)
    centre = (flat.min(axis=0) + flat.max(axis=0)) / 2.0
    out = np.empty_like(cad)
    out[..., 0] = (centre[2] - cad[..., 2]) / 1000.0             # forward: CAD -Z
    out[..., 1] = (centre[0] - cad[..., 0]) / 1000.0             # port: CAD -X (a rotation, not a mirror)
    out[..., 2] = (cad[..., 1] - HULL_BOTTOM_MM) / 1000.0 + HULL_BOTTOM_M
    return out


def save(path: pathlib.Path, triangles: np.ndarray, origin: tuple[float, float, float], title: str) -> None:
    local = triangles - np.array(origin)
    normals = np.cross(local[:, 1] - local[:, 0], local[:, 2] - local[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    result = np.zeros(len(local), dtype=RECORD)
    result["n"] = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0)
    result["v"] = local
    with open(path, "wb") as f:
        f.write(f"VEDC buoy {title}, metres, x forward y port z up".encode().ljust(80, b" "))
        f.write(struct.pack("<I", len(local)))
        f.write(result.tobytes())
    flat = local.reshape(-1, 3)
    print(f"{path.name:20s} {len(local):6d} triangles  x {flat[:, 0].min():6.3f}..{flat[:, 0].max():6.3f}"
          f"  y {flat[:, 1].min():6.3f}..{flat[:, 1].max():6.3f}  z {flat[:, 2].min():6.3f}..{flat[:, 2].max():6.3f}")


def main() -> None:
    source, out_dir = sys.argv[1], pathlib.Path(sys.argv[2])
    out_dir.mkdir(parents=True, exist_ok=True)
    tri = load(source)
    mid, low, high = tri.mean(axis=1), tri.min(axis=1), tri.max(axis=1)

    # Under the hull bottoms at the stern: struts, motors, propellers.
    rear = (high[:, 0] < -0.30) & (high[:, 2] <= HULL_BOTTOM_M + 1e-4) & (mid[:, 2] < HULL_BOTTOM_M - 1e-3)
    rear_left, rear_right = rear & (mid[:, 1] > 0), rear & (mid[:, 1] < 0)
    # In the tunnel: the front motor with its mount, from the tunnel roof down.
    front = (np.abs(mid[:, 1]) < 0.075) & (low[:, 0] > 0.10) & (high[:, 0] < 0.27) & (mid[:, 2] < 0.14)
    # On the bow box: stem and camera body, above the two base plates.
    camera = (mid[:, 0] > 0.38) & (np.abs(mid[:, 1]) < 0.03) & (low[:, 2] >= 0.27 - 1e-4)
    hull = ~(rear | front | camera)
    for name, mask in (("rear left pod", rear_left), ("rear right pod", rear_right), ("front pod", front), ("camera", camera)):
        if not mask.any():
            raise SystemExit(f"the CAD has changed: no triangles found for the {name}")

    save(out_dir / "hull.stl", tri[hull], (0.0, 0.0, 0.0), "hull")
    save(out_dir / "pod_rear_left.stl", tri[rear_left], POD_REAR_LEFT, "rear left pod")
    save(out_dir / "pod_rear_right.stl", tri[rear_right], POD_REAR_RIGHT, "rear right pod")
    # Lowering the front motor: its link origin moves down with it, so the mesh in the link frame is unchanged.
    save(out_dir / "pod_front.stl", tri[front], (POD_FRONT[0], POD_FRONT[1], POD_FRONT[2] + FRONT_DROP_M), "front pod")
    save(out_dir / "camera.stl", tri[camera], CAMERA, "camera")


if __name__ == "__main__":
    main()
