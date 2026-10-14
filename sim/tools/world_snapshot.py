#!/usr/bin/env python3
"""Save one frame of a Gazebo camera topic as a picture. Run under sim/env.sh, with the world running.

    python3 sim/tools/world_snapshot.py /vedc_central/overview overview.png     # the whole scene from above
    python3 sim/tools/world_snapshot.py /vedc_buoy/camera buoy.png              # what the buoy's camera sees
"""
from __future__ import annotations

import sys
import threading

import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        return 2
    topic, path = sys.argv[1], sys.argv[2]
    got = threading.Event()
    frames: list[np.ndarray] = []

    def on_image(msg: Image) -> None:
        frames.append(np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)[:, :, ::-1].copy())
        if len(frames) >= 2:        # the first frame of a camera is often rendered before the scene has loaded
            got.set()

    node = Node()
    if not node.subscribe(Image, topic, on_image):
        print(f"cannot subscribe to {topic}", file=sys.stderr)
        return 1
    if not got.wait(timeout=60.0):
        print(f"no picture on {topic} within 60 s", file=sys.stderr)
        return 1
    cv2.imwrite(path, frames[-1])
    print(f"{path}: {frames[-1].shape[1]} x {frames[-1].shape[0]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
