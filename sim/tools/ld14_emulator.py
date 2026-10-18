#!/usr/bin/env python3
"""The LD14 LiDAR in the simulator: the Gazebo LiDAR of the buoy, sent as the LD14's serial byte
stream (47-byte packets, 0x54 0x2C, Rasp_Pi/modules/lidar_ld14.py) on a pseudo-terminal.

The Pi then reads its obstacles the way it will on the boat: SerialLidar -> LD14Decoder -> scan,
instead of taking the Gazebo topic directly. Like the real module, it streams as soon as it runs:
the LD14 spins on power, it takes no start command.

    python3 ld14_emulator.py --pty /tmp/vedc_lidar --topic /vedc_buoy/lidar
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
import tty
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "Rasp_Pi"))

from gz.msgs10.laserscan_pb2 import LaserScan  # noqa: E402
from gz.transport13 import Node  # noqa: E402

from modules.lidar_ld14 import POINTS_PER_PACKET, encode_scan  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pty", required=True, type=Path, help="path the Pi opens as serial.lidar_port (a link to the pseudo-terminal)")
    parser.add_argument("--topic", default="/vedc_buoy/lidar")
    args = parser.parse_args()

    master, slave = os.openpty()
    tty.setraw(slave)
    if args.pty.is_symlink() or args.pty.exists():
        args.pty.unlink()
    args.pty.symlink_to(os.ttyname(slave))
    os.set_blocking(master, False)

    latest: list[bytes] = []
    fresh = threading.Event()
    started = time.monotonic()

    def on_scan(msg: LaserScan) -> None:
        # Gazebo angles grow anticlockwise seen from above; the LD14's grow clockwise from its zero,
        # set at the bow, so the Pi needs no yaw offset and lidar_anticlockwise stays false.
        count = len(msg.ranges) - len(msg.ranges) % POINTS_PER_PACKET
        if count <= 0:
            return
        step = 360.0 / count
        distances: list[float | None] = [None] * count
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or not msg.range_min <= distance <= msg.range_max:
                continue
            sample = round((-math.degrees(msg.angle_min + index * msg.angle_step)) % 360.0 / step) % count
            if distances[sample] is None or distance < distances[sample]:
                distances[sample] = distance
        latest[:] = [encode_scan(distances, timestamp_ms=int((time.monotonic() - started) * 1000))]
        fresh.set()

    node = Node()
    if not node.subscribe(LaserScan, args.topic, on_scan):
        print(f"cannot subscribe to {args.topic}", file=sys.stderr)
        return 1
    print(f"LD14 on {args.pty} -> {os.ttyname(slave)}, from {args.topic}", flush=True)
    try:
        while True:
            if fresh.wait(timeout=0.2):
                fresh.clear()
                try:
                    os.write(master, latest[0])
                except (BlockingIOError, OSError):
                    pass                        # the Pi is not reading: that revolution is lost
    except KeyboardInterrupt:
        pass
    finally:
        if args.pty.is_symlink():
            args.pty.unlink()
    return 0


if __name__ == "__main__":
    sys.exit(main())
