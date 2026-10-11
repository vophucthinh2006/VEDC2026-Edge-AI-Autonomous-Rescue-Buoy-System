#!/usr/bin/env python3
"""The LDS-008 LiDAR in the simulator: the Gazebo LiDAR of the buoy, sent as the serial byte
stream the Pi's decoder expects (Rasp_Pi/modules/lidar_lds008.py), on a pseudo-terminal.

The Pi then reads its obstacles the way it will on the boat: SerialLidar -> LDS008Decoder ->
ray_obstacles, instead of taking the Gazebo topic directly. Like the real module, nothing is sent
until the Pi writes "startlds$".

The format is the assumed one (LDS-006 / Neato XV-11): this proves the Pi's serial path and what
happens when the LiDAR goes silent, not that the real LDS-008 speaks this format. That is still
to be checked on the module with Rasp_Pi/tools/lidar_capture.py.

    python3 lds008_emulator.py --pty /tmp/vedc_lidar --topic /vedc_buoy/lidar
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

from modules.lidar_lds008 import START_COMMAND, STOP_COMMAND, encode_scan  # noqa: E402


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

    def on_scan(msg: LaserScan) -> None:
        # Gazebo angles grow anticlockwise seen from above; the module's index is taken clockwise
        # from the bow, so the Pi needs no yaw offset and lidar_anticlockwise stays false.
        distances: list[float | None] = [None] * 360
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or not msg.range_min <= distance <= msg.range_max:
                continue
            degree = round(-math.degrees(msg.angle_min + index * msg.angle_step)) % 360
            if distances[degree] is None or distance < distances[degree]:
                distances[degree] = distance
        latest[:] = [encode_scan(distances)]
        fresh.set()

    node = Node()
    if not node.subscribe(LaserScan, args.topic, on_scan):
        print(f"cannot subscribe to {args.topic}", file=sys.stderr)
        return 1
    print(f"LDS-008 on {args.pty} -> {os.ttyname(slave)}, from {args.topic}", flush=True)
    spinning, command = False, b""
    try:
        while True:
            try:
                command = (command + os.read(master, 256))[-16:]
            except (BlockingIOError, OSError):
                pass
            if START_COMMAND in command:
                spinning, command = True, b""
                print("startlds: spinning", flush=True)
            elif STOP_COMMAND in command:
                spinning, command = False, b""
                print("stoplds: stopped", flush=True)
            if fresh.wait(timeout=0.05):
                fresh.clear()
                if spinning and latest:
                    try:
                        os.write(master, latest[0])
                    except (BlockingIOError, OSError):
                        pass                        # the Pi is not reading: that revolution is lost
            else:
                time.sleep(0.0)
    except KeyboardInterrupt:
        pass
    finally:
        if args.pty.is_symlink():
            args.pty.unlink()
    return 0


if __name__ == "__main__":
    sys.exit(main())
