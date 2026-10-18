"""Where the boat itself shows up in the LD14 scan, to set vehicle.lidar_min_range_m and lidar_mask.

With the boat in open space (nothing but the boat within max_m), run on the Pi from Rasp_Pi:
    python3 tools/lidar_near.py                          # serial.lidar_port of config/boat_stm32.yaml
    python3 tools/lidar_near.py PORT [seconds] [max_m]
Per 1-degree bin of the LiDAR's own angle: how often a return nearer than max_m shows, its
min/median/max range, and whether the current filter drops it. Stop main.py first.
"""
from __future__ import annotations

import statistics
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from modules.lidar_ld14 import LD14Decoder, LidarPoint, ScanAssembler  # noqa: E402
from modules.lidar_serial import drop_self_returns  # noqa: E402
from utils.config import load_config  # noqa: E402


def main() -> None:
    config = load_config(Path("config/settings.yaml"), [Path("config/boat_stm32.yaml")])
    port = sys.argv[1] if len(sys.argv) > 1 else config["serial"]["lidar_port"]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    max_m = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
    vehicle = config["vehicle"]
    min_range_m, mask = float(vehicle.get("lidar_min_range_m", 0.0)), list(vehicle.get("lidar_mask") or [])

    decoder, assembler, scans = LD14Decoder(), ScanAssembler(), []
    with serial.Serial(port, int(config["serial"]["lidar_baud"]), timeout=0.1) as link:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            scan = assembler.add(decoder.feed(link.read(512)))
            if scan:
                scans.append(scan)
    scans = scans[1:]   # the first may be a partial turn
    if not scans:
        sys.exit(f"no scan from {port}")
    bins: dict[int, list[float]] = {}
    for scan in scans:
        nearest: dict[int, float] = {}
        for p in scan:
            if p.intensity > 0 and 0.0 < p.distance_m < max_m:
                a = int(p.angle_deg) % 360
                nearest[a] = min(nearest.get(a, max_m), p.distance_m)
        for a, d in nearest.items():
            bins.setdefault(a, []).append(d)
    print(f"{len(scans)} scans, returns nearer than {max_m} m (min_range {min_range_m} m, mask {mask}):")
    for a in sorted(bins):
        d = bins[a]
        dropped = drop_self_returns((LidarPoint(a + 0.5, max(d), 1),), min_range_m, mask)[0].distance_m == 0.0
        print(f"  {a:3d} deg  in {len(d) / len(scans):4.0%} of scans  {min(d):.2f}/{statistics.median(d):.2f}/{max(d):.2f} m"
              f"  {'dropped' if dropped else 'KEPT'}")


if __name__ == "__main__":
    main()
