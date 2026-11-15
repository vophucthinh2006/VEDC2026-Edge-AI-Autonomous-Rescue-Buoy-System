#!/usr/bin/env python3
"""Raspberry Pi 5: main.py with the STM32 boat's settings and the Pi 5 ports.

    python3 run_pi5.py [main.py options]          the complete system (config/pi5_system.yaml)
    python3 run_pi5.py --buoy-test [options]      one buoy, no shore route, no LiDAR (config/pi5_buoy_test.yaml)

The same main.py as the simulator and the Pi 4: only the overlays differ. A later --overlay wins.
"""
from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main() -> None:
    args = sys.argv[1:]
    test = "--buoy-test" in args
    args = [a for a in args if a != "--buoy-test"]
    overlays = ["--overlay", "config/boat_stm32.yaml", "--overlay", f"config/{'pi5_buoy_test' if test else 'pi5_system'}.yaml"]
    os.chdir(HERE)
    sys.path.insert(0, str(HERE))
    sys.argv = [str(HERE / "main.py"), *overlays, *(["--no-lidar"] if test else []), *args]
    runpy.run_path(str(HERE / "main.py"), run_name="__main__")


if __name__ == "__main__":
    main()
