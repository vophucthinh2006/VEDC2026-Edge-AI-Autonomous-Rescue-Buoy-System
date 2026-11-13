#!/usr/bin/env python3
"""Start the Raspberry Pi + STM32F407 buoy-only test on Raspberry Pi 5.

This launcher selects the test overlay and runs the unchanged STM32 runtime in
runtime/main.py. It detects and processes people, drives the camera servo with
$CAM, and sends motor/steering commands with $NAV. All arguments are passed through unchanged; a later --overlay overrides
the Pi 5 defaults.
"""
from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent
RUNTIME = ROOT / "runtime"


def main() -> None:
    os.chdir(RUNTIME)
    sys.path.insert(0, str(RUNTIME))
    sys.argv = [
        str(RUNTIME / "main.py"),
        "--overlay",
        "config/pi5_buoy_test.yaml",
        "--no-lidar",
        *sys.argv[1:],
    ]
    runpy.run_path(str(RUNTIME / "main.py"), run_name="__main__")


if __name__ == "__main__":
    main()
