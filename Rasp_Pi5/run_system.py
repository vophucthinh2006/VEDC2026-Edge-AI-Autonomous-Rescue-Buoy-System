#!/usr/bin/env python3
"""Start the complete RedgeSCUE system on Raspberry Pi 5.

This launcher only selects the Pi 5 hardware overlay.  The application itself
is the unchanged STM32F407 runtime in runtime/main.py, which communicates with
the buoy, shore station, and web dashboard through the existing protocol.
All arguments are passed through unchanged; a later --overlay overrides the
Pi 5 defaults.
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
        "config/pi5_system.yaml",
        *sys.argv[1:],
    ]
    runpy.run_path(str(RUNTIME / "main.py"), run_name="__main__")


if __name__ == "__main__":
    main()
