#!/usr/bin/env python3
"""The STM32F103 helper board in the simulator.

Runs the firmware's own logic (STM32F103/Tests/helper_sim.c, compiled for the PC) behind a
pseudo-terminal, so Rasp_Pi/modules/helper_board.py talks to it as it will to the real board on
/dev/ttyUSB0, and carries out what the board decides in the simulated world:

    camera servo pulse   ->  the camera_pan_joint of the Gazebo buoy
    LoRa beacon          ->  the cloud, as the ESP32 shore station would post it (with --ingest-url
                             and VEDC_INGEST_TOKEN set)
    E-stop button        <-  a file named "estop" in --dir: there = pressed

Everything the board does is appended to <dir>/helper_events.log ("<unix time> servo 1335").

    python3 helper_board_sim.py --pty /tmp/vedc_helper --binary ./helper_sim --dir . \
        --pan-topic /model/vedc_buoy/joint/camera_pan_joint/cmd_pos
"""
from __future__ import annotations

import argparse
import json
import math
import os
import select
import subprocess
import sys
import threading
import time
import tty
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
# App/app_config.h: pulse = CAM_CENTER_US + CAM_PAN_DIR * 5.5 us per degree.
CAM_CENTER_US, CAM_US_PER_DEG, CAM_PAN_DIR = 1500.0, 5.5, -1.0
FAKE_RSSI, FAKE_SNR = -72, 9.5      # a strong link: the simulator has no radio path


def station_json(frame: str) -> dict | None:
    """What the ESP32 station posts for one LoRa line (esp32-lora-station/main/lora_payload.c and
    main.c), or None for a line it would drop."""
    out: dict = {"rssi": FAKE_RSSI, "snr": FAKE_SNR, "raw": frame[:99]}
    coordinates, attitude = [], {}
    for token in frame.split(","):
        key, equals, value = token.strip().partition("=")
        if not equals:
            if "." in key and len(coordinates) < 2:
                try:
                    coordinates.append(float(key))
                except ValueError:
                    pass
        elif key == "id":
            out["id"] = value
        elif key == "v":
            if value.isdigit() and int(value) <= 255:
                out["victims"] = int(value)
        else:
            attitude[key] = value
    fix = len(coordinates) == 2 and abs(coordinates[0]) <= 90.0 and abs(coordinates[1]) <= 180.0
    out["fix"] = 1 if fix else 0
    if fix:
        out["lat"], out["lon"] = coordinates
    try:
        if set("rpytmicq") <= set(attitude):
            out.update(roll=float(attitude["r"]), pitch=float(attitude["p"]), yaw=float(attitude["y"]), target_yaw=float(attitude["t"]),
                       mode=attitude["m"], imu_ok=int(attitude["i"]), calib=int(attitude["c"]), seq=int(attitude["q"]))
    except ValueError:
        pass                          # a bad attitude group costs only the attitude
    return out


def post(url: str, token: str, body: dict) -> None:
    request = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                     headers={"Content-Type": "application/json", "Authorization": "Bearer " + token})
    try:
        urllib.request.urlopen(request, timeout=3).close()
    except OSError as exc:
        print(f"ingest failed: {exc}", file=sys.stderr, flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pty", required=True, type=Path, help="path the Pi opens as helper.port (a link to the pseudo-terminal)")
    parser.add_argument("--binary", required=True, type=Path, help="helper_sim, compiled from STM32F103/Tests/helper_sim.c")
    parser.add_argument("--dir", type=Path, default=Path("."), help="where helper_events.log goes and the estop file is looked for")
    parser.add_argument("--pan-topic", help="Gazebo topic of the camera joint; without it the servo is only logged")
    parser.add_argument("--ingest-url", help="cloud /api/ingest; the beacons are posted there with VEDC_INGEST_TOKEN")
    args = parser.parse_args()

    master, slave = os.openpty()
    tty.setraw(slave)                 # no echo, no line editing: bytes as on a UART
    if args.pty.is_symlink() or args.pty.exists():
        args.pty.unlink()
    args.pty.symlink_to(os.ttyname(slave))
    os.set_blocking(master, False)

    send_pan = None
    if args.pan_topic:
        sys.path.insert(0, str(REPO / "Rasp_Pi"))
        from modules.gazebo_io import GazeboPan     # the publisher the Pi itself uses without the board
        send_pan = GazeboPan(args.pan_topic)
    token = os.environ.get("VEDC_INGEST_TOKEN", "")
    board = subprocess.Popen([str(args.binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=0)
    events = (args.dir / "helper_events.log").open("a", buffering=1)
    estop_file, estop = args.dir / "estop", False
    pending = b""
    print(f"helper board on {args.pty} -> {os.ttyname(slave)}", flush=True)
    try:
        while board.poll() is None:
            readable, _, _ = select.select([master, board.stdout], [], [], 0.05)
            if master in readable:
                try:
                    board.stdin.write(os.read(master, 1024))
                except (BlockingIOError, OSError):
                    pass
            if board.stdout in readable:
                pending += os.read(board.stdout.fileno(), 4096)
                *lines, pending = pending.split(b"\n")
                for line in lines:
                    if line.startswith(b"$"):
                        try:
                            os.write(master, line + b"\n")
                        except (BlockingIOError, OSError):
                            pass                    # nobody reads the port: the report is lost, as on a real UART
                        continue
                    kind, _, value = line.decode("ascii", "replace")[2:].partition(" ")
                    events.write(f"{time.time():.2f} {kind} {value}\n")
                    if kind == "servo" and send_pan is not None:
                        send_pan((float(value) - CAM_CENTER_US) / (CAM_US_PER_DEG * CAM_PAN_DIR))
                    elif kind == "beacon" and args.ingest_url and token:
                        body = station_json(value)
                        if body is not None:
                            threading.Thread(target=post, args=(args.ingest_url, token, body), daemon=True).start()
            pressed = estop_file.exists()
            if pressed != estop:
                estop = pressed
                board.stdin.write(b"!estop 1\n" if estop else b"!estop 0\n")
    except KeyboardInterrupt:
        pass
    finally:
        board.kill()
        if args.pty.is_symlink():
            args.pty.unlink()
    return 0


if __name__ == "__main__":
    sys.exit(main())
