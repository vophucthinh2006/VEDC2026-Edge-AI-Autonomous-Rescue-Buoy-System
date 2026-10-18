"""Find out what is on each serial port of the Pi: Pixhawk, STM32F103 helper board, STM32F407 or LiDAR.

On the Pixhawk boat the three links may all go through USB-to-TTL adapters. Those are often the
same chip (CP2102) without a serial number, so /dev/ttyUSB0..2 can swap between boots; the
/dev/serial/by-path names follow the physical USB socket and do not. Run on the Pi:
    python3 tools/port_probe.py                      # every by-path port, /dev/serial0, /dev/ttyAMA3
    python3 tools/port_probe.py /dev/ttyUSB0 /dev/ttyUSB1
For each port it listens at the bauds the boat uses, sends the LDS-008 start command at 115200,
and names what answered. At the end it prints the lines for config/boat_pixhawk.yaml.
Nothing must be running on these ports (stop rescue_main.py first). The LiDAR is told to stop
again; the Pixhawk and the helper board ignore the bytes.
"""
from __future__ import annotations

import glob
import sys
from pathlib import Path
from time import monotonic

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from modules.lidar_ld14 import LD14Decoder  # noqa: E402
from modules.lidar_lds008 import START_COMMAND, STOP_COMMAND, LDS008Decoder  # noqa: E402

BAUDS = (115200, 921600, 57600, 230400)   # helper board and LDS-008, TELEM2 after boat.parm, TELEM2 default, LD14
LISTEN_S = 1.5


def heartbeats(data: bytes) -> int:
    """MAVLink HEARTBEATs with a valid checksum in data; -1 without pymavlink."""
    try:
        from pymavlink.dialects.v20 import ardupilotmega as mavlink
    except ImportError:
        return -1
    parser = mavlink.MAVLink(None)
    parser.robust_parsing = True
    try:
        messages = parser.parse_buffer(data) or []
    except mavlink.MAVError:
        return 0
    return sum(1 for message in messages if message.get_type() == "HEARTBEAT")


def classify(data: bytes, sent: bytes) -> str:
    """What sent these bytes, or "" for nothing recognised."""
    if sent and data.startswith(sent):
        return "echo: TX and RX of this adapter are joined"
    if b"$AUX" in data:
        return "helper"
    if b"$IMU," in data and b"$SYS," in data:
        return "stm32f407"
    beats = heartbeats(data)
    if beats > 0:
        return "mavlink"
    if beats < 0 and data.count(0xFD) >= 2:
        return "mavlink (guessed from 0xFD, pymavlink missing)"
    if LDS008Decoder().feed(data):
        return "lds008"
    if LD14Decoder().feed(data):
        return "ld14"
    return ""


def probe(port: str) -> tuple[str, int] | None:
    """(what, baud) of the first baud that gave something recognisable."""
    for baud in BAUDS:
        try:
            ser = serial.Serial(port, baud, timeout=0.1)
        except (serial.SerialException, OSError) as exc:
            print(f"  cannot open: {exc}")
            return None
        with ser:
            sent = START_COMMAND if baud == 115200 else b""
            ser.write(sent)
            data = bytearray()
            deadline = monotonic() + LISTEN_S
            while monotonic() < deadline:
                data.extend(ser.read(1024))
            if sent:
                ser.write(STOP_COMMAND)
        what = classify(bytes(data), sent)
        print(f"  @{baud}: {len(data)} B" + (f" -> {what}" if what else (f", unknown: {bytes(data[:16]).hex(' ')}" if data else "")))
        if what:
            return what, baud
    return None


def main() -> int:
    ports = sys.argv[1:] or sorted(glob.glob("/dev/serial/by-path/*")) + [p for p in ("/dev/serial0", "/dev/ttyAMA3") if Path(p).exists()]
    if not ports:
        print("no serial ports found")
        return 1
    found: dict[str, tuple[str, int]] = {}
    for port in ports:
        print(port)
        result = probe(port)
        if result:
            found[port] = result
        else:
            print("  nothing recognised: device off, TX/RX not crossed (adapter TXD -> device RX), or no common GND")
    print()
    lines, serial_lines = [], []
    for port, (what, baud) in found.items():
        if what == "helper":
            lines += ["helper:", f"  port: {port}"]
        elif what.startswith("mavlink"):
            lines += ["mavlink:", f"  url: {port}", f"  baud: {baud}"]
        elif what == "stm32f407":
            serial_lines += [f"  stm32_port: {port}", f"  stm32_baud: {baud}"]
        elif what in ("lds008", "ld14"):
            serial_lines += [f"  lidar_model: {what}", f"  lidar_port: {port}", f"  lidar_baud: {baud}"]
    if serial_lines:
        lines += ["serial:"] + serial_lines
    if lines:
        print("for config/boat_stm32.yaml" if any("stm32_port" in line for line in lines) else "for config/boat_pixhawk.yaml", end=":\n")
        print("\n".join(lines))
    else:
        print("nothing found on any port")
    return 0 if lines else 1


if __name__ == "__main__":
    raise SystemExit(main())
