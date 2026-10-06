"""Record raw bytes from the LDS-008 and check them against the assumed packet format.

The LDS-008 has no datasheet; modules/lidar_lds008.py follows its sibling LDS-006 (Neato XV-11
format). Run this on the real module before relying on the decoder:
    python3 tools/lidar_capture.py                       # /dev/ttyAMA3 @ 115200, 5 s
    python3 tools/lidar_capture.py /dev/ttyUSB0 115200 10
The raw bytes go to lidar_capture.bin. If "valid packets" stays at 0, send that file along:
the format is different and the decoder has to be rewritten from it.
"""
from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path
from time import monotonic

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from modules.lidar_lds008 import LDS008Decoder, PACKET_SIZE, START_COMMAND, STOP_COMMAND  # noqa: E402


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA3"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
    raw = bytearray()
    with serial.Serial(port, baud, timeout=0.1) as ser:
        ser.write(START_COMMAND)
        print(f"sent {START_COMMAND!r}, recording {port} @ {baud} for {seconds:.0f} s")
        deadline = monotonic() + seconds
        while monotonic() < deadline:
            raw.extend(ser.read(256))
        ser.write(STOP_COMMAND)
    Path("lidar_capture.bin").write_bytes(raw)
    print(f"{len(raw)} bytes -> lidar_capture.bin")
    if not raw:
        print("nothing received: check power, the module's TX -> Pi RX wire and the baud rate")
        return 1
    print("first 64 bytes:", raw[:64].hex(" "))
    print("most common bytes:", ", ".join(f"0x{byte:02X} x{count}" for byte, count in Counter(raw).most_common(5)))
    decoder = LDS008Decoder()
    points = decoder.feed(bytes(raw))
    print(f"valid packets: {len(raw) // PACKET_SIZE} possible, {decoder.bad_checksums} rejected starts, {len(points)} points")
    if points:
        distances = [point.distance_m for point in points]
        print(f"angles {min(p.angle_deg for p in points):.0f}..{max(p.angle_deg for p in points):.0f} deg, "
              f"range {min(distances):.2f}..{max(distances):.2f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
