"""Listen for the STM32 $TST link-test lines on UART4 and count what arrives.

Build the STM32 with PI_LINK_TEST_ENABLED 1, then on the Pi:
    python3 tools/stm32_link_test.py            # /dev/serial0 @ 115200
    python3 tools/stm32_link_test.py /dev/ttyAMA0 115200
"""
from __future__ import annotations

import sys
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from utils.nmea_packet import decode  # noqa: E402


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/serial0"
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    good = bad = lost = 0
    last_seq: int | None = None
    with serial.Serial(port, baud, timeout=1.0) as ser:
        print(f"listening on {port} @ {baud}, Ctrl+C to stop")
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    print("... nothing for 1 s")
                    continue
                packet = decode(raw)
                if packet is None:
                    bad += 1
                    print(f"BAD  {raw!r}")
                    continue
                good += 1
                if packet.command == "TST" and packet.fields:
                    seq = int(packet.fields[0])
                    if last_seq is not None:
                        lost += (seq - last_seq - 1) & 0xFFFF
                    last_seq = seq
                print(f"OK   {raw.decode('ascii').strip()}   good={good} bad={bad} lost={lost}")
        except KeyboardInterrupt:
            pass
    print(f"\ngood={good} bad={bad} lost={lost}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
