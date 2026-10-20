"""Count what the STM32F407 sends on its UART for a few seconds, by line type, with the last of each.

Checks the Pi <-> F407 link with the normal firmware (no PI_LINK_TEST_ENABLED build needed): the
F407 sends $IMU, $GPS, $SYS every 100 ms and $LNK every second whether or not the Pi talks. Stop
main.py first. From Rasp_Pi:
    python3 tools/stm32_lines.py                 # serial.stm32_port of config/boat_stm32.yaml, 5 s
    python3 tools/stm32_lines.py PORT [seconds]
"""
from __future__ import annotations

import sys
import time
from collections import Counter
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from utils.config import load_config  # noqa: E402
from utils.nmea_packet import decode  # noqa: E402

EXPECTED = {"IMU": 10.0, "GPS": 10.0, "SYS": 10.0, "LNK": 1.0}   # lines a second


def main() -> int:
    config = load_config(Path("config/settings.yaml"), [Path("config/boat_stm32.yaml")])
    port = sys.argv[1] if len(sys.argv) > 1 else config["serial"]["stm32_port"]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    baud = int(config["serial"]["stm32_baud"])
    counts: Counter[str] = Counter()
    last: dict[str, str] = {}
    bad = 0
    with serial.Serial(port, baud, timeout=0.5) as link:
        print(f"listening on {port} @ {baud} for {seconds:.0f} s")
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            raw = link.readline()
            if not raw:
                continue
            packet = decode(raw.strip())
            if packet is None:
                bad += 1
                continue
            counts[packet.command] += 1
            last[packet.command] = ",".join(packet.fields)
    ok = True
    for command, rate in EXPECTED.items():
        got = counts.get(command, 0)
        good = got >= 0.7 * rate * seconds
        ok &= good
        print(f"{'ok  ' if good else 'LOW '} ${command}: {got} lines ({got / seconds:.1f}/s, expected {rate:.0f}/s)  last: {last.get(command, '-')}")
    for command in sorted(set(counts) - set(EXPECTED)):
        print(f"     ${command}: {counts[command]} lines  last: {last[command]}")
    print(f"{bad} lines with a bad checksum or shape")
    if not counts:
        print("nothing heard: wrong port, TX/RX not crossed, no common GND, or the F407 not running")
    print("PASS" if ok and bad == 0 else "FAIL")
    return 0 if ok and bad == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
