"""Live view of the battery as the INA226 reads it, over ST-LINK (no halt).

    python Tools/battery_monitor.py              # refresh in place, Ctrl+C to stop
    python Tools/battery_monitor.py --once       # one snapshot

The struct layout below must match battery_debug_t (App/battery.h); update them together.
Compare the voltage with a multimeter on the pack: within 0.1 V. The current is right only once
INA226_SHUNT_OHM (App/app_config.h) is the shunt on your module.
"""
import argparse
import os
import struct
import sys
import time

from openocd_probe import OpenOcd, ROOT
from lora_monitor import symbols


def snapshot(ocd, address):
    return struct.unpack("<2fbBHIB", ocd.read(address, 17))


def fmt(s):
    volts, amps, percent, ok, fails, reads, i2c = s
    if not ok:
        return (f"INA226 not found at 0x40..0x4F on I2C1 (SCL PB8, SDA PB9): check 3.3 V, GND and the two "
                f"wires, the firmware looks again every 5 s | {reads} reads, {fails} failed")
    charge = "unknown" if percent < 0 else f"{percent} %"
    return f"battery {volts:6.2f} V  {amps:+6.2f} A  charge {charge} | INA226 at 0x{i2c:02X}, {reads} reads, {fails} failed"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", default=os.path.join(ROOT, "build", "Debug", "STM32F4.elf"))
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--period", type=float, default=0.5)
    args = ap.parse_args()

    table = symbols(args.elf)
    if "battery_debug" not in table:
        sys.exit(f"battery_debug not in {args.elf}: build and flash the current firmware first")
    try:
        ocd = OpenOcd()
    except ConnectionError as e:
        sys.exit(str(e))
    try:
        if args.once:
            print(fmt(snapshot(ocd, table["battery_debug"])))
            return
        while True:
            print("\r" + fmt(snapshot(ocd, table["battery_debug"])) + "   ", end="", flush=True)
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        ocd.close()


if __name__ == "__main__":
    main()
