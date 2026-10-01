"""Read BNO055 Roll/Pitch/Yaw from a running STM32F4 through ST-LINK.

The symbol addresses come from the selected ELF, so the ELF must match the
firmware flashed on the target. The target keeps running; ST-LINK_CLI connects
in HOTPLUG mode and only reads RAM.

    python Tools/bno055_monitor.py --once
    python Tools/bno055_monitor.py
    python Tools/bno055_monitor.py --elf build/Release/STM32F4.elf
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import struct
import subprocess
import sys
import time


HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEFAULT_STLINK_CLI = os.path.join(
    os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"),
    "STMicroelectronics",
    "STM32 ST-LINK Utility",
    "ST-LINK Utility",
    "ST-LINK_CLI.exe",
)

STATUS = {
    0: "OK",
    1: "ERR_PARAM",
    2: "ERR_I2C",
    3: "ERR_TIMEOUT",
    4: "ERR_CHIP_ID",
    5: "ERR_MODE",
    6: "ERR_SYS",
    7: "ERR_PROFILE",
}


def find_tool(name: str, fallback: str | None = None) -> str:
    path = shutil.which(name)
    if path:
        return path
    cache = os.path.join(ROOT, "build", "Debug", "CMakeCache.txt")
    if os.path.exists(cache):
        text = open(cache, encoding="utf-8", errors="replace").read()
        match = re.search(r"CMAKE_C_COMPILER_AR:FILEPATH=(.*)", text)
        if match:
            path = os.path.join(os.path.dirname(match.group(1).strip()), name + ".exe")
            if os.path.exists(path):
                return path
    if fallback and os.path.exists(fallback):
        return fallback
    sys.exit(f"{name} not found")


def load_symbols(elf: str) -> dict[str, int]:
    nm = find_tool("arm-none-eabi-nm")
    output = subprocess.run(
        [nm, "-n", elf], capture_output=True, text=True, check=True
    ).stdout
    table: dict[str, int] = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) == 3:
            table[parts[2]] = int(parts[0], 16)
    required = (
        "bno055_init_status",
        "bno055_read_status",
        "bno055_euler",
        "bno055_calib",
        "hbno055",
        "uwTick",
    )
    missing = [name for name in required if name not in table]
    if missing:
        sys.exit(f"symbols not found in {elf}: {', '.join(missing)}")
    return table


def parse_memory(output: str) -> dict[int, bytes]:
    memory: dict[int, bytes] = {}
    pattern = re.compile(r"^0x([0-9A-Fa-f]{8})\s*:\s*((?:[0-9A-Fa-f]{2}\s+)*[0-9A-Fa-f]{2})\s*$")
    for line in output.splitlines():
        match = pattern.match(line.strip())
        if match:
            memory[int(match.group(1), 16)] = bytes.fromhex(match.group(2))
    return memory


def memory_block(memory: dict[int, bytes], address: int, size: int) -> bytes:
    data = bytearray()
    cursor = address
    while len(data) < size:
        chunk = memory.get(cursor)
        if chunk is None:
            raise RuntimeError(f"ST-LINK output did not contain 0x{cursor:08X}")
        data.extend(chunk)
        cursor += len(chunk)
    return bytes(data[:size])


def read_snapshot(cli: str, symbols: dict[str, int]) -> dict[str, object]:
    reads = (
        (symbols["bno055_init_status"], 1),
        (symbols["bno055_read_status"], 1),
        (symbols["bno055_euler"], 12),
        (symbols["bno055_calib"], 4),
        (symbols["hbno055"], 32),
        (symbols["uwTick"], 4),
    )
    command = [cli, "-c", "SWD", "HOTPLUG", "-Q"]
    for address, size in reads:
        command.extend(("-r8", f"0x{address:08X}", f"0x{size:X}"))
    result = subprocess.run(command, capture_output=True, text=True)
    output = result.stdout + result.stderr
    if result.returncode != 0:
        raise RuntimeError(output.strip())
    memory = parse_memory(output)
    init_data = memory_block(memory, symbols["bno055_init_status"], 1)
    read_data = memory_block(memory, symbols["bno055_read_status"], 1)
    euler_data = memory_block(memory, symbols["bno055_euler"], 12)
    calibration = tuple(memory_block(memory, symbols["bno055_calib"], 4))
    handle = memory_block(memory, symbols["hbno055"], 32)
    tick = struct.unpack("<I", memory_block(memory, symbols["uwTick"], 4))[0]
    roll, pitch, yaw = struct.unpack("<fff", euler_data)
    return {
        "init": init_data[0],
        "read": read_data[0],
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw,
        "calibration": calibration,
        "address": int.from_bytes(handle[4:6], "little"),
        "ext_crystal": handle[28],
        "sys_err": handle[29],
        "tick": tick,
    }


def print_snapshot(snapshot: dict[str, object]) -> None:
    init = int(snapshot["init"])
    read = int(snapshot["read"])
    sys_cal, gyr_cal, acc_cal, mag_cal = snapshot["calibration"]
    print(
        f"uptime={int(snapshot['tick']) / 1000.0:8.1f}s  "
        f"addr=0x{int(snapshot['address']):02X}  "
        f"init={STATUS.get(init, str(init))}  read={STATUS.get(read, str(read))}"
    )
    print(
        f"Roll={float(snapshot['roll']):8.2f} deg  "
        f"Pitch={float(snapshot['pitch']):8.2f} deg  "
        f"Yaw={float(snapshot['yaw']):8.2f} deg"
    )
    print(
        f"Calib SYS/GYR/ACC/MAG={sys_cal}/{gyr_cal}/{acc_cal}/{mag_cal}  "
        f"ext_crystal={int(snapshot['ext_crystal'])}  sys_err={int(snapshot['sys_err'])}"
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--elf", default=os.path.join(ROOT, "build", "Debug", "STM32F4.elf"))
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()
    elf = os.path.abspath(args.elf)
    if not os.path.exists(elf):
        sys.exit(f"ELF not found: {elf}")

    cli = find_tool("ST-LINK_CLI", DEFAULT_STLINK_CLI)
    symbols = load_symbols(elf)
    try:
        while True:
            snapshot = read_snapshot(cli, symbols)
            if not args.once:
                os.system("cls" if os.name == "nt" else "clear")
            print_snapshot(snapshot)
            if args.once:
                return 0
            time.sleep(max(args.interval, 0.1))
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"BNO055 monitor failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
