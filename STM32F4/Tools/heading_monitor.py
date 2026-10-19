"""Watch the heading PID live on a running STM32F4 through ST-LINK.

Reads heading_debug (App/control.h) and actuator_debug from RAM while the
target keeps running. The ELF must match the flashed firmware.

    python Tools/heading_monitor.py              # refresh every 0.2 s
    python Tools/heading_monitor.py --csv run.csv  # also log every sample

Desk check: turn the IMU clockwise (seen from above) -> yaw rises and
rate is positive. With the Pi driving (AUTO), turning the IMU right of the target
must give u < 0: rudders swing the bow left and the RIGHT motor speeds up.
There is no RC: the boat is armed whenever it is safe and the Pi decides when it moves.
"""
from __future__ import annotations

import argparse
import os
import struct
import subprocess
import sys
import time

from bno055_monitor import DEFAULT_STLINK_CLI, ROOT, find_tool, load_symbols, memory_block, parse_memory

STATE = {0: "OFF", 1: "AUTO"}
BLOCK = {  # drive_block_t in App/control.h
    0: "-",
    1: "not armed",
    2: "the Pi is not driving: no route from the dashboard, NAV STOP, or no NAV/HBT",
    3: "IMU stale (Euler or gyro read failing)",
}
ARM_BLOCK = {  # arm_block_t in App/control.h
    0: "-",
    1: "IMU stale",
    2: "overturned: roll or pitch above 35 deg",
}
HEADING_FORMAT = "<13fH6B"          # heading_debug_t: 13 floats, imu_fail, state, imu_ok, block, armed, arm_block, last_disarm
HEADING_SIZE = struct.calcsize(HEADING_FORMAT)
ESC_NAMES = ("front", "right", "left")  # esc_id_t order
SERVO_NAMES = ("cam", "front", "right", "left")  # servo_id_t order
ACTUATOR_SIZE = 16 + 2 * 3 + 2 * 4   # actuator_cmd_t + esc_us[3] + servo_us[4]


def read_snapshot(cli: str, symbols: dict[str, int]) -> dict[str, object]:
    reads = ((symbols["heading_debug"], HEADING_SIZE), (symbols["actuator_debug"], ACTUATOR_SIZE), (symbols["uwTick"], 4))
    command = [cli, "-c", "SWD", "HOTPLUG", "-Q"]
    for address, size in reads:
        command.extend(("-r8", f"0x{address:08X}", f"0x{size:X}"))
    result = subprocess.run(command, capture_output=True, text=True)
    output = result.stdout + result.stderr
    if result.returncode != 0:
        raise RuntimeError(output.strip())
    memory = parse_memory(output)
    values = struct.unpack(HEADING_FORMAT, memory_block(memory, symbols["heading_debug"], HEADING_SIZE))
    actuators = memory_block(memory, symbols["actuator_debug"], ACTUATOR_SIZE)
    names = ("target", "yaw", "rate", "error", "p", "i", "d", "u", "steer", "left", "right", "roll", "pitch",
             "imu_fail", "state", "imu_ok", "block", "armed", "arm_block", "last_disarm")
    snap: dict[str, object] = dict(zip(names, values))
    snap["esc_us"] = struct.unpack("<3H", actuators[16:22])
    snap["servo_us"] = struct.unpack("<4H", actuators[22:30])
    snap["tick"] = struct.unpack("<I", memory_block(memory, symbols["uwTick"], 4))[0]
    return snap


def print_snapshot(s: dict[str, object]) -> None:
    target = float(s["target"])
    print(f"uptime={int(s['tick']) / 1000.0:8.1f}s  state={STATE.get(int(s['state']), s['state'])}  "
          f"armed={int(s['armed'])}  imu={'OK' if s['imu_ok'] else 'STALE'}")
    print(f"roll={float(s['roll']):6.1f}  pitch={float(s['pitch']):6.1f}  IMU read fails={s['imu_fail']}")
    if int(s["last_disarm"]):
        print(f"last disarm: {ARM_BLOCK.get(int(s['last_disarm']), s['last_disarm'])}")
    if not int(s["armed"]):
        print(f"ARM blocked: {ARM_BLOCK.get(int(s['arm_block']), s['arm_block'])}")
    elif int(s["block"]) != 0:
        print(f"not driving: {BLOCK.get(int(s['block']), s['block'])}")
    print(f"yaw={float(s['yaw']):7.1f} deg  rate={float(s['rate']):7.1f} deg/s  "
          f"target={'  -  ' if target < 0 else f'{target:7.1f}'}  error={float(s['error']):7.1f}")
    print(f"P={float(s['p']):+.3f}  I={float(s['i']):+.3f}  D={float(s['d']):+.3f}  ->  u={float(s['u']):+.3f}"
          f"   ({'bow RIGHT' if float(s['u']) > 0.01 else 'bow LEFT' if float(s['u']) < -0.01 else 'straight'})")
    print(f"cmd: steer={float(s['steer']):+.2f}  rear L={float(s['left']):.2f}  R={float(s['right']):.2f}")
    esc = ", ".join(f"{n}={v}" for n, v in zip(ESC_NAMES, s["esc_us"]))
    servo = ", ".join(f"{n}={v}" for n, v in zip(SERVO_NAMES, s["servo_us"]))
    print(f"ESC us: {esc}\nservo us: {servo}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--elf", default=os.path.join(ROOT, "build", "Debug", "STM32F4.elf"))
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--csv", help="append every sample to this CSV file")
    parser.add_argument("--once", action="store_true", help="print one sample and exit")
    args = parser.parse_args()
    elf = os.path.abspath(args.elf)
    if not os.path.exists(elf):
        sys.exit(f"ELF not found: {elf}")
    cli = find_tool("ST-LINK_CLI", DEFAULT_STLINK_CLI)
    symbols = load_symbols(elf)
    for name in ("heading_debug", "actuator_debug"):
        if name not in symbols:
            sys.exit(f"symbol {name} not in {elf}: rebuild the firmware")
    log = open(args.csv, "a", encoding="utf-8") if args.csv else None
    if log and log.tell() == 0:
        log.write("t_s,state,target,yaw,rate,error,p,i,d,u,steer,left,right\n")
    try:
        while True:
            s = read_snapshot(cli, symbols)
            if not args.once:
                os.system("cls" if os.name == "nt" else "clear")
            print_snapshot(s)
            if args.once:
                return 0
            if log:
                log.write(f"{int(s['tick']) / 1000.0:.2f},{int(s['state'])}," + ",".join(
                    f"{float(s[k]):.3f}" for k in ("target", "yaw", "rate", "error", "p", "i", "d", "u", "steer", "left", "right")) + "\n")
                log.flush()
            time.sleep(max(args.interval, 0.05))
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f"heading monitor failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if log:
            log.close()


if __name__ == "__main__":
    raise SystemExit(main())
