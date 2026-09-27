"""Live view of the RC link, arming state and actuator commands over ST-LINK.

Reads RAM while the firmware runs (no halt), through OpenOCD's Tcl port.
Symbol addresses come from the ELF, so it follows the build it is given.

    python Tools/rc_monitor.py              # refresh in place, Ctrl+C to stop
    python Tools/rc_monitor.py --once       # one snapshot
    python Tools/rc_monitor.py --log 120    # print only changes for 120 s

Struct layouts below must match ibus_state_t, controller_t and
actuator_debug_t; update them together.
"""
import argparse
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def find_tool(name):
    path = shutil.which(name)
    if path:
        return path
    cache = os.path.join(ROOT, "build", "Debug", "CMakeCache.txt")
    if os.path.exists(cache):
        m = re.search(r"CMAKE_C_COMPILER_AR:FILEPATH=(.*)", open(cache).read())
        if m:
            path = os.path.join(os.path.dirname(m.group(1).strip()), name + ".exe")
            if os.path.exists(path):
                return path
    sys.exit(f"{name} not found")


def symbols(elf):
    out = subprocess.run([find_tool("arm-none-eabi-nm"), elf], capture_output=True, text=True, check=True).stdout
    table = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3:
            table.setdefault(parts[2], int(parts[0], 16))
    missing = [s for s in ("ibus", "control", "actuator_debug", "uwTick") if s not in table]
    if missing:
        sys.exit(f"symbols not in {elf}: {missing} (HW_TEST_ENABLED must be 0)")
    return table


class OpenOcd:
    def __init__(self):
        self.proc = subprocess.Popen(
            [find_tool("openocd"), "-f", "interface/stlink.cfg", "-f", "target/stm32f4x.cfg",
             "-c", "tcl_port 6666", "-c", "gdb_port disabled", "-c", "telnet_port disabled", "-c", "init"],
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        for _ in range(50):
            try:
                self.sock = socket.create_connection(("127.0.0.1", 6666), timeout=2)
                return
            except OSError:
                if self.proc.poll() is not None:
                    sys.exit("openocd failed:\n" + self.proc.stderr.read().decode(errors="replace"))
                time.sleep(0.2)
        sys.exit("openocd did not open its Tcl port")

    def cmd(self, text):
        self.sock.sendall(text.encode() + b"\x1a")
        data = b""
        while not data.endswith(b"\x1a"):
            data += self.sock.recv(4096)
        return data[:-1].decode()

    def read(self, addr, size):
        words = self.cmd(f"read_memory 0x{addr:08x} 8 {size}").split()
        return bytes(int(w, 0) for w in words)

    def close(self):
        try:
            self.cmd("exit")
        except OSError:
            pass
        self.proc.terminate()


def snapshot(ocd, sym):
    ch = struct.unpack("<14H", ocd.read(sym["ibus"], 28))
    last_rx, valid = struct.unpack("<IB", ocd.read(sym["ibus"] + 28, 5))
    c = ocd.read(sym["control"], 16)
    armed, released, ready, overturned, _fault, estop = c[:6]
    last_imu = struct.unpack("<I", c[12:16])[0]
    d = ocd.read(sym["actuator_debug"], 30)
    cmd = struct.unpack("<4f", d[:16])
    esc = struct.unpack("<3H", d[16:22])
    servo = struct.unpack("<4H", d[22:30])
    now = struct.unpack("<I", ocd.read(sym["uwTick"], 4))[0]
    rc_age = now - last_rx if valid else None
    return {
        "rc": "OK" if valid and rc_age <= 250 else ("STALE" if valid else "NONE"),
        "ch": ch[:6],
        "armed": armed, "sw_released": released, "manual_ready": ready,
        "estop": estop, "overturned": overturned, "imu": "OK" if now - last_imu <= 100 else "STALE",
        "cmd": cmd, "esc": esc, "servo": servo,
    }


def fmt(s):
    ch = " ".join(f"{v:4d}" for v in s["ch"])
    cmd = "rearL %.2f rearR %.2f front %+.2f steer %+.2f" % s["cmd"]
    return (f"RC {s['rc']:5s} CH1-6 [{ch}] | ARM {s['armed']} (sw_off_seen {s['sw_released']}, ready {s['manual_ready']}) "
            f"E-stop {s['estop']} IMU {s['imu']} lat {s['overturned']} | {cmd} | "
            f"ESC F/R/L {s['esc'][0]}/{s['esc'][1]}/{s['esc'][2]} | rudder F/R/L {s['servo'][1]}/{s['servo'][2]}/{s['servo'][3]}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", default=os.path.join(ROOT, "build", "Debug", "STM32F4.elf"))
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--log", type=float, metavar="SECONDS", help="print changes only, for this long")
    ap.add_argument("--period", type=float, default=0.1)
    args = ap.parse_args()

    sym = symbols(args.elf)
    ocd = OpenOcd()
    try:
        if args.once:
            print(fmt(snapshot(ocd, sym)))
            return
        end = time.time() + args.log if args.log else None
        previous = None
        while end is None or time.time() < end:
            s = snapshot(ocd, sym)
            if args.log:
                # Channel jitter of a few us is not a change worth a line.
                key = (s["rc"], tuple(v // 25 for v in s["ch"]), s["armed"], s["estop"], s["imu"], s["esc"], s["servo"])
                if key != previous:
                    print(time.strftime("%H:%M:%S"), fmt(s), flush=True)
                previous = key
            else:
                print("\r" + fmt(s), end="", flush=True)
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        ocd.close()


if __name__ == "__main__":
    main()
