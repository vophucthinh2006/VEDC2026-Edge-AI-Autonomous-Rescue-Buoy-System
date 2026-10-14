"""Live view of the helper board over ST-LINK.

Reads helper_debug (App/app.h) from RAM while the firmware runs (no halt), through OpenOCD's Tcl
port. The address comes from the ELF, which must be the one flashed.

    python Tools/helper_monitor.py                # refresh in place, Ctrl+C to stop
    python Tools/helper_monitor.py --once         # one snapshot
    python Tools/helper_monitor.py --csv run.csv  # also log every sample
    python Tools/helper_monitor.py --layout       # check the layout against the ELF, no probe needed

FORMAT below must match helper_debug_t; update them together.
"""
import argparse
import os
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# helper_debug_t: lora_tx, lora_fail, lora_seq, current_ma; bus_mv, servo_us, pi_rx_ok, pi_rx_bad,
# ibus_polls; pi_ok, victims, alarm, estop, ina_ok, lora_version, lora_state
FORMAT = "<3Ii5H7B3x"            # 33 bytes of fields, padded to the struct's 4-byte alignment
FIELDS = ("lora_tx", "lora_fail", "lora_seq", "current_ma", "bus_mv", "servo_us", "pi_rx_ok", "pi_rx_bad", "ibus_polls",
          "pi_ok", "victims", "alarm", "estop", "ina_ok", "lora_version", "lora_state")
SIZE = struct.calcsize(FORMAT)
LORA_STATES = {0: "off", 1: "INIT FAILED", 2: "idle", 3: "sending"}


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


def symbol(elf, name):
    """Address and size of a symbol in the ELF."""
    out = subprocess.run([find_tool("arm-none-eabi-nm"), "-S", elf], capture_output=True, text=True, check=True).stdout
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[3] == name:
            return int(parts[0], 16), int(parts[1], 16)
    sys.exit(f"{name} is not in {elf}: build first")


class OpenOcd:
    def __init__(self):
        # OpenOCD keeps logging while it runs; a pipe nobody reads fills up and stalls it.
        self.log = tempfile.TemporaryFile()
        self.proc = subprocess.Popen(
            [find_tool("openocd"), "-f", "interface/stlink.cfg", "-f", "target/stm32f1x.cfg",
             "-c", "tcl_port 6666", "-c", "gdb_port disabled", "-c", "telnet_port disabled", "-c", "init"],
            stdout=self.log, stderr=subprocess.STDOUT)
        for _ in range(50):
            try:
                self.sock = socket.create_connection(("127.0.0.1", 6666), timeout=5)
                return
            except OSError:
                if self.proc.poll() is not None:
                    self.log.seek(0)
                    raise ConnectionError("openocd failed:\n" + self.log.read().decode(errors="replace"))
                time.sleep(0.2)
        raise ConnectionError("openocd did not open its Tcl port")

    def cmd(self, text):
        self.sock.sendall(text.encode() + b"\x1a")
        data = b""
        while not data.endswith(b"\x1a"):
            data += self.sock.recv(4096)
        return data[:-1].decode()

    def read(self, addr, size):
        # Whole 32-bit words, so a counter the MCU bumps mid-read is never stitched from two values.
        words = self.cmd(f"read_memory 0x{addr:08x} 32 {(size + 3) // 4}").split()
        return b"".join(int(w, 0).to_bytes(4, "little") for w in words)[:size]

    def close(self):
        try:
            self.cmd("exit")
        except OSError:
            pass
        self.proc.terminate()
        self.proc.wait()


def fmt(s):
    pi = f"Pi {'OK' if s['pi_ok'] else 'SILENT'} ({s['pi_rx_ok']} lines, {s['pi_rx_bad']} bad)"
    lora = f"LoRa chip 0x{s['lora_version']:02X} {LORA_STATES.get(s['lora_state'], '?')}, {s['lora_tx']} sent / {s['lora_fail']} failed"
    battery = f"battery {s['bus_mv'] / 1000:.2f} V {s['current_ma'] / 1000:+.2f} A" if s["ina_ok"] else "INA226 not found"
    return (f"{pi} | servo {s['servo_us']} us | people {s['victims']}{' ALARM' if s['alarm'] else ''}"
            f"{' E-STOP' if s['estop'] else ''} | {battery} | {lora} | handset polls {s['ibus_polls']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--elf", default=os.path.join(ROOT, "build", "Debug", "STM32F103.elf"))
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--period", type=float, default=0.5)
    ap.add_argument("--csv", help="append every sample to this file")
    ap.add_argument("--layout", action="store_true", help="only check FORMAT against the ELF")
    args = ap.parse_args()

    address, size = symbol(args.elf, "helper_debug")
    if size != SIZE:
        sys.exit(f"helper_debug is {size} bytes in the ELF but FORMAT describes {SIZE}: update FORMAT with helper_debug_t")
    if args.layout:
        print(f"helper_debug at 0x{address:08x}, {size} bytes: layout matches")
        return
    try:
        ocd = OpenOcd()
    except ConnectionError as e:
        sys.exit(str(e))
    log = None
    if args.csv:
        new = not os.path.exists(args.csv)
        log = open(args.csv, "a", buffering=1)
        if new:
            log.write("time," + ",".join(FIELDS) + "\n")
    try:
        while True:
            s = dict(zip(FIELDS, struct.unpack(FORMAT, ocd.read(address, SIZE))))
            if log:
                log.write(f"{time.time():.2f}," + ",".join(str(s[name]) for name in FIELDS) + "\n")
            if args.once:
                print(fmt(s))
                return
            print("\r" + fmt(s) + "   ", end="", flush=True)
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        ocd.close()
        if log:
            log.close()


if __name__ == "__main__":
    main()
