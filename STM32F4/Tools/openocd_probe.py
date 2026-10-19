"""Reading the running firmware's RAM over ST-LINK, through OpenOCD's Tcl port (no halt).

Shared by the live monitors (lora_monitor.py). Symbol addresses come from the ELF, so the
monitors follow the build they are given.
"""
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
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


class OpenOcd:
    def __init__(self):
        # OpenOCD keeps logging while it runs; a pipe nobody reads fills up
        # and stalls it, so its output goes to a file instead.
        self.log = tempfile.TemporaryFile()
        self.proc = subprocess.Popen(
            [find_tool("openocd"), "-f", "interface/stlink.cfg", "-f", "target/stm32f4x.cfg",
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
        # Whole 32-bit words, so a counter the MCU bumps mid-read (uwTick)
        # is never stitched from bytes of two different values.
        # addr must be word-aligned; every symbol read here is.
        count = (size + 3) // 4
        words = self.cmd(f"read_memory 0x{addr:08x} 32 {count}").split()
        data = b"".join(int(w, 0).to_bytes(4, "little") for w in words)
        return data[:size]

    def close(self):
        try:
            self.cmd("exit")
        except OSError:
            pass
        self.proc.terminate()
        self.proc.wait()
