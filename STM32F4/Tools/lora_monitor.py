"""Live view of the LoRa beacon and the GPS state over ST-LINK.

Reads RAM while the firmware runs (no halt), through OpenOCD's Tcl port,
reusing the probe plumbing of rc_monitor.py. Symbol addresses come from the ELF.

    python Tools/lora_monitor.py              # refresh in place, Ctrl+C to stop
    python Tools/lora_monitor.py --once       # one snapshot

Struct layouts below must match lora_debug_t (App/lora_beacon.h) and
gps_state_t (Modules/GPS/gps_nmea.h); update them together.
"""
import argparse
import os
import struct
import subprocess
import sys
import time

from rc_monitor import OpenOcd, ROOT, find_tool

STATES = {0: "DISABLED", 1: "INIT_FAIL", 2: "IDLE", 3: "TX"}
ERRORS = {0: "ok", 1: "SPI error", 2: "chip not found (REG_VERSION != 0x12)", 3: "bad argument"}


def symbols(elf):
    out = subprocess.run([find_tool("arm-none-eabi-nm"), elf], capture_output=True, text=True, check=True).stdout
    table = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3:
            table.setdefault(parts[2], int(parts[0], 16))
    missing = [s for s in ("lora_debug", "gps", "uwTick") if s not in table]
    if missing:
        sys.exit(f"symbols not in {elf}: {missing} (build with LORA_ENABLED and flash first)")
    return table


def snapshot(ocd, sym):
    lora = ocd.read(sym["lora_debug"], 24)
    now = struct.unpack("<I", ocd.read(sym["uwTick"], 4))[0]
    g = ocd.read(sym["gps"], 32)
    version, init_ok, state, err = lora[:4]
    tx_count, tx_fail, seq, last_tx = struct.unpack("<4I", lora[4:20])
    last_len, gps_valid = struct.unpack("<2H", lora[20:24])
    lat, lon, hdop = struct.unpack("<3f", g[:12])
    fix = g[20]
    last_rx = struct.unpack("<I", g[24:28])[0]
    return {
        "version": version, "init_ok": init_ok, "state": STATES.get(state, str(state)), "err": ERRORS.get(err, str(err)),
        "tx": tx_count, "fail": tx_fail, "seq": seq,
        "last_tx_ago": None if tx_count == 0 else ((now - last_tx) & 0xFFFFFFFF) / 1000.0,
        "len": last_len, "frame_fix": gps_valid,
        "fix": fix, "lat": lat, "lon": lon, "hdop": hdop,
        "gps_age": None if last_rx == 0 else ((now - last_rx) & 0xFFFFFFFF) / 1000.0,
        "uptime_ms": now,
    }


def fmt(s):
    chip = f"chip 0x{s['version']:02X} {'OK' if s['init_ok'] else 'FAIL: ' + s['err']}"
    tx_ago = "-" if s["last_tx_ago"] is None else f"{s['last_tx_ago']:.1f} s ago"
    gps_age = "never" if s["gps_age"] is None else f"{s['gps_age']:.1f} s ago"
    return (f"LoRa {chip} | {s['state']:9s} | tx {s['tx']} ok / {s['fail']} fail | next seq {s['seq']} | "
            f"last tx {tx_ago} ({s['len']} B, {'with fix' if s['frame_fix'] else 'NO_FIX'}) | "
            f"GPS fix {s['fix']} {s['lat']:.6f},{s['lon']:.6f} hdop {s['hdop']:.1f} last sentence {gps_age}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", default=os.path.join(ROOT, "build", "Debug", "STM32F4.elf"))
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--period", type=float, default=0.5)
    args = ap.parse_args()

    sym = symbols(args.elf)
    try:
        ocd = OpenOcd()
    except ConnectionError as e:
        sys.exit(str(e))
    try:
        if args.once:
            print(fmt(snapshot(ocd, sym)))
            return
        while True:
            print("\r" + fmt(snapshot(ocd, sym)), end="", flush=True)
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        ocd.close()


if __name__ == "__main__":
    main()
