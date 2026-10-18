"""Write the STM32 boat's search route (config/mission_stm32.yaml, read by main.py). Run from Rasp_Pi:

    python3 tools/set_waypoints.py show
    python3 tools/set_waypoints.py here            # append where the boat is now as a waypoint
    python3 tools/set_waypoints.py fence           # append where the boat is now as a fence corner
    python3 tools/set_waypoints.py add LAT LON     # append a waypoint by hand
    python3 tools/set_waypoints.py clear           # no waypoints, no fence
    python3 tools/set_waypoints.py loop on|off

"here" and "fence" read the STM32F407's $GPS packets on serial.stm32_port: stop main.py first,
and wait for a 3D fix. They average 5 s of fixes, the GPS wanders by a few metres in a pool.
--file picks another route file.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from time import monotonic

import serial
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utils.config import load_config            # noqa: E402
from utils.nmea_packet import decode            # noqa: E402


def read_route(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) if path.exists() else None
    data = data or {}
    return {"loop": data.get("loop", True), "waypoints": data.get("waypoints") or [], "geofence": data.get("geofence") or []}


def write_route(path: Path, route: dict) -> None:
    lines = ["# Search route of the STM32 boat (main.py, modules/waypoints.py); tools/set_waypoints.py writes it.",
             f"loop: {'true' if route['loop'] else 'false'}", "waypoints:"]
    lines += [f"  - [{lat:.7f}, {lon:.7f}]" for lat, lon in route["waypoints"]] or ["  []"]
    lines += ["# Corners of the pool, in order; fewer than 3: no fence. Outside it the boat stops.", "geofence:"]
    lines += [f"  - [{lat:.7f}, {lon:.7f}]" for lat, lon in route["geofence"]] or ["  []"]
    text = "\n".join(lines).replace("waypoints:\n  []", "waypoints: []").replace("geofence:\n  []", "geofence: []")
    path.write_text(text + "\n", encoding="utf-8")


def gps_here(port: str, baud: int, seconds: float = 5.0) -> tuple[float, float]:
    """Mean of the 3D fixes the STM32 reports over `seconds`."""
    fixes = []
    end = monotonic() + 15.0                   # how long to wait for a first fix
    with serial.Serial(port, baud, timeout=0.2) as link:
        while monotonic() < end:
            packet = decode(link.readline())
            if packet and packet.command == "GPS" and len(packet.fields) >= 4 and int(packet.fields[3]) >= 3:
                if not fixes:
                    end = monotonic() + seconds    # the first fix: average from here
                fixes.append((float(packet.fields[1]), float(packet.fields[2])))
    if not fixes:
        raise SystemExit(f"no 3D fix from the STM32 on {port}: is main.py stopped, is the GPS outdoors?")
    lat, lon = sum(f[0] for f in fixes) / len(fixes), sum(f[1] for f in fixes) / len(fixes)
    print(f"{len(fixes)} fixes, mean {lat:.7f} {lon:.7f}")
    return lat, lon


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=["show", "here", "fence", "add", "clear", "loop"])
    parser.add_argument("values", nargs="*")
    parser.add_argument("--file", type=Path, default=Path("config/mission_stm32.yaml"))
    args = parser.parse_args()
    route = read_route(args.file)
    if args.action in ("here", "fence"):
        serial_config = load_config(Path("config/settings.yaml"), [Path("config/boat_stm32.yaml")])["serial"]
        point = gps_here(serial_config["stm32_port"], int(serial_config["stm32_baud"]))
        route["waypoints" if args.action == "here" else "geofence"].append(list(point))
    elif args.action == "add":
        if len(args.values) != 2:
            parser.error("add LAT LON")
        route["waypoints"].append([float(args.values[0]), float(args.values[1])])
    elif args.action == "clear":
        route["waypoints"], route["geofence"] = [], []
    elif args.action == "loop":
        if args.values not in (["on"], ["off"]):
            parser.error("loop on|off")
        route["loop"] = args.values == ["on"]
    if args.action != "show":
        write_route(args.file, route)
    print(f"{args.file}: {len(route['waypoints'])} waypoints, loop {'on' if route['loop'] else 'off'}, "
          f"fence {len(route['geofence'])} corners")
    for i, (lat, lon) in enumerate(route["waypoints"], 1):
        print(f"  wp {i}: {lat:.7f} {lon:.7f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
