#!/usr/bin/env python3
"""The ESP32 shore station in the simulator (esp32-lora-station/main/main.c), for the STM32 boat.

Hears the beacons of the simulated STM32F407 (stm32_board_sim.py --radio-lib) over the simulated
LoRa link (lora_radio.py), posts each one to the Worker as the station does, and answers it with
the route an admin sent with "Gửi tới phao" (docs/lora-route-downlink.md), built by the station's
own route_downlink.c.

The route comes from the Worker (GET /api/downlink every 5 s, Bearer VEDC_INGEST_TOKEN), or with
--route-file from a YAML file instead (waypoints: [[lat, lon], ...], loop, id): every change of
the file is a new revision, so the cloud is not needed to try the radio and the Pi side.

    VEDC_INGEST_TOKEN=dev-token python3 shore_station_sim.py --radio-lib librouteradio.so --cloud http://127.0.0.1:8787
    python3 shore_station_sim.py --radio-lib librouteradio.so --route-file route.yaml
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import threading
import time
import urllib.request
from pathlib import Path

from helper_board_sim import post, station_json
from lora_radio import BEACON_PORT, DOWNLINK_PORT, StationRoute, airtime_s

POLL_S = 5.0                 # DOWNLINK_POLL_MS
TURNAROUND_S = 0.05          # LORA_TURNAROUND_MS


def log(text: str) -> None:
    print(f"{time.strftime('%H:%M:%S')} {text}", flush=True)


def fetch_cloud(url: str, token: str) -> dict | None:
    request = urllib.request.Request(url + "/api/downlink", headers={"Authorization": "Bearer " + token})
    try:
        with urllib.request.urlopen(request, timeout=8) as response:
            return json.loads(response.read())
    except (OSError, ValueError) as exc:
        log(f"GET /api/downlink failed: {exc}")
        return None


def read_file(path: Path, rev: int) -> dict | None:
    import yaml
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        log(f"route file: {exc}")
        return None
    return {"rev": rev, "id": data.get("id", "PHAO-01"), "loop": bool(data.get("loop", False)),
            "pts": [[float(lat), float(lon)] for lat, lon in data.get("waypoints") or []]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--radio-lib", required=True, type=Path, help="librouteradio.so (tools/hw_standins.sh)")
    parser.add_argument("--cloud", help="Worker base URL; beacons are posted to /api/ingest, the route is read from /api/downlink")
    parser.add_argument("--route-file", type=Path, help="YAML route instead of the Worker's (waypoints, loop, id)")
    args = parser.parse_args()
    token = os.environ.get("VEDC_INGEST_TOKEN", "")
    if args.cloud and not token:
        parser.error("--cloud needs VEDC_INGEST_TOKEN (the Worker's INGEST_TOKEN)")

    route = StationRoute(args.radio_lib)
    lock = threading.Lock()

    def poll() -> None:
        file_rev, file_mtime = 0, None
        while True:
            data = None
            if args.route_file:
                try:
                    mtime = args.route_file.stat().st_mtime
                except OSError:
                    mtime = None
                if mtime is not None and mtime != file_mtime:
                    file_mtime, file_rev = mtime, file_rev % 65535 + 1
                    data = read_file(args.route_file, file_rev)
            elif args.cloud:
                data = fetch_cloud(args.cloud.rstrip("/"), token)
            if data is not None and int(data.get("rev", 0)) != route.rev:
                points = [(float(p[0]), float(p[1])) for p in data.get("pts", [])]
                with lock:
                    ok = route.set(int(data["rev"]), str(data.get("id", "PHAO-01")), bool(data.get("loop")), points) if data["rev"] else False
                if ok:
                    log(f"route #{data['rev']} to send: {len(points)} points for {data.get('id')}{', loop' if data.get('loop') else ''}")
            time.sleep(POLL_S if not args.route_file else 1.0)

    threading.Thread(target=poll, daemon=True).start()
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", BEACON_PORT))
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log(f"shore station listening on udp {BEACON_PORT}, route from {args.route_file or args.cloud or 'nowhere'}")
    try:
        while True:
            raw = rx.recv(256).decode("ascii", "replace")
            body = station_json(raw)
            if body is None:
                continue
            w = next((token[2:] for token in raw.split(",") if token.startswith("w=")), None)
            if w is not None:
                try:
                    rev, have, pi = (int(value) for value in w.split(":"))
                    body["route"] = {"rev": rev, "have": have, "pi": pi}
                except ValueError:
                    pass
            if args.cloud:
                threading.Thread(target=post, args=(args.cloud.rstrip("/") + "/api/ingest", token, body), daemon=True).start()
            with lock:
                frame = route.frame(body.get("id", ""), w)
            if frame:
                time.sleep(TURNAROUND_S + airtime_s(len(frame)))   # arrives when it is off the air
                tx.sendto(frame, ("127.0.0.1", DOWNLINK_PORT))
                log(f"beacon w={w}: sent {frame.decode()}")
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
