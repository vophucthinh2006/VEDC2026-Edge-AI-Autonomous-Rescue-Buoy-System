#!/usr/bin/env python3
"""The ESP32 shore station in the simulator (esp32-lora-station/main/main.c), for the STM32 boat.

Hears the beacons of the simulated STM32F407 (stm32_board_sim.py --radio-lib) over the simulated
LoRa link (lora_radio.py), posts each one to the Worker as the station does, and answers every
frame, as the station does, with the station's own route_downlink.c (docs/lora-route-downlink.md):
the route an admin sent with "Gửi tới phao" while the boat still lacks it, otherwise an ACK with the
event acknowledged, the admin's lights and the station's position (--station-pos), the boat's way
home. Event frames (ev=...: a person, overturned, the confirm button) are posted as the "event" group.

A file named station_off in --dir silences the station: no answers, no posts, as if switched off.
The lights and the STOP come from the Worker's "lights" and "hold" (GET /api/downlink), or with
--route-file from files in --dir: lights holding the mask (1 light 1, 2 light 2, 3 both), and hold
(present: the boat is stopped). They go down as one command, bit 2 the STOP.

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
STATION_REPORT_S = 10.0      # STATION_REPORT_MS


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


def read_commands(directory: Path) -> dict[str, int]:
    """The lights file's mask, with bit 2 when a file named hold is there."""
    try:
        mask = int((directory / "lights").read_text(encoding="utf-8").strip() or "0") & 3
    except (OSError, ValueError):
        mask = 0
    return {"PHAO-01": mask | (4 if (directory / "hold").exists() else 0)}


def cloud_commands(data: dict) -> dict[str, int] | None:
    """{"lights": {id: mask}, "hold": {id: 0|1}} of GET /api/downlink as one command per buoy."""
    lights, hold = data.get("lights") or {}, data.get("hold") or {}
    if not isinstance(lights, dict) or not isinstance(hold, dict):
        return None
    return {str(k): (int(lights.get(k, 0)) & 3) | (4 if int(hold.get(k, 0)) else 0) for k in set(lights) | set(hold)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--radio-lib", required=True, type=Path, help="librouteradio.so (tools/hw_standins.sh)")
    parser.add_argument("--cloud", help="Worker base URL; beacons are posted to /api/ingest, the route is read from /api/downlink")
    parser.add_argument("--route-file", type=Path, help="YAML route instead of the Worker's (waypoints, loop, id)")
    parser.add_argument("--dir", type=Path, default=Path("."), help="where station_off and lights are looked for")
    parser.add_argument("--station-pos", help="lat,lon of the station's GPS, sent in every ACK; none: no fix")
    args = parser.parse_args()
    station_pos = tuple(float(v) for v in args.station_pos.split(",")[:2]) if args.station_pos else None
    token = os.environ.get("VEDC_INGEST_TOKEN", "")
    if args.cloud and not token:
        parser.error("--cloud needs VEDC_INGEST_TOKEN (the Worker's INGEST_TOKEN)")

    route = StationRoute(args.radio_lib)
    lock = threading.Lock()
    lights: dict[str, int] = {}

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
            new_lights = read_commands(args.dir) if args.route_file else (cloud_commands(data) if data else None)
            if isinstance(new_lights, dict):
                with lock:
                    for buoy, mask in new_lights.items():
                        if lights.get(buoy, 0) != mask:
                            log(f"command for {buoy}: lights {mask & 3}{', STOP' if mask & 4 else ''}")
                    lights.clear()
                    lights.update(new_lights)
            if data is not None and int(data.get("rev", 0)) != route.rev:
                points = [(float(p[0]), float(p[1])) for p in data.get("pts", [])]
                with lock:
                    ok = route.set(int(data["rev"]), str(data.get("id", "PHAO-01")), bool(data.get("loop")), points) if data["rev"] else False
                if ok:
                    log(f"route #{data['rev']} to send: {len(points)} points for {data.get('id')}{', loop' if data.get('loop') else ''}")
            time.sleep(POLL_S if not args.route_file else 1.0)

    threading.Thread(target=poll, daemon=True).start()

    def report_position() -> None:
        # main.c gui_vi_tri_tram: the station's own GPS to /api/station every STATION_REPORT_MS, for the
        # dashboard's station marker and its LoRa coverage circles.
        body = {"fix": 1, "lat": station_pos[0], "lon": station_pos[1], "sats": 12, "hdop": 0.8}
        while True:
            post(args.cloud.rstrip("/") + "/api/station", token, body)
            time.sleep(STATION_REPORT_S)

    if args.cloud and station_pos:
        threading.Thread(target=report_position, daemon=True).start()
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", BEACON_PORT))
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    log(f"shore station listening on udp {BEACON_PORT}, route from {args.route_file or args.cloud or 'nowhere'}")
    try:
        while True:
            data = rx.recv(256)
            if (args.dir / "station_off").exists():
                continue                      # switched off: hears nothing, answers nothing
            raw = data.decode("ascii", "replace")
            event = route.event_json(data)
            if event is not None:
                buoy, group = event
                if args.cloud:
                    body = {"id": buoy, "rssi": -72, "snr": 9.5, "raw": raw[:99], "event": group}
                    threading.Thread(target=post, args=(args.cloud.rstrip("/") + "/api/ingest", token, body), daemon=True).start()
                with lock:
                    frame = route.ack(buoy, group["seq"], lights.get(buoy, 0), station_pos)
                time.sleep(TURNAROUND_S + airtime_s(len(frame)))
                tx.sendto(frame, ("127.0.0.1", DOWNLINK_PORT))
                log(f"event {group}: sent {frame.decode()}")
                continue
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
            buoy = body.get("id", "")
            if not buoy:
                continue
            with lock:
                want = lights.get(buoy, 0)
                # RTE while the boat lacks the route and its lights and STOP are right; otherwise an ACK.
                right = body.get("lights", want & 3) == want & 3 and body.get("hold", (want >> 2) & 1) == (want >> 2) & 1
                frame = route.frame(buoy, w) if right else b""
                if not frame:
                    frame = route.ack(buoy, 0, want, station_pos)
            time.sleep(TURNAROUND_S + airtime_s(len(frame)))   # arrives when it is off the air
            tx.sendto(frame, ("127.0.0.1", DOWNLINK_PORT))
            log(f"beacon w={w} L={body.get('lights', '-')}: sent {frame.decode()}")
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
