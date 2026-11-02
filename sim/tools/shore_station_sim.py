#!/usr/bin/env python3
"""The ESP32 shore station in the simulator (esp32-lora-station/main/main.c), for the STM32 boat.

Hears the beacons of the simulated STM32F407 (stm32_board_sim.py --radio-lib) over the simulated
LoRa link (lora_radio.py), posts each one to the Worker as the station does, and answers every
frame, as the station does, with the station's own route_downlink.c (docs/lora-route-downlink.md):
the route an admin sent with "Gửi tới phao" while the boat still lacks it, otherwise an ACK with the
event acknowledged, the admin's lights and the station's position (--station-pos), the boat's way
home. Event frames (ev=...: a person, overturned, the confirm button) are posted as the "event" group.

A file named station_off in --dir silences the station: no answers, no posts, as if switched off.
The lights, the STOP and the RETURN come from the Worker's "lights", "hold" and "ret" (GET
/api/downlink), or with --route-file from files in --dir: lights holding the mask (1 light 1, 2 light 2,
3 both), hold and ret (present: the boat is stopped, called home). They go down as one command, bit 2
the STOP, bit 3 the RETURN. The Worker's "home" gives each boat its own parking spot, sent in its ACKs
instead of --station-pos, and "fol" the boats sailing in formation (docs/fleet-control.md).

The routes come from the Worker (GET /api/downlink every 5 s, Bearer VEDC_INGEST_TOKEN), one per boat
("routes", main.c keeps DOWNLINK_SLOTS of them), or with --route-file from a YAML file instead
(waypoints: [[lat, lon], ...], loop, id): every change of the file is a new revision, so the cloud is
not needed to try the radio and the Pi side.

Every boat of the fleet shares the one channel (lora_radio.Air): with --collisions the station loses
frames that overlap on the air and those that come in while it sends. What it heard and lost goes to
air_station.json in --dir every 10 s, for sim/tools/fleet_compare.py.

    VEDC_INGEST_TOKEN=dev-token python3 shore_station_sim.py --radio-lib librouteradio.so --cloud http://127.0.0.1:8787
    python3 shore_station_sim.py --radio-lib librouteradio.so --route-file route.yaml
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
import urllib.request
from pathlib import Path

from helper_board_sim import post, station_json
from lora_radio import BEACON_PORT, Air, StationRoute, airtime_s

POLL_S = 5.0                 # DOWNLINK_POLL_MS
TURNAROUND_S = 0.05          # LORA_TURNAROUND_MS
STATION_REPORT_S = 10.0      # STATION_REPORT_MS
DOWNLINK_SLOTS = 4           # route_downlink.h: boats the station holds a route for at once
STATS_S = 10.0


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
    """The lights file's mask, with bit 2 when a file named hold is there, bit 3 for one named ret."""
    try:
        mask = int((directory / "lights").read_text(encoding="utf-8").strip() or "0") & 3
    except (OSError, ValueError):
        mask = 0
    return {"PHAO-01": mask | (4 if (directory / "hold").exists() else 0) | (8 if (directory / "ret").exists() else 0)}


def cloud_commands(data: dict) -> dict[str, int] | None:
    """{"lights": {id: mask}, "hold": {id: 0|1}, "ret": {id: 0|1}} of GET /api/downlink as one command per buoy."""
    lights, hold, ret = data.get("lights") or {}, data.get("hold") or {}, data.get("ret") or {}
    if not isinstance(lights, dict) or not isinstance(hold, dict) or not isinstance(ret, dict):
        return None
    return {str(k): (int(lights.get(k, 0)) & 3) | (4 if int(hold.get(k, 0)) else 0) | (8 if int(ret.get(k, 0)) else 0)
            for k in set(lights) | set(hold) | set(ret)}


def cloud_routes(data: dict) -> list[dict]:
    """The routes waiting for their boats: "routes", or the one at the top from an older Worker."""
    routes = data.get("routes")
    if isinstance(routes, list):
        return [r for r in routes if isinstance(r, dict) and int(r.get("rev", 0))][:DOWNLINK_SLOTS]
    return [data] if int(data.get("rev", 0)) else []


def cloud_places(data: dict) -> tuple[dict[str, tuple[float, float]], dict[str, tuple[str, int, int]]]:
    """Each boat's parking spot ("home") and place in a formation ("fol": leader, right_m, back_m)."""
    homes, roles = {}, {}
    for buoy, p in (data.get("home") or {}).items():
        try:
            homes[str(buoy)] = (float(p[0]), float(p[1]))
        except (TypeError, ValueError, IndexError):
            pass
    for buoy, r in (data.get("fol") or {}).items():
        try:
            roles[str(buoy)] = (str(r[0]), int(round(float(r[1]))), int(round(float(r[2]))))
        except (TypeError, ValueError, IndexError):
            pass
    return homes, roles


def diag_json(raw: str) -> dict | None:
    """lora_payload.c lora_diag_parse, as the JSON main.c xu_ly_chan_doan posts; None when it is not one."""
    if not raw.startswith("dg="):
        return None
    head, *fields = raw[3:].split(",")
    try:
        f = dict(field.split("=", 1) for field in fields)
        wp, wp_total = (int(v) for v in f["w"].split("/"))
        i2c, imu = (int(v) for v in f["k"].split(":"))
        out = {"id": head, "reason": "" if f["n"] == "-" else f["n"], "wp": wp, "wp_total": wp_total,
               "arm": int(f["a"]), "drive": int(f["d"]), "i2c": i2c, "imu": imu}
        if f["s"] != "-":
            out["mission"] = f["s"]
        if "u" in f and "j" in f:
            out["volts"], out["amps"] = int(f["u"]) / 100, int(f["j"]) / 100
        if "f" in f:
            out["fps"] = int(f["f"]) / 10
    except (KeyError, ValueError):
        return None
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--radio-lib", required=True, type=Path, help="librouteradio.so (tools/hw_standins.sh)")
    parser.add_argument("--cloud", help="Worker base URL; beacons are posted to /api/ingest, the route is read from /api/downlink")
    parser.add_argument("--route-file", type=Path, help="YAML route instead of the Worker's (waypoints, loop, id)")
    parser.add_argument("--dir", type=Path, default=Path("."), help="where station_off and lights are looked for")
    parser.add_argument("--station-pos", help="lat,lon of the station's GPS, sent in every ACK; none: no fix")
    parser.add_argument("--collisions", action="store_true", help="lose frames that overlap on the shared channel (a fleet)")
    args = parser.parse_args()
    station_pos = tuple(float(v) for v in args.station_pos.split(",")[:2]) if args.station_pos else None
    token = os.environ.get("VEDC_INGEST_TOKEN", "")
    if args.cloud and not token:
        parser.error("--cloud needs VEDC_INGEST_TOKEN (the Worker's INGEST_TOKEN)")

    routes: dict[str, StationRoute] = {}      # boat -> the route the admin sent it, while it still lacks it
    lock = threading.Lock()
    lights: dict[str, int] = {}
    homes: dict[str, tuple[float, float]] = {}
    roles: dict[str, tuple[str, int, int]] = {}

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
            if data is not None and not args.route_file:
                new_homes, new_roles = cloud_places(data)
                with lock:
                    for buoy, role in new_roles.items():
                        if roles.get(buoy) != role:
                            log(f"formation: {buoy} keeps {role[1]} m right, {role[2]} m back of {role[0]}")
                    homes.clear(), homes.update(new_homes)
                    roles.clear(), roles.update(new_roles)
            if data is not None and (not args.route_file or int(data.get("rev", 0))):
                kept: dict[str, StationRoute] = {}
                for r in cloud_routes(data):
                    buoy, rev = str(r.get("id", "PHAO-01")), int(r["rev"])
                    with lock:
                        old = routes.get(buoy)
                    if old is not None and old.rev == rev:
                        kept[buoy] = old
                        continue
                    points = [(float(p[0]), float(p[1])) for p in r.get("pts", [])]
                    route = StationRoute(args.radio_lib)
                    if route.set(rev, buoy, bool(r.get("loop")), points):
                        kept[buoy] = route
                        log(f"route #{rev} to send: {len(points)} points for {buoy}{', loop' if r.get('loop') else ''}")
                with lock:
                    if args.route_file:
                        routes.update(kept)
                    else:
                        routes.clear(), routes.update(kept)
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
    air = Air(BEACON_PORT, collisions=args.collisions)
    builder = StationRoute(args.radio_lib)     # the event parser and the ACK builder of main.c
    heard: dict[str, int] = {}                 # frames heard from each boat, for air_station.json

    def write_stats() -> None:
        while True:
            time.sleep(STATS_S)
            try:
                (args.dir / "air_station.json").write_text(json.dumps({**air.stats(), "frames": heard, "t": time.time()}))
            except OSError:
                pass

    def answer(frame: bytes) -> None:
        time.sleep(TURNAROUND_S + airtime_s(len(frame)))   # arrives when it is off the air
        air.send(frame)

    def ack(buoy: str, eseq: int) -> bytes:
        """main.c tra_loi_phao: the command, the boat's own parking spot (else the station), its formation."""
        with lock:
            return builder.ack(buoy, eseq, lights.get(buoy, 0), homes.get(buoy, station_pos), roles.get(buoy))

    def serve(data: bytes) -> None:
        if (args.dir / "station_off").exists():
            return                            # switched off: hears nothing, answers nothing
        raw = data.decode("ascii", "replace")
        if raw.startswith(("RTE,", "ACK,")):
            return                            # nothing but the station sends these
        event = builder.event_json(data)
        if event is not None:
            buoy, group = event
            heard[buoy] = heard.get(buoy, 0) + 1
            if args.cloud:
                body = {"id": buoy, "rssi": -72, "snr": 9.5, "raw": raw[:99], "event": group}
                threading.Thread(target=post, args=(args.cloud.rstrip("/") + "/api/ingest", token, body), daemon=True).start()
            frame = ack(buoy, group["seq"])
            answer(frame)
            log(f"event {group}: sent {frame.decode()}")
            return
        diag = diag_json(raw)
        if diag is not None:
            # main.c xu_ly_chan_doan: posted beside the buoy's beacon, never answered
            if args.cloud:
                body = {"id": diag.pop("id"), "rssi": -72, "snr": 9.5, "diag": diag}
                threading.Thread(target=post, args=(args.cloud.rstrip("/") + "/api/ingest", token, body), daemon=True).start()
            log(f"diagnostics {raw}")
            return
        body = station_json(raw)
        if body is None:
            return
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
            return
        heard[buoy] = heard.get(buoy, 0) + 1
        with lock:
            want = lights.get(buoy, 0)
            route = routes.get(buoy)
            # RTE while the boat lacks the route and its lights and STOP are right; otherwise an ACK.
            right = body.get("lights", want & 3) == want & 3 and body.get("hold", (want >> 2) & 1) == (want >> 2) & 1
            frame = route.frame(buoy, w) if right and route is not None else b""
        if not frame:
            frame = ack(buoy, 0)
        answer(frame)
        log(f"beacon {buoy} w={w} L={body.get('lights', '-')}: sent {frame.decode()}")

    threading.Thread(target=write_stats, daemon=True).start()
    log(f"shore station listening on udp {BEACON_PORT}, route from {args.route_file or args.cloud or 'nowhere'}"
        f"{', frames that overlap on the air are lost' if args.collisions else ''}")
    try:
        while True:
            for data in air.receive(timeout=None):
                serve(data)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
