#!/usr/bin/env python3
"""Runs the fleet in the simulator under different plans and compares them (docs/fleet-control.md).

    run     one scenario: tools/fleet_sim.sh headless, then the plan sent to the Worker as an admin would
            (POST /api/fleet/plan), then --duration seconds of search; everything lands in <out>/<name>
    report  every scenario under <out>: the numbers and a map of the tracks, as report.html and report.json

    VEDC_INGEST_TOKEN=... VEDC_ADMIN_TOKEN=... python3 fleet_compare.py run --name sectors3 --mode sectors --boats 3
    python3 fleet_compare.py run --name formation3 --mode formation --boats 3 --shape line --spacing 4
    python3 fleet_compare.py run --name formation3_lost --mode formation --boats 3 --kill-leader-at 150
    python3 fleet_compare.py report

The search area and the people come from the world: the area is AREAS[world] (the street of the central-Vietnam
scene, sim/worlds/vedc_central.sdf, x -2..76 m, y -5.5..5.5 m; or the open water of vedc_wide.sdf, x 0..60 m,
y -18..18 m), the people every <include> named victim_*.
What is compared, from each boat's track.csv and stm32_events.log and the station's air_station.json:
  - people found: a VICTIM_FOUND report within MATCH_M of a person counts for that person; the time from the
    plan to the first report of each; reports of a person already reported (by any boat) are repeats
  - area seen: the share of the area within SEEN_M of a track (the camera sweeps both sides)
  - distance run, energy (the rear throttle summed over time, in throttle-seconds)
  - close passes: seconds two boats were within CLOSE_M of each other
  - the LoRa channel: frames the station heard, lost to overlaps, lost while it was sending
  - formation: how far each follower was from its place (median), after the first minute
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
HOME = (10.883438, 106.796019)       # the world origin (run_boat.sh HOME_LOCATION)
AREAS = {
    "central": [(-2.0, -5.5), (76.0, -5.5), (76.0, 5.5), (-2.0, 5.5)],   # kept off the house walls at y +-9
    "wide": [(0.0, -18.0), (60.0, -18.0), (60.0, 18.0), (0.0, 18.0)],     # open water, about 2.5 times the street
}
MATCH_M, SEEN_M, CLOSE_M = 6.0, 6.0, 3.0
EARTH = 6378137.0


def to_ll(x: float, y: float) -> tuple[float, float]:
    return (HOME[0] + math.degrees(y / EARTH), HOME[1] + math.degrees(x / (EARTH * math.cos(math.radians(HOME[0])))))


def to_xy(lat: float, lon: float) -> tuple[float, float]:
    return (math.radians(lon - HOME[1]) * EARTH * math.cos(math.radians(HOME[0])), math.radians(lat - HOME[0]) * EARTH)


def victims(world: Path) -> list[tuple[str, float, float]]:
    text = world.read_text()
    return [(m.group(1), float(m.group(2)), float(m.group(3)))
            for m in re.finditer(r"<name>(victim[\w]*)</name><pose>([-\d.]+) ([-\d.]+)", text)]


def api(path: str, body: dict) -> dict:
    base = os.environ.get("VEDC_CLOUD_URL", "http://127.0.0.1:8787").rstrip("/")
    req = urllib.request.Request(base + path, data=json.dumps(body).encode(), method="POST",
                                 headers={"Authorization": "Bearer " + os.environ["VEDC_ADMIN_TOKEN"], "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=15) as res:
        return json.loads(res.read() or b"{}")


# ---- run --------------------------------------------------------------------------------------------------

def run(args: argparse.Namespace) -> int:
    out = Path(args.out).expanduser() / args.name
    ids = [f"PHAO-0{n}" for n in range(1, args.boats + 1)]
    # A clean slate on the Worker: no formation, nobody stopped or called home, no dots from the last run
    api("/api/formation/end", {})
    api("/api/fleet", {"action": "resume", "ids": ids})
    api("/api/victims/clear", {})
    for i in ids:
        api("/api/mission/send", {"id": i, "clear": True})
    cmd = ["bash", str(REPO / "sim/tools/fleet_sim.sh"), f"--boats={args.boats}", f"--world={args.world}", "--headless",
           f"--run={out}", f"--duration={args.duration + args.warmup + 60}", f"--vision={args.vision}", f"--hold={args.hold}"]
    if args.no_tdma:
        cmd.append("--no-tdma")
    print(" ".join(cmd), flush=True)
    sim = subprocess.Popen(cmd, start_new_session=True)
    try:
        # Up once every boat's track is being written and the station has heard every boat
        deadline = time.time() + 300
        while time.time() < deadline:
            heard = {}
            try:
                heard = json.loads((out / "air_station.json").read_text()).get("frames", {})
            except (OSError, ValueError):
                pass
            if all(i in heard for i in ids) and all((out / f"boat_{n}" / "track.csv").exists() for n in range(1, args.boats + 1)):
                break
            if sim.poll() is not None:
                print("the fleet stopped before it was up", file=sys.stderr)
                return 1
            time.sleep(2)
        else:
            print("the fleet did not come up within 300 s", file=sys.stderr)
            return 1
        print(f"fleet up, warming up {args.warmup} s", flush=True)
        time.sleep(args.warmup)
        poly = [dict(zip(("lat", "lon"), to_ll(x, y))) for x, y in AREAS[args.world]]
        body = {"mode": "formation" if args.mode == "formation" else "sectors", "poly": poly, "ids": ids,
                "spacing": args.spacing, "shape": args.shape, "leader": ids[0]}
        if args.angle is not None:
            body["angle"] = args.angle
        plan = api("/api/fleet/plan", body)
        plan_at = time.time()
        print(f"plan sent: {plan.get('mode')}", flush=True)
        (out / "scenario.json").write_text(json.dumps({"name": args.name, "mode": args.mode, "boats": args.boats,
                                                       "tdma": not args.no_tdma, "plan_at": plan_at, "duration": args.duration,
                                                       "kill_leader_at": args.kill_leader_at, "plan": plan, "world": args.world,
                                                       "vision": args.vision}, indent=1))
        if args.kill_leader_at:
            time.sleep(args.kill_leader_at)
            # The leader's F407 dies: no more beacons, its boat stops
            subprocess.run(["pkill", "-f", f"stm32_board_sim.py .*--buoy-id {ids[0]}"])
            print(f"{ids[0]}'s board killed at {args.kill_leader_at} s", flush=True)
            time.sleep(max(0, args.duration - args.kill_leader_at))
        else:
            time.sleep(args.duration)
    finally:
        try:
            os.killpg(sim.pid, signal.SIGINT)
            sim.wait(timeout=30)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(sim.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    print(f"done: {out}", flush=True)
    return 0


# ---- report -----------------------------------------------------------------------------------------------

def read_track(path: Path) -> list[dict]:
    rows = []
    with path.open() as f:
        for r in csv.DictReader(f):
            try:
                rows.append({"t": float(r["t"]), "x": float(r["x"]), "y": float(r["y"]), "heading": float(r["heading"]),
                             "speed": float(r["speed"]), "throttle": float(r["throttle"]), "mission": r["mission"]})
            except (KeyError, ValueError):
                pass
    return rows


def read_reports(path: Path) -> list[tuple[float, float, float]]:
    """(time, x, y) of every VICTIM_FOUND the board passed on."""
    out = []
    for line in path.read_text(errors="replace").splitlines():
        m = re.match(r"([\d.]+) TXD VICTIM_FOUND,([-\d.]+),([-\d.]+)", line)
        if m:
            lat, lon = float(m.group(2)), float(m.group(3))
            if lat or lon:
                out.append((float(m.group(1)), *to_xy(lat, lon)))
    return out


def inside(x: float, y: float, poly: list[tuple[float, float]]) -> bool:
    c = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def analyse(d: Path) -> dict | None:
    try:
        sc = json.loads((d / "scenario.json").read_text())
    except (OSError, ValueError):
        return None
    t0, t1 = sc["plan_at"], sc["plan_at"] + sc["duration"]
    world = d / "worlds" / f"vedc_{sc['world']}.sdf"
    people = victims(world)
    tracks = {}
    for b in sorted(d.glob("boat_*")):
        n = int(b.name.split("_")[1])
        rows = [r for r in read_track(b / "track.csv") if t0 - 5 <= r["t"] <= t1] if (b / "track.csv").exists() else []
        tracks[f"PHAO-0{n}"] = {"rows": rows, "reports": [r for r in read_reports(b / "stm32_events.log") if t0 <= r[0] <= t1]
                                if (b / "stm32_events.log").exists() else []}
    # People found, the first report of each, repeats
    first: dict[str, tuple[float, str]] = {}
    repeats, reporters = 0, {}
    events = sorted((t, x, y, boat) for boat, tr in tracks.items() for t, x, y in tr["reports"])
    for t, x, y, boat in events:
        name, dist = min(((p[0], math.hypot(p[1] - x, p[2] - y)) for p in people), key=lambda v: v[1], default=(None, 1e9))
        if name is None or dist > MATCH_M:
            continue
        reporters.setdefault(name, set()).add(boat)
        if name in first:
            repeats += 1
        else:
            first[name] = (t - t0, boat)
    # Area seen
    area = AREAS[sc["world"]]
    xs, ys = [p[0] for p in area], [p[1] for p in area]
    cells = [(x + 0.5, y + 0.5) for x in range(int(min(xs)), int(max(xs))) for y in range(int(min(ys)), int(max(ys)))
             if inside(x + 0.5, y + 0.5, area)]
    pts = [(r["x"], r["y"]) for tr in tracks.values() for r in tr["rows"] if r["t"] >= t0]
    seen = sum(1 for cx, cy in cells if any((cx - px) ** 2 + (cy - py) ** 2 <= SEEN_M ** 2 for px, py in pts[::2]))
    # Distance, energy
    per_boat = {}
    for boat, tr in tracks.items():
        rows = [r for r in tr["rows"] if r["t"] >= t0]
        dist = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(rows, rows[1:]))
        energy = sum(a["throttle"] * (b["t"] - a["t"]) for a, b in zip(rows, rows[1:]))
        per_boat[boat] = {"distance_m": round(dist, 1), "energy": round(energy, 1), "reports": len(tr["reports"])}
    # Close passes: seconds two boats were within CLOSE_M
    close = 0
    names = list(tracks)
    by_sec = {boat: {int(r["t"]): (r["x"], r["y"]) for r in tracks[boat]["rows"] if r["t"] >= t0} for boat in names}
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = by_sec[names[i]], by_sec[names[j]]
            close += sum(1 for s in a.keys() & b.keys() if math.dist(a[s], b[s]) < CLOSE_M)
    # Formation: followers off their place, after the first minute
    formation = None
    plan = sc.get("plan") or {}
    if sc["mode"] == "formation" and plan.get("members"):
        errs = []
        leader = by_sec.get(plan["leader"], {})
        heads = {int(r["t"]): r["heading"] for r in tracks.get(plan["leader"], {"rows": []})["rows"]}
        for boat, place in plan["members"].items():
            for s, (fx, fy) in by_sec.get(boat, {}).items():
                if s - t0 < 60 or s not in leader or s not in heads:
                    continue
                h = math.radians(heads[s])
                lx, ly = leader[s]
                # right of a compass heading h: (cos h, -sin h); back: (-sin h, -cos h)
                px = lx + place["right"] * math.cos(h) - place["back"] * math.sin(h)
                py = ly - place["right"] * math.sin(h) - place["back"] * math.cos(h)
                errs.append(math.hypot(fx - px, fy - py))
        errs.sort()
        formation = {"median_m": round(errs[len(errs) // 2], 1) if errs else None, "samples": len(errs)}
    try:
        air = json.loads((d / "air_station.json").read_text())
    except (OSError, ValueError):
        air = {}
    found_times = sorted(t for t, _ in first.values())
    return {
        "name": sc["name"], "world": sc["world"], "area_xy": area, "mode": sc["mode"], "boats": sc["boats"], "tdma": sc["tdma"], "duration_s": sc["duration"],
        "kill_leader_at": sc.get("kill_leader_at"),
        "people": len(people), "found": len(first),
        "found_by": {k: {"after_s": round(v[0]), "boat": v[1], "boats": sorted(reporters.get(k, []))} for k, v in sorted(first.items())},
        "time_half_s": round(found_times[(len(people) + 1) // 2 - 1]) if len(found_times) >= (len(people) + 1) // 2 else None,
        "time_all_s": round(found_times[-1]) if len(first) == len(people) else None,
        "repeat_reports": repeats,
        "people_by_several": sum(1 for v in reporters.values() if len(v) > 1),
        "seen_pct": round(100.0 * seen / max(len(cells), 1), 1),
        "distance_m": round(sum(p["distance_m"] for p in per_boat.values()), 1),
        "energy": round(sum(p["energy"] for p in per_boat.values()), 1),
        "close_s": close,
        "per_boat": per_boat,
        "lora": {k: air.get(k) for k in ("received", "collided", "deaf")} | {"frames": air.get("frames")},
        "formation": formation,
        "tracks": {boat: [(round(r["x"], 1), round(r["y"], 1)) for r in tr["rows"][::3] if r["t"] >= t0] for boat, tr in tracks.items()},
        "reports_xy": {boat: [(round(x, 1), round(y, 1)) for _, x, y in tr["reports"]] for boat, tr in tracks.items()},
        "victims_xy": [(p[0], p[1], p[2]) for p in people],
    }


def report(args: argparse.Namespace) -> int:
    root = Path(args.out).expanduser()
    results = [r for d in sorted(root.iterdir()) if d.is_dir() and (r := analyse(d))]
    (root / "report.json").write_text(json.dumps(results, indent=1))
    for r in results:
        print(f"{r['name']:<18} {r['mode']:<9} boats {r['boats']}  found {r['found']}/{r['people']}  half {r['time_half_s']} s  "
              f"all {r['time_all_s']} s  seen {r['seen_pct']}%  run {r['distance_m']} m  energy {r['energy']}  "
              f"repeats {r['repeat_reports']}  close {r['close_s']} s  lora {r['lora']}  formation {r['formation']}")
    print(f"{root / 'report.json'}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--name", required=True)
    r.add_argument("--mode", choices=("sectors", "formation"), default="sectors")
    r.add_argument("--boats", type=int, default=3)
    r.add_argument("--world", default="central", choices=sorted(AREAS))
    r.add_argument("--spacing", type=float, default=8.0)
    r.add_argument("--angle", type=float, help="sweep direction, compass degrees (default: along the area)")
    r.add_argument("--shape", default="line", choices=("line", "v", "column"))
    r.add_argument("--duration", type=int, default=600)
    r.add_argument("--warmup", type=int, default=20)
    r.add_argument("--no-tdma", action="store_true")
    r.add_argument("--vision", default="all", choices=("all", "first"))
    r.add_argument("--hold", type=int, default=20, help="seconds with a person before going on (follow.max_hold_s), the same for every plan")
    r.add_argument("--kill-leader-at", type=int, default=0, help="seconds after the plan: PHAO-01's board stops")
    r.add_argument("--out", default="~/vedc_sim/compare")
    p = sub.add_parser("report")
    p.add_argument("--out", default="~/vedc_sim/compare")
    args = parser.parse_args()
    return run(args) if args.cmd == "run" else report(args)


if __name__ == "__main__":
    sys.exit(main())
