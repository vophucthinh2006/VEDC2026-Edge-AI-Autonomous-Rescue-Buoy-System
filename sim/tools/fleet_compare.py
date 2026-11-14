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
  - duplicate stops: seconds a boat followed (mission F) a person another boat had reported first, within
    STOP_M of them: what the victim claims (docs/victim-claim.md) save
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
# The same water: empty, with flood debris, with debris and people (sim/worlds/vedc_wide_*.sdf)
AREAS |= {f"wide_{k}": AREAS["wide"] for k in ("empty", "obstacles", "full")}
MATCH_M, SEEN_M, CLOSE_M = 6.0, 6.0, 3.0
STOP_M = 10.0   # a boat following within this of a person is stopping for them
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
    cmd = ["bash", str(REPO / "sim/tools/fleet_sim.sh"), f"--boats={args.boats}", f"--world={args.world}",
           f"--run={out}", f"--duration={args.duration + args.warmup + 60}", f"--vision={args.vision}", f"--hold={args.hold}"]
    if not args.gui:
        cmd.append("--headless")
    if args.offsets:
        cmd.append(f"--formation-offsets={args.offsets}")
    if args.no_tdma:
        cmd.append("--no-tdma")
    if args.fast_beacon:
        cmd.append("--fast-beacon")
    cmd.append(f"--heading-gains={args.heading_gains}")
    if args.no_claims:
        cmd.append("--no-claims")
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
                "spacing": args.spacing, "shape": args.shape, "leader": ids[0], **({"offsets": args.offsets} if args.offsets else {})}
        if args.angle is not None:
            body["angle"] = args.angle
        plan = api("/api/fleet/plan", body)
        plan_at = time.time()
        print(f"plan sent: {plan.get('mode')}", flush=True)
        (out / "scenario.json").write_text(json.dumps({"name": args.name, "mode": args.mode, "boats": args.boats,
                                                       "tdma": not args.no_tdma, "claims": not args.no_claims, "plan_at": plan_at, "duration": args.duration,
                                                       "kill_leader_at": args.kill_leader_at, "plan": plan, "world": args.world,
                                                       "vision": args.vision, "spacing": args.spacing, "shape": args.shape, "offsets": args.offsets,
                                                       "fast_beacon": args.fast_beacon, "heading_gains": args.heading_gains}, indent=1))
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


OFF_ROUTE_M = 1.5   # further than this from its route, a boat is off it (round an obstacle, blown off)


def off_route(x: float, y: float, pts: list[tuple[float, float]]) -> float:
    """Distance from (x, y) to the route's polyline."""
    best = math.inf
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        dx, dy = bx - ax, by - ay
        k = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy))) if dx or dy else 0.0
        best = min(best, math.hypot(x - ax - k * dx, y - ay - k * dy))
    return best


# A loop: the heading turns LOOP_DEG one way within LOOP_S. A lane end takes 180 to 205 deg in 90 s; the circles seen
# at the lane ends (round 4, img/v4_01) were 290 (a circle of the simulated boat, 6 m round at 0.5 m/s, takes ~75 s).
LOOP_DEG, LOOP_S = 270.0, 90.0


def loops(rows: list[dict]) -> int:
    """How many times the boat went round in a circle: the heading, unwrapped, changing by LOOP_DEG within LOOP_S."""
    unwrapped, total, last = [], 0.0, None
    for r in rows:
        if last is not None:
            total += (r["heading"] - last + 180.0) % 360.0 - 180.0
        last = r["heading"]
        unwrapped.append((r["t"], total))
    count, i = 0, 0
    while i < len(unwrapped):
        t, h = unwrapped[i]
        j = next((k for k in range(i + 1, len(unwrapped)) if unwrapped[k][0] - t > LOOP_S
                  or abs(unwrapped[k][1] - h) >= LOOP_DEG), None)
        if j is not None and abs(unwrapped[j][1] - h) >= LOOP_DEG:
            count, i = count + 1, j
        else:
            i += 1
    return count


def lane_end_turns(rows: list[dict], pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Each turn from one lane onto the next (lanes are the legs 0-1, 2-3, ...; the legs between are the short ones
    across): (seconds from within 4 m of the lane's end until on the next lane, within OFF_ROUTE_M and 30 deg of its
    heading; the furthest off the route meanwhile)."""
    out = []
    for i in range(1, len(pts) - 2, 2):
        end, (ax, ay), (bx, by) = pts[i], pts[i + 1], pts[i + 2]
        lane = math.degrees(math.atan2(bx - ax, by - ay)) % 360.0
        start = next((k for k, r in enumerate(rows) if math.hypot(r["x"] - end[0], r["y"] - end[1]) <= 4.0), None)
        if start is None:
            continue
        done = next((k for k in range(start, len(rows)) if off_route(rows[k]["x"], rows[k]["y"], [pts[i + 1], pts[i + 2]]) <= OFF_ROUTE_M
                     and abs((rows[k]["heading"] - lane + 180.0) % 360.0 - 180.0) <= 30.0), None)
        if done is None:
            continue
        out.append((rows[done]["t"] - rows[start]["t"],
                    max(off_route(r["x"], r["y"], pts) for r in rows[start:done + 1])))
    return out


def lane_keeping(d: Path, sc: dict, tracks: dict, t0: float, per_boat: dict) -> None:
    """Per boat with a route of its own (each boat in sectors, the leader in formation), from the first time it is
    within OFF_ROUTE_M of it: how far off (median, p90), the spells off it (count, longest, furthest); for every
    boat: the share of its time under way at 0 throttle, and how often the Pi stopped or backed off for an obstacle
    (pi.log logs each change of command)."""
    plan = sc.get("plan") or {}
    routes = {r["id"]: r["pts"] for r in plan.get("routes", [])} if sc["mode"] == "sectors" else \
        ({plan["leader"]: plan["route"]} if plan.get("route") else {})
    for boat, tr in tracks.items():
        if boat not in per_boat:
            continue
        rows = [r for r in tr["rows"] if r["t"] >= t0]
        moving = [r for r in rows if r["mission"] in ("P", "O")]
        if moving:
            per_boat[boat]["zero_throttle_pct"] = round(100.0 * sum(r["throttle"] <= 0.0 for r in moving) / len(moving), 1)
        log = d / f"boat_{boat[-1]}" / "pi.log"
        if log.exists():
            text = log.read_text(errors="replace")
            per_boat[boat]["stops"] = len(re.findall(r"NAV STOP [\d.]+ m/s \d+ \((?:obstacle_too_close|bow_blocked)\)", text))
            per_boat[boat]["backs"] = len(re.findall(r"NAV BACK ", text))
            per_boat[boat]["overspeed_cuts"] = len(re.findall(r"\(overspeed\)", text))
        per_boat[boat]["loops"] = loops(rows)
        if boat not in routes:
            continue
        pts = [to_xy(*p) for p in routes[boat]]
        turns = lane_end_turns(rows, pts)
        if turns:
            per_boat[boat]["turn_s"] = [round(t) for t, _ in turns]
            per_boat[boat]["turn_off_max_m"] = round(max(o for _, o in turns), 1)
        offs = [(r["t"], off_route(r["x"], r["y"], pts)) for r in rows]
        start = next((i for i, (_, o) in enumerate(offs) if o <= OFF_ROUTE_M), None)
        if start is None:
            continue
        offs = offs[start:]
        vals = sorted(o for _, o in offs)
        spells, cur = [], None
        for t, o in offs:
            if o > OFF_ROUTE_M:
                cur = [t, t, o] if cur is None else [cur[0], t, max(cur[2], o)]
            elif cur is not None:
                spells.append(cur)
                cur = None
        if cur is not None:
            spells.append(cur)
        per_boat[boat] |= {
            "off_route_median_m": round(vals[len(vals) // 2], 2), "off_route_p90_m": round(vals[int(0.9 * (len(vals) - 1))], 2),
            "off_spells": len(spells), "off_s": round(sum(b - a for a, b, _ in spells)),
            "off_max_m": round(max((m for _, _, m in spells), default=0.0), 1),
            "route_m": round(sum(math.dist(a, b) for a, b in zip(pts, pts[1:])), 1)}


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
    # Sectors: the share of each boat's samples inside its own strip (its route's box, half a lane wider all
    # round), from the first time it is in there; and whether it got to the end of its route
    if sc["mode"] == "sectors":
        half = sc.get("spacing", 8.0) / 2
        for route in (sc.get("plan") or {}).get("routes", []):
            pts = [to_xy(*p) for p in route["pts"]]
            rows = [r for r in tracks.get(route["id"], {"rows": []})["rows"] if r["t"] >= t0]
            if not pts or route["id"] not in per_boat:
                continue
            x0, x1 = min(p[0] for p in pts) - half, max(p[0] for p in pts) + half
            y0, y1 = min(p[1] for p in pts) - half, max(p[1] for p in pts) + half
            ins = [x0 <= r["x"] <= x1 and y0 <= r["y"] <= y1 for r in rows]
            ins = ins[ins.index(True):] if True in ins else []
            per_boat[route["id"]]["in_strip_pct"] = round(100.0 * sum(ins) / len(ins), 1) if ins else 0.0
            per_boat[route["id"]]["route_done"] = any(math.hypot(r["x"] - pts[-1][0], r["y"] - pts[-1][1]) < 4.0 for r in rows)
    lane_keeping(d, sc, tracks, t0, per_boat)
    # Close passes: seconds two boats were within CLOSE_M
    # Duplicate stops: a boat following somebody another boat reported first (by the first report's time)
    dup_stop, follow_s = 0, 0
    for boat, tr in tracks.items():
        for r in tr["rows"]:
            if r["t"] < t0 or r["mission"] != "F":
                continue
            follow_s += 1
            near = min(((p[0], math.hypot(p[1] - r["x"], p[2] - r["y"])) for p in people), key=lambda v: v[1], default=(None, 1e9))
            owner = first.get(near[0])
            if near[1] <= STOP_M and owner is not None and owner[1] != boat and owner[0] + t0 <= r["t"]:
                dup_stop += 1
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
                # right of a compass heading h: (cos h, -sin h); back: (-sin h, -cos h). On a lane the other way
                # about a follower keeps its side of the water, the leader's other side (modules/formation.py):
                # the nearer of the two places
                errs.append(min(math.hypot(fx - (lx + s * place["right"] * math.cos(h) - place["back"] * math.sin(h)),
                                           fy - (ly - s * place["right"] * math.sin(h) - place["back"] * math.cos(h))) for s in (1, -1)))
        errs.sort()
        formation = {"median_m": round(errs[len(errs) // 2], 1) if errs else None, "samples": len(errs)}
    try:
        air = json.loads((d / "air_station.json").read_text())
    except (OSError, ValueError):
        air = {}
    found_times = sorted(t for t, _ in first.values())
    return {
        "name": sc["name"], "world": sc["world"], "area_xy": area, "mode": sc["mode"], "boats": sc["boats"], "tdma": sc["tdma"],
        "claims": sc.get("claims", False), "duration_s": sc["duration"], "dup_stop_s": dup_stop, "follow_s": follow_s,
        "kill_leader_at": sc.get("kill_leader_at"),
        "people": len(people), "found": len(first),
        "found_by": {k: {"after_s": round(v[0]), "boat": v[1], "boats": sorted(reporters.get(k, []))} for k, v in sorted(first.items())},
        "time_half_s": round(found_times[(len(people) + 1) // 2 - 1]) if people and len(found_times) >= (len(people) + 1) // 2 else None,
        "time_all_s": round(found_times[-1]) if people and len(first) == len(people) else None,
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


COLOURS = {"PHAO-01": "#1f6fd1", "PHAO-02": "#d6402b", "PHAO-03": "#1f9d55", "PHAO-04": "#8e44ad"}
# Footprint radius (m) of the debris models, for the map
DEBRIS_R = {"flooded_house": 3.5, "flooded_house_tin": 3.5, "storm_house": 3.2, "wrecked_boat": 4.0,
            "bamboo_clump": 1.0, "basket_boat": 1.0, "broken_pole": 0.4, "spherical_buoy": 0.5}


def plot(args: argparse.Namespace) -> int:
    """The tracks of one run as a picture: area, the plan, each boat's way, debris, people, reports."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    d = Path(args.run).expanduser()
    r = analyse(d)
    if r is None:
        print(f"no scenario.json in {d}", file=sys.stderr)
        return 1
    sc = json.loads((d / "scenario.json").read_text())
    plan = sc.get("plan") or {}
    if args.bg:
        return plot_on_view(args, d, sc, r, plan)
    text = (d / "worlds" / f"vedc_{sc['world']}.sdf").read_text()
    t0, t1 = sc["plan_at"], sc["plan_at"] + sc["duration"]
    fig, ax = plt.subplots(figsize=(12, 7.8), dpi=150)
    area = r["area_xy"] + r["area_xy"][:1]
    ax.plot([p[0] for p in area], [p[1] for p in area], color="#333", lw=1.2, ls="--", label="vùng tìm kiếm")
    debris = [(float(m.group(1)), float(m.group(2)), DEBRIS_R[m.group(3)])
              for m in re.finditer(r"<pose>([-\d.]+) ([-\d.]+)[^<]*</pose><uri>model://(\w+)</uri>", text) if m.group(3) in DEBRIS_R]
    for x, y, rad in debris:
        ax.add_patch(plt.Circle((x, y), rad, color="#8a7a66", alpha=0.55, lw=0))
    if debris:
        ax.scatter([], [], s=80, color="#8a7a66", alpha=0.55, label="vật cản")
    # The plan: each strip in sectors, the leader's sweep in formation
    for route in plan.get("routes", []):
        pts = [to_xy(*p) for p in route["pts"]]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=COLOURS.get(route["id"], "#555"), lw=0.8, ls=":", alpha=0.8)
    if plan.get("route"):
        pts = [to_xy(*p) for p in plan["route"]]
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color="#555", lw=0.8, ls=":", label="lộ trình dẫn đầu (kế hoạch)")
    for b in sorted(d.glob("boat_*")):
        boat = f"PHAO-0{b.name.split('_')[1]}"
        rows = [x for x in read_track(b / "track.csv") if t0 - 5 <= x["t"] <= t1] if (b / "track.csv").exists() else []
        if not rows:
            continue
        c = COLOURS.get(boat, "#555")
        role = " (dẫn đầu)" if plan.get("leader") == boat else ""
        ax.plot([x["x"] for x in rows], [x["y"] for x in rows], color=c, lw=1.8, label=boat + role)
        ax.plot(rows[0]["x"], rows[0]["y"], "o", color=c, ms=7, mec="white")
        for x in rows[30::30]:   # an arrow every 30 s along the way, the boat's heading
            h = math.radians(x["heading"])
            ax.annotate("", xy=(x["x"] + 1.6 * math.sin(h), x["y"] + 1.6 * math.cos(h)), xytext=(x["x"], x["y"]),
                        arrowprops={"arrowstyle": "-|>", "color": c, "lw": 1.2})
    if r["victims_xy"]:
        ax.scatter([v[1] for v in r["victims_xy"]], [v[2] for v in r["victims_xy"]], marker="x", s=90, color="black", lw=2, label="người gặp nạn", zorder=5)
    reps = [(x, y, boat) for boat, xy in r["reports_xy"].items() for x, y in xy]
    if reps:
        ax.scatter([p[0] for p in reps], [p[1] for p in reps], s=40, facecolors="none", edgecolors=[COLOURS.get(p[2], "#555") for p in reps],
                   lw=1.5, label="báo VICTIM_FOUND", zorder=6)
    ax.set_aspect("equal")
    ax.set_xlabel("Đông (m)")
    ax.set_ylabel("Bắc (m)")
    ax.grid(alpha=0.25)
    ax.set_title(args.title or f"{sc['name']}: {sc['mode']}, {sc['boats']} phao, {sc['world']}")
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=9)
    fig.tight_layout()
    out = Path(args.png).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    print(out)
    return 0


# The overview camera of the wide worlds (sim/worlds/vedc_wide_*.sdf): straight down from 66 m over (30, 0), hfov
# 1.0 rad over 1280 x 800, east to the right and north up in the picture
VIEW_CENTRE, VIEW_HEIGHT_M, VIEW_HFOV, VIEW_W, VIEW_H = (30.0, 0.0), 66.0, 1.0, 1280, 800
VIEW_COLOURS = {"PHAO-01": (0, 200, 255), "PHAO-02": (60, 90, 255), "PHAO-03": (216, 79, 255), "PHAO-04": (255, 255, 0)}  # BGR


def view_px(x: float, y: float) -> tuple[int, int]:
    """Pixel of the point (x east, y north) in the overview camera's picture."""
    scale = VIEW_W / (2.0 * VIEW_HEIGHT_M * math.tan(VIEW_HFOV / 2.0))
    return round(VIEW_W / 2 + (x - VIEW_CENTRE[0]) * scale), round(VIEW_H / 2 - (y - VIEW_CENTRE[1]) * scale)


def plot_on_view(args: argparse.Namespace, d: Path, sc: dict, r: dict, plan: dict) -> int:
    """The tracks drawn on a frame of the overview camera (--bg), the style of record_media.py overview_with_track:
    each boat's way in its colour, green where it started, white where it ended, people in red, the boat's
    VICTIM_FOUND as a ring of its colour, the plan as thin dots; a caption band below."""
    import cv2
    import numpy as np
    from record_media import caption
    image = cv2.imread(str(Path(args.bg).expanduser()))
    if image is None or image.shape[:2] != (VIEW_H, VIEW_W):
        print(f"{args.bg}: not a {VIEW_W}x{VIEW_H} frame of the overview camera", file=sys.stderr)
        return 1
    t0, t1 = sc["plan_at"], sc["plan_at"] + sc["duration"]
    routes = [(route["id"], route["pts"]) for route in plan.get("routes", [])] + ([(plan["leader"], plan["route"])] if plan.get("route") else [])
    for boat, pts in routes:
        px = [view_px(*to_xy(*p)) for p in pts]
        for a, b in zip(px, px[1:]):
            n = max(1, int(math.dist(a, b) // 9))
            for k in range(n + 1):   # dotted: what the station gave the boat
                cv2.circle(image, (round(a[0] + (b[0] - a[0]) * k / n), round(a[1] + (b[1] - a[1]) * k / n)), 2,
                           (235, 235, 235), -1, cv2.LINE_AA)
    legend = []
    for b in sorted(d.glob("boat_*")):
        boat = f"PHAO-0{b.name.split('_')[1]}"
        rows = [x for x in read_track(b / "track.csv") if t0 - 5 <= x["t"] <= t1] if (b / "track.csv").exists() else []
        if not rows:
            continue
        colour = VIEW_COLOURS.get(boat, (255, 255, 255))
        pts = [view_px(x["x"], x["y"]) for x in rows]
        cv2.polylines(image, [np.array(pts, np.int32)], False, colour, 3, cv2.LINE_AA)
        for x in rows[30::30]:   # the way it went, every 30 s
            h = math.radians(x["heading"])
            a = view_px(x["x"], x["y"])
            cv2.arrowedLine(image, a, view_px(x["x"] + 1.2 * math.sin(h), x["y"] + 1.2 * math.cos(h)), colour, 2, cv2.LINE_AA, tipLength=0.6)
        cv2.circle(image, pts[0], 7, (0, 255, 0), -1, cv2.LINE_AA)
        cv2.circle(image, pts[-1], 7, (255, 255, 255), -1, cv2.LINE_AA)
        legend.append(boat + (" (dẫn đầu)" if plan.get("leader") == boat else ""))
        for x, y in r["reports_xy"].get(boat, []):
            cv2.circle(image, view_px(x, y), 9, colour, 2, cv2.LINE_AA)
    for _, x, y in r["victims_xy"]:
        cv2.circle(image, view_px(x, y), 12, (0, 0, 255), 3, cv2.LINE_AA)
    title = args.title or f"{sc['name']}: {sc['mode']}, {sc['boats']} phao, {sc['world']}"
    names = "   ".join(f"{n}: {'vàng' if n.startswith('PHAO-01') else 'đỏ cam' if n.startswith('PHAO-02') else 'hồng'}" for n in legend)
    key = f"{names}   |   chấm xanh lá: xuất phát, trắng: kết thúc, chấm mờ: lộ trình được giao" + \
          ("   |   vòng đỏ: người gặp nạn" if r["victims_xy"] else "")
    image = caption(image, [(title, 22, True), (args.stats or key, 16, False)] + ([(key, 15, False)] if args.stats else []), top=False)
    out = Path(args.png).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), image, [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(out)
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
    r.add_argument("--no-claims", action="store_true", help="the station tells no boat to leave a person alone (docs/victim-claim.md)")
    r.add_argument("--vision", default="all", choices=("all", "first"))
    r.add_argument("--hold", type=int, default=20, help="seconds with a person before going on (follow.max_hold_s), the same for every plan")
    r.add_argument("--kill-leader-at", type=int, default=0, help="seconds after the plan: PHAO-01's board stops")
    r.add_argument("--out", default="~/vedc_sim/compare")
    r.add_argument("--gui", action="store_true", help="with the Gazebo window (to film it)")
    r.add_argument("--offsets", choices=("ned",), help="formation places as metres north, east (ArduPilot AP_Follow FOLL_OFS_TYPE 0)")
    r.add_argument("--fast-beacon", action="store_true", help="simulation only: SF7, 0.6 s slots, a beacon every 2.4 s (fleet_sim.sh)")
    r.add_argument("--heading-gains", default="firmware", choices=("firmware", "tuned", "legacy"),
                   help="the simulated F407's heading PID (stm32_board_sim.py HEADING_GAINS)")
    p = sub.add_parser("report")
    p.add_argument("--out", default="~/vedc_sim/compare")
    g = sub.add_parser("plot")
    g.add_argument("--run", required=True, help="one scenario's directory, <out>/<name>")
    g.add_argument("--png", required=True)
    g.add_argument("--title")
    g.add_argument("--bg", help="a frame of the overview camera (record_media.py record --topic top=/vedc_wide/overview): draw on it")
    g.add_argument("--stats", help="with --bg: a line of figures under the title")
    args = parser.parse_args()
    return {"run": run, "report": report, "plot": plot}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
