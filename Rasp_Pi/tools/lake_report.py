"""How well the boat steers, from main.py's logs/nav_*.csv (utils/run_log.py), as one HTML page with charts.
Runs on the PC after a lake test (docs/lake-test.md, scripts/stm32/lake_report.bat). From Rasp_Pi:

    python tools/lake_report.py logs/nav_20261120_093000.csv [more.csv ...] [-o report.html]
    python tools/lake_report.py --sample sample.csv     # a made-up square run, to try the report

Measured on the rows where the boat drives (cmd_mode AUTO, cmd_speed > 0):
  heading     yaw (BNO055) minus the $NAV heading the F407's PID holds; steady = away from the turns
  turns       each step of the commanded heading of TURN_DEG or more: settling time and overshoot
  compass     yaw minus the GPS course on straight runs: the compass's offset
  legs        cross-track error to the line from the previous waypoint, time, path length, distance
              to the waypoint when the boat moved on to the next one (vehicle.arrive_radius_m)
  speed       GPS speed against the commanded speed, per 0.1 m/s
"""
from __future__ import annotations

import argparse
import base64
import csv
import html
import io
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from utils.geometry import bearing_deg, haversine_m, offset_latlon, signed_angle_deg   # noqa: E402
from utils.run_log import COLUMNS                                                       # noqa: E402

TURN_DEG = 30.0          # a commanded heading step this large is a turn
TURN_WINDOW_S = 1.0      # ... within this time
SETTLED_DEG = 10.0       # settled: |error| below this ...
SETTLED_HOLD_S = 2.0     # ... for this long
TURN_MAX_S = 20.0        # a turn not settled by then counts as never settled
COURSE_MIN_MPS = 0.3     # GPS course is noise below this speed
SPEED_HOLD_S = 3.0       # speed compared once the command has stood this long

# name: (the value's key, low, high) -- passes when low <= value <= high
LIMITS = {
    "heading RMS error (deg)": ("heading_rms", 0.0, 10.0),
    "heading p95 |error| (deg)": ("heading_p95", 0.0, 20.0),
    "worst turn settling (s)": ("settle_max", 0.0, 8.0),
    "worst turn overshoot (deg)": ("overshoot_max", 0.0, 15.0),
    "compass offset to GPS course (deg)": ("compass_offset", -10.0, 10.0),
    "mean |cross-track| (m)": ("xte_mean", 0.0, 2.0),
    "max |cross-track| (m)": ("xte_max", 0.0, 5.0),
    "worst distance at waypoint (m)": ("arrive_max", 0.0, 3.0),
    "worst path / straight line": ("efficiency_max", 0.0, 1.3),
    "GPS speed / command": ("speed_ratio", 0.7, 1.2),
}


def load(paths: list[Path]) -> list[dict]:
    """Rows as numbers; t_s made to run on across files."""
    rows: list[dict] = []
    offset = 0.0
    for path in paths:
        with path.open(encoding="utf-8", newline="") as f:
            start = offset
            for r in csv.DictReader(f):
                num = lambda k: float(r[k]) if r.get(k) not in (None, "") else None   # noqa: E731
                rows.append({"t": start + (num("t_s") or 0.0), "lat": num("lat"), "lon": num("lon"), "fix": int(num("fix") or 0),
                             "speed": num("speed_mps") or 0.0, "yaw": num("yaw_deg") or 0.0, "course": num("course_deg"),
                             "mode": r.get("cmd_mode", ""), "cmd_speed": num("cmd_speed_mps") or 0.0, "cmd_heading": num("cmd_heading_deg"),
                             "reason": r.get("reason", ""), "wp": r.get("wp_index", ""), "wp_lat": num("wp_lat"), "wp_lon": num("wp_lon")})
                offset = rows[-1]["t"] + 1.0
    return rows


def driving(r: dict) -> bool:
    return r["mode"] == "AUTO" and r["cmd_speed"] > 0.0 and r["cmd_heading"] is not None


def heading_error(r: dict) -> float:
    """Positive: the bow is right (clockwise) of the commanded heading."""
    return signed_angle_deg(r["yaw"] - r["cmd_heading"])


def turns(rows: list[dict]) -> list[dict]:
    """Steps of the commanded heading: when, how large, settling time (None: not settled within
    TURN_MAX_S or the run ended) and overshoot (the largest error past the new heading)."""
    drive = [r for r in rows if driving(r)]
    found: list[dict] = []
    j = 0
    for i, r in enumerate(drive):
        while drive[j]["t"] < r["t"] - TURN_WINDOW_S:
            j += 1
        step = signed_angle_deg(r["cmd_heading"] - drive[j]["cmd_heading"])
        if abs(step) < TURN_DEG or (found and r["t"] - found[-1]["t"] < TURN_MAX_S / 2):
            continue
        after = [x for x in drive[i:] if x["t"] - r["t"] <= TURN_MAX_S]
        settle = None
        for k, x in enumerate(after):
            if abs(heading_error(x)) >= SETTLED_DEG:
                continue
            hold = [y for y in after[k:] if y["t"] - x["t"] <= SETTLED_HOLD_S]
            if hold[-1]["t"] - x["t"] >= SETTLED_HOLD_S - 0.15 and all(abs(heading_error(y)) < SETTLED_DEG for y in hold):
                settle = x["t"] - r["t"]
                break
        # The error starts on the side the boat turns from; past the new heading it changes sign.
        sign = math.copysign(1.0, heading_error(r))
        overshoot = max((-sign * heading_error(x) for x in after), default=0.0)
        found.append({"t": r["t"], "step": step, "settle": settle, "overshoot": max(0.0, overshoot)})
    return found


def in_turn(t: float, found: list[dict]) -> bool:
    return any(0.0 <= t - x["t"] <= (x["settle"] if x["settle"] is not None else TURN_MAX_S) for x in found)


def cross_track_m(start: tuple[float, float], end: tuple[float, float], lat: float, lon: float) -> float:
    """Signed distance of (lat, lon) from the line start -> end, positive to its right."""
    return haversine_m(*start, lat, lon) * math.sin(math.radians(bearing_deg(*start, lat, lon) - bearing_deg(*start, *end)))


def legs(rows: list[dict]) -> list[dict]:
    """The route's legs, one per waypoint gone to. Starts at the previous waypoint, or where the boat
    was when it set off for the first. reached: the boat moved on (a later row has another waypoint
    or none), so end_m is how far from the waypoint it was then."""
    out: list[dict] = []
    for i, r in enumerate(rows):
        if r["wp_lat"] is None:
            continue
        key = (r["wp"], r["wp_lat"], r["wp_lon"])
        if not out or out[-1]["key"] != key or out[-1]["last"] != i - 1:
            prev = out[-1] if out and out[-1]["last"] == i - 1 else None
            start = prev["wp_pos"] if prev else (r["lat"], r["lon"])
            out.append({"key": key, "wp": r["wp"], "wp_pos": (r["wp_lat"], r["wp_lon"]), "start": start, "first": i, "last": i})
        out[-1]["last"] = i
    for leg in out:
        part = [r for r in rows[leg["first"]:leg["last"] + 1] if r["fix"] > 0]
        xte = [abs(cross_track_m(leg["start"], leg["wp_pos"], r["lat"], r["lon"])) for r in part if driving(r)]
        path = sum(haversine_m(a["lat"], a["lon"], b["lat"], b["lon"]) for a, b in zip(part, part[1:]))
        straight = haversine_m(*leg["start"], *leg["wp_pos"])
        leg.update(time_s=rows[leg["last"]]["t"] - rows[leg["first"]]["t"], xte=xte,
                   xte_mean=sum(xte) / len(xte) if xte else None, xte_max=max(xte, default=None), path_m=path, straight_m=straight,
                   efficiency=path / straight if straight > 1.0 else None,
                   end_m=haversine_m(part[-1]["lat"], part[-1]["lon"], *leg["wp_pos"]) if part else None,
                   reached=leg["last"] + 1 < len(rows))
    return out


def speeds(rows: list[dict]) -> list[dict]:
    """Per commanded speed (0.1 m/s): rows, mean GPS speed and its ratio to the command, once the
    command has stood SPEED_HOLD_S."""
    buckets: dict[float, list[float]] = {}
    since, level = None, None
    for r in rows:
        cmd = round(r["cmd_speed"], 1) if driving(r) and r["fix"] > 0 else None
        if cmd != level:
            since, level = r["t"], cmd
        if cmd and r["t"] - since >= SPEED_HOLD_S:
            buckets.setdefault(cmd, []).append(r["speed"])
    return [{"cmd": c, "n": len(v), "gps": sum(v) / len(v), "ratio": sum(v) / len(v) / c} for c, v in sorted(buckets.items())]


def percentile(values: list[float], q: float) -> float:
    s = sorted(values)
    return s[min(len(s) - 1, int(q * len(s)))]


def analyse(rows: list[dict]) -> dict:
    found = turns(rows)
    steady = [heading_error(r) for r in rows if driving(r) and not in_turn(r["t"], found)]
    compass = [signed_angle_deg(r["yaw"] - r["course"]) for r in rows
               if driving(r) and r["fix"] > 0 and r["speed"] > COURSE_MIN_MPS and r["course"] is not None and not in_turn(r["t"], found)]
    route = legs(rows)
    reached = [x for x in route if x["reached"]]
    xte = [e for x in route for e in x["xte"]]
    speed = speeds(rows)
    n = sum(s["n"] for s in speed)
    v = {
        "rows": len(rows), "drive_s": sum(min(1.0, b["t"] - a["t"]) for a, b in zip(rows, rows[1:]) if driving(a)),
        "heading_mean": sum(steady) / len(steady) if steady else None,
        "heading_rms": math.sqrt(sum(e * e for e in steady) / len(steady)) if steady else None,
        "heading_p95": percentile([abs(e) for e in steady], 0.95) if steady else None,
        "settle_max": None if not found else (TURN_MAX_S if any(x["settle"] is None for x in found) else max(x["settle"] for x in found)),
        "overshoot_max": max((x["overshoot"] for x in found), default=None),
        "compass_offset": sum(compass) / len(compass) if compass else None,
        "compass_sd": math.sqrt(sum((c - sum(compass) / len(compass)) ** 2 for c in compass) / len(compass)) if compass else None,
        "xte_mean": sum(xte) / len(xte) if xte else None, "xte_max": max(xte, default=None),
        "arrive_max": max((x["end_m"] for x in reached if x["end_m"] is not None), default=None),
        "efficiency_max": max((x["efficiency"] for x in reached if x["efficiency"] is not None), default=None),
        "speed_ratio": sum(s["ratio"] * s["n"] for s in speed) / n if n else None,
    }
    checks = [(name, v[key], None if v[key] is None else lo <= v[key] <= hi, lo, hi) for name, (key, lo, hi) in LIMITS.items()]
    return {"values": v, "checks": checks, "turns": found, "legs": route, "speeds": speed}


# -- the page ------------------------------------------------------------------------------------
def _png(fig) -> str:
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=110, bbox_inches="tight")
    return f'<img src="data:image/png;base64,{base64.b64encode(buf.getvalue()).decode()}">'


def charts(rows: list[dict], result: dict) -> list[str]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out = []
    fixed = [r for r in rows if r["fix"] > 0 and r["lat"] is not None]
    if fixed:
        lat0, lon0 = fixed[0]["lat"], fixed[0]["lon"]
        k = math.cos(math.radians(lat0)) * 111_320.0
        en = lambda la, lo: ((lo - lon0) * k, (la - lat0) * 110_540.0)   # noqa: E731
        fig, ax = plt.subplots(figsize=(7, 7))
        for i, leg in enumerate(result["legs"]):
            pts = [en(r["lat"], r["lon"]) for r in rows[leg["first"]:leg["last"] + 1] if r["fix"] > 0]
            if pts:
                ax.plot(*zip(*pts), lw=1.5, label=f"leg {i + 1} -> wp {leg['wp']}")
            ax.plot(*zip(en(*leg["start"]), en(*leg["wp_pos"])), "k--", lw=0.8)
            ax.add_patch(plt.Circle(en(*leg["wp_pos"]), 3.0, fill=False, color="grey", lw=0.8))
            ax.plot(*en(*leg["wp_pos"]), "kx")
        rest = [en(r["lat"], r["lon"]) for r in fixed if r["wp_lat"] is None]
        if rest:
            ax.plot(*zip(*rest), ".", ms=1, color="lightgrey", label="no waypoint")
        ax.set_aspect("equal")
        ax.set_xlabel("east (m)")
        ax.set_ylabel("north (m)")
        ax.set_title("Track, legs (dashed) and the 3 m arrive circle")
        ax.legend(fontsize=7)
        out.append(_png(fig))
        plt.close(fig)
    drive = [r for r in rows if driving(r)]
    if drive:
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        t = [r["t"] for r in drive]
        a1.plot(t, [r["cmd_heading"] for r in drive], ".", ms=2, label="command ($NAV)")
        a1.plot(t, [r["yaw"] for r in drive], ".", ms=2, label="yaw (BNO055)")
        a1.set_ylabel("heading (deg)")
        a1.legend(fontsize=7)
        a2.plot(t, [heading_error(r) for r in drive], ".", ms=2)
        for x in result["turns"]:
            a2.axvline(x["t"], color="orange", lw=0.8)
        a2.axhspan(-SETTLED_DEG, SETTLED_DEG, color="green", alpha=0.1)
        a2.set_ylabel("error (deg)")
        a2.set_xlabel("time (s); orange: turns")
        out.append(_png(fig))
        plt.close(fig)
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
        a1.plot(t, [r["cmd_speed"] for r in drive], ".", ms=2, label="command")
        a1.plot(t, [r["speed"] for r in drive], ".", ms=2, label="GPS")
        a1.set_ylabel("speed (m/s)")
        a1.legend(fontsize=7)
        for leg in result["legs"]:
            part = [r for r in rows[leg["first"]:leg["last"] + 1] if r["fix"] > 0 and driving(r)]
            a2.plot([r["t"] for r in part], [abs(cross_track_m(leg["start"], leg["wp_pos"], r["lat"], r["lon"])) for r in part], ".", ms=2)
        a2.set_ylabel("|cross-track| (m)")
        a2.set_xlabel("time (s)")
        out.append(_png(fig))
        plt.close(fig)
    return out


def _f(v: float | None, digits: int = 1) -> str:
    return "-" if v is None else f"{v:.{digits}f}"


def page(title: str, rows: list[dict], result: dict, images: list[str]) -> str:
    v = result["values"]
    checks = "".join(f'<tr class="{"" if ok is None else "ok" if ok else "bad"}"><td>{html.escape(n)}</td><td>{_f(val, 2)}</td>'
                     f'<td>{_f(lo)} .. {_f(hi)}</td><td>{"no data" if ok is None else "PASS" if ok else "FAIL"}</td></tr>'
                     for n, val, ok, lo, hi in result["checks"])
    legs_html = "".join(f"<tr><td>{i + 1}</td><td>{html.escape(x['wp'])}</td><td>{_f(x['time_s'])}</td><td>{_f(x['straight_m'])}</td>"
                        f"<td>{_f(x['path_m'])}</td><td>{_f(x['efficiency'], 2)}</td><td>{_f(x['xte_mean'])}</td><td>{_f(x['xte_max'])}</td>"
                        f"<td>{_f(x['end_m'])}</td><td>{'yes' if x['reached'] else 'no'}</td></tr>" for i, x in enumerate(result["legs"]))
    turns_html = "".join(f"<tr><td>{_f(x['t'])}</td><td>{_f(x['step'])}</td><td>{_f(x['settle']) if x['settle'] is not None else 'never'}</td>"
                         f"<td>{_f(x['overshoot'])}</td></tr>" for x in result["turns"])
    speed_html = "".join(f"<tr><td>{s['cmd']:.1f}</td><td>{s['n']}</td><td>{s['gps']:.2f}</td><td>{s['ratio']:.2f}</td></tr>" for s in result["speeds"])
    return f"""<!doctype html><html><head><meta charset="utf-8"><title>{html.escape(title)}</title><style>
body{{font:14px system-ui,sans-serif;margin:16px;max-width:1100px}} table{{border-collapse:collapse;margin:8px 0 20px}}
td,th{{border:1px solid #ccc;padding:3px 8px;text-align:right}} td:first-child{{text-align:left}}
tr.ok td:last-child{{background:#d4f4d4}} tr.bad td:last-child{{background:#f8d0d0}} img{{max-width:100%}}
</style></head><body><h1>{html.escape(title)}</h1>
<p>{v['rows']} rows, {v['drive_s']:.0f} s driving in AUTO. Steady heading error mean {_f(v['heading_mean'])} deg;
compass offset spread (SD) {_f(v['compass_sd'])} deg.</p>
<h2>Checks</h2><table><tr><th>metric</th><th>value</th><th>pass range</th><th></th></tr>{checks}</table>
<h2>Legs</h2><table><tr><th>#</th><th>waypoint</th><th>time s</th><th>straight m</th><th>path m</th><th>path/straight</th>
<th>mean |xte| m</th><th>max |xte| m</th><th>end dist m</th><th>reached</th></tr>{legs_html}</table>
<h2>Turns (command step &ge; {TURN_DEG:.0f} deg)</h2><table><tr><th>t s</th><th>step deg</th><th>settle s</th><th>overshoot deg</th></tr>{turns_html}</table>
<h2>Speed</h2><table><tr><th>command m/s</th><th>rows</th><th>GPS m/s</th><th>ratio</th></tr>{speed_html}</table>
{''.join(images)}</body></html>"""


# -- a made-up run, to try the report -------------------------------------------------------------
def sample(path: Path, seed: int = 1) -> None:
    """A 20 m square from a start beside its first corner: yaw follows the command at 25 deg/s with
    noise, the boat makes 0.45 m/s of the 0.5 asked, a little cross current pushes it east."""
    rnd = random.Random(seed)
    lat, lon = 10.7700000, 106.6600000
    corners = [offset_latlon(lat, lon, 0.0, 20.0)]
    for b in (90.0, 180.0, 270.0):
        corners.append(offset_latlon(*corners[-1], b, 20.0))
    yaw, t, i = 0.0, 0.0, 0
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, restval="")
        w.writeheader()
        while t < 400.0:
            wp = corners[i] if i < len(corners) else None
            if wp and haversine_m(lat, lon, *wp) < 3.0:
                i += 1
                continue
            cmd = bearing_deg(lat, lon, *wp) if wp else yaw
            speed_cmd = 0.5 if wp else 0.0
            yaw = (yaw + max(-2.5, min(2.5, signed_angle_deg(cmd - yaw))) + rnd.gauss(0.0, 1.0)) % 360.0
            speed = 0.45 * speed_cmd / 0.5
            lat, lon = offset_latlon(lat, lon, yaw, speed * 0.1)
            lat, lon = offset_latlon(lat, lon, 90.0, 0.03 * 0.1 if wp else 0.0)
            w.writerow({"t_s": f"{t:.1f}", "fix": 3, "lat": f"{lat:.7f}", "lon": f"{lon:.7f}", "speed_mps": f"{speed + rnd.gauss(0, 0.02):.2f}",
                        "yaw_deg": f"{yaw:.1f}", "course_deg": f"{(yaw + rnd.gauss(3.0, 2.0)) % 360:.1f}", "hdop": "0.9",
                        "phase": "PATROL" if wp else "IDLE", "cmd_mode": "AUTO" if wp else "STOP", "cmd_speed_mps": f"{speed_cmd:.2f}",
                        "cmd_heading_deg": f"{cmd:.1f}", "reason": "waypoint" if wp else "no_goal",
                        "wp_index": f"{i + 1 if wp else 0}/4", "wp_lat": f"{wp[0]:.7f}" if wp else "", "wp_lon": f"{wp[1]:.7f}" if wp else ""})
            t += 0.1
            if not wp and t > 0:
                break


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv", type=Path, nargs="*", help="logs/nav_*.csv from the Pi, read as one run in this order")
    ap.add_argument("-o", "--out", type=Path, help="the HTML page (default: beside the first CSV, .html)")
    ap.add_argument("--sample", type=Path, metavar="CSV", help="write a made-up square run to CSV and stop")
    args = ap.parse_args()
    if args.sample:
        sample(args.sample)
        print(f"sample run: {args.sample}")
        return 0
    if not args.csv:
        ap.error("no CSV given")
    rows = load(args.csv)
    if not rows:
        sys.exit("no rows in the CSV")
    result = analyse(rows)
    out = args.out or args.csv[0].with_suffix(".html")
    out.write_text(page(f"Lake test: {', '.join(p.name for p in args.csv)}", rows, result, charts(rows, result)), encoding="utf-8")
    for name, value, ok, _lo, _hi in result["checks"]:
        print(f"{'no data' if ok is None else 'PASS' if ok else 'FAIL':8s}{name}: {_f(value, 2)}")
    print(f"report: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
