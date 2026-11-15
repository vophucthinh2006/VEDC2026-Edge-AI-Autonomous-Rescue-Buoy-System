#!/usr/bin/env python3
"""The boat's yaw as identified by heading_step_test.py --sysid, and a grid search of the heading gains on it.

Model, per throttle: r' = (g(u) + bias - r) / T, heading' = r, with g(u) odd and piecewise linear through
the open-loop steady yaw rates at u = 0.3, 0.6, 1.0 (bias taken out: the mean of u = +0.6 and -0.6),
T = 0.3 s (the rate settles within one 1 s window in every sysid run), and the waves as a 1 Hz yaw-rate
wobble on the gyro. The controller is the board's own (stm32_board_sim.HeadingController), 10 ms steps.

    python3 sim/tools/heading_tune_model.py --sysid media/simulation/pid/sysid_metrics.csv [--check firmware]
"""
from __future__ import annotations

import argparse
import csv
import itertools
import math
import multiprocessing
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import stm32_board_sim as bs  # noqa: E402
from heading_step_test import step_metrics, wrap  # noqa: E402

T_YAW, DT = 0.3, 0.01
WAVE_DPS, WAVE_HZ = 4.0, 1.0


def plants(path: Path) -> dict[float, dict]:
    rate = {}
    for row in csv.DictReader(path.open()):
        rate[(float(row["throttle"]), float(row["step"]))] = float(row["rate_ss_dps"])
    out = {}
    for th in sorted({k[0] for k in rate}):
        bias = (rate[(th, 0.6)] + rate[(th, -0.6)]) / 2
        g06 = (rate[(th, 0.6)] - rate[(th, -0.6)]) / 2
        pts = [0.0, max(0.0, rate[(th, 0.3)] - bias), g06, max(g06, rate[(th, 1.0)] - bias)]
        out[th] = {"bias": bias, "g": pts}
    return out


def g_of(u: float, pts: list[float]) -> float:
    a = min(abs(u), 1.0)
    xs = (0.0, 0.3, 0.6, 1.0)
    for i in range(3):
        if a <= xs[i + 1]:
            return math.copysign(pts[i] + (pts[i + 1] - pts[i]) * (a - xs[i]) / (xs[i + 1] - xs[i]), u)
    return math.copysign(pts[3], u)


def simulate(gains: dict, plant: dict, throttle: float, profile, seconds: float, seed: int = 1):
    """Heading 0, straight, then the target from profile(t). Returns t, err, rate, u."""
    rnd = random.Random(seed)
    phase = rnd.uniform(0, 2 * math.pi)
    pid = bs.HeadingController(gains)
    h, r = 0.0, plant["bias"] * 0.0
    out = ([], [], [], [])
    lead = 8.0
    for k in range(int((lead + seconds) / DT)):
        t = k * DT - lead
        target = profile(t) % 360.0 if t >= 0 else 0.0
        gyro = r + WAVE_DPS * math.sin(2 * math.pi * WAVE_HZ * k * DT + phase)
        u = pid.update(target, h % 360.0, gyro, DT, throttle)
        r += DT / T_YAW * (g_of(u, plant["g"]) + plant["bias"] - r)
        h += r * DT
        if t >= 0:
            for lst, v in zip(out, (t, wrap(target - h), r, u)):
                lst.append(v)
    return out


def score(gains: dict, models: dict) -> tuple[float, list[dict]]:
    """Sum over throttles and steps of settling time (40 s if never) + overshoot/2 + 2 |steady error|, and u's jitter."""
    total, rows = 0.0, []
    for th, plant in models.items():
        for step in (30.0, -30.0, 90.0, -90.0, 180.0):
            t, err, rate, u = simulate(gains, plant, th, lambda _t, s=step: s, 40.0)
            m = step_metrics(t, err, step, [1.0] * len(t), rate)
            tail = u[-500:]
            jitter = math.sqrt(sum((x - sum(tail) / len(tail)) ** 2 for x in tail) / len(tail))
            settle = m["settle_s"] if not math.isnan(m["settle_s"]) else 40.0
            total += settle + 0.5 * (m["overshoot_deg"] if not math.isnan(m["overshoot_deg"]) else 20) + 2 * abs(m["sse_deg"]) + 20 * jitter
            rows.append({"throttle": th, "step": step, **m, "u_jitter": jitter})
    return total, rows


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sysid", type=Path, nargs="+", required=True, help="sysid_metrics.csv of heading_step_test.py --sysid")
    p.add_argument("--check", help="only print the model's step metrics for these gains (a HEADING_GAINS name)")
    args = p.parse_args()
    models = {}
    for path in args.sysid:
        models.update(plants(path))
    for th, m in models.items():
        print(f"throttle {th:.2f}: bias {m['bias']:+.2f} deg/s, g(0.3, 0.6, 1.0) = {m['g'][1]:.1f}, {m['g'][2]:.1f}, {m['g'][3]:.1f} deg/s")
    if args.check:
        _, rows = score(bs.HEADING_GAINS[args.check], models)
        for r in rows:
            print("  ".join(f"{k}={v:.2f}" for k, v in r.items()))
        return 0
    base = dict(bs.HEADING_GAINS["legacy"])
    grid = [dict(base, kp=kp, kd=kd, ki=ki, i_zone=zone, ref_throttle=ref, min_throttle=0.05 if ref else 0.0)
            for kp, kd, ki, zone, ref in itertools.product((0.02, 0.04, 0.06, 0.08, 0.10, 0.14, 0.2, 0.3), (0.0, 0.004, 0.008, 0.016),
                                                           (0.0, 0.002, 0.005, 0.01, 0.02, 0.04), (15.0, math.inf), (0.0, 0.15))]
    with multiprocessing.Pool() as pool:
        scores = pool.starmap(score, [(g, models) for g in grid])
    results = sorted(((s[0], g) for s, g in zip(scores, grid)), key=lambda x: x[0])
    print(f"legacy: {score(base, models)[0]:.0f}")
    for s, g in results[:15]:
        print(f"{s:7.0f}  kp {g['kp']:.3f} kd {g['kd']:.3f} ki {g['ki']:.3f} zone {g['i_zone']:g} ref {g['ref_throttle']:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
