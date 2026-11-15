#!/usr/bin/env python3
"""Checks of the board's heading controller options and of the step metrics. Run in WSL (the board
imports the gz modules):  python3 sim/tools/heading_controller_test.py"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import stm32_board_sim as bs  # noqa: E402
from heading_step_test import step_metrics  # noqa: E402

TUNED, LEGACY = bs.HEADING_GAINS["firmware"], bs.HEADING_GAINS["legacy"]
assert bs.HEADING_GAINS["tuned"] is TUNED


def firmware_reference(target, current, rates, dt, throttle):
    """App/heading_controller.c written out again, the constants of app_config.h."""
    integral, rate, out = 0.0, None, []
    scale = 0.15 / max(throttle, 0.05) if throttle < 0.15 else 1.0
    for r in rates:
        e = bs.heading_error(target, current)
        rate = r if rate is None else rate + dt / (1 / (2 * math.pi * 5.0) + dt) * (r - rate)
        d = -0.008 * rate * scale
        if abs(e) <= 2.0:
            out.append(max(-1.0, min(1.0, 0.010 * integral * scale + d)))
            continue
        cand = max(-100.0, min(100.0, integral + e * dt)) if abs(e) < 15.0 else integral
        raw = (0.10 * e + 0.010 * cand) * scale + d
        o = max(-1.0, min(1.0, raw))
        if raw == o or (o >= 1 and e < 0) or (o <= -1 and e > 0):
            integral = cand
        out.append(o)
    return out


def run(gains, target, current, rates, dt, throttle):
    pid = bs.HeadingController(gains)
    return [pid.update(target, current, r, dt, throttle) for r in rates]


# The default is the firmware, at every throttle.
rates = [3.0, 5.0, -2.0, 0.5] * 50
for target, current in ((40.0, 0.0), (10.0, 0.0), (1.0, 0.0), (350.0, 20.0), (180.0, 0.0)):
    for th in (0.0, 0.1, 0.25, 0.5):
        ours, ref = run(TUNED, target, current, rates, 0.01, th), firmware_reference(target, current, rates, 0.01, th)
        assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(ours, ref)), (target, th)
assert bs.throttle_scale(LEGACY, 0.1) == 1.0

# Throttle scheduling: ref / max(throttle, min).
if TUNED["ref_throttle"] > 0:
    ref, low = TUNED["ref_throttle"], TUNED["min_throttle"]
    assert bs.throttle_scale(TUNED, ref) == bs.throttle_scale(TUNED, 0.5) == 1.0
    assert math.isclose(bs.throttle_scale(TUNED, ref / 2), 2.0) or ref / 2 < low
    assert bs.throttle_scale(TUNED, 0.0) == bs.throttle_scale(TUNED, low) == ref / low
    # A small error at a low throttle asks for more than at the reference one, never past 1.
    lo = run(TUNED, 10.0, 0.0, [0.0], 0.01, low)[0]
    hi = run(TUNED, 10.0, 0.0, [0.0], 0.01, ref)[0]
    assert abs(lo) >= abs(hi) and abs(lo) <= 1.0

# The integral zone: far from the target the integral rests, close to it it runs.
if math.isfinite(TUNED["i_zone"]):
    pid = bs.HeadingController(TUNED)
    for _ in range(500):
        pid.update(TUNED["i_zone"] + 20.0, 0.0, 0.0, 0.01, 0.3)
    assert pid.integral == 0.0
    for _ in range(100):
        pid.update(TUNED["i_zone"] / 2, 0.0, 0.0, 0.01, 0.3)
    assert pid.integral > 0.0

# Step metrics on a made-up first-order turn: 90 deg, time constant 2 s, 1 m/s.
t = [i * 0.02 for i in range(1500)]
heading = [90.0 * (1 - math.exp(-x / 2.0)) for x in t]
rate = [45.0 * math.exp(-x / 2.0) for x in t]
m = step_metrics(t, [90.0 - h for h in heading], 90.0, [1.0] * len(t), rate)
assert math.isclose(m["rise_s"], 2.0 * math.log(9), abs_tol=0.05), m
assert m["overshoot_deg"] == 0.0 and abs(m["sse_deg"]) < 0.01
assert math.isclose(m["settle_s"], 2.0 * math.log(18), abs_tol=0.05), m
# An overshoot of 10 deg, then back on the target: counted, and settled.
heading = [min(x * 30.0, 100.0) if x < 5 else 90.0 for x in t]
m = step_metrics(t, [90.0 - h for h in heading], 90.0, [1.0] * len(t), [30.0] * len(t))
assert math.isclose(m["overshoot_deg"], 10.0) and math.isclose(m["settle_s"], 5.0, abs_tol=0.03), m
print("heading controller: ok")
