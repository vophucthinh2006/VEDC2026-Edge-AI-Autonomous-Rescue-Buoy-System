#!/usr/bin/env python3
"""Heading step response of the F407 heading loop alone, in Gazebo.

One boat on open water (sim/worlds/vedc_wide.sdf without its markers, people and sensor rendering), and
tools/stm32_board_sim.py as the F407. The script plays the Pi: $NAV AUTO <speed> <heading> and $HBT
on the board's pseudo-terminal, nothing from Rasp_Pi/, so only the F407's PID and mix are measured.
The true heading comes from the model's odometry (sim time).

Each case: motors off, the boat put back at the origin facing north, AUTO north at the case's throttle
(speed = throttle x 1 m/s) until it runs straight, then the target jumps by the step (or follows a
sine) and the response is recorded.

    step  +-30, +-90, 180 deg at throttle 0.08, 0.15, 0.25, 0.35, 0.5
    sine  30 deg amplitude, 20 s period, at the same throttles

Metrics per case (heading_metrics): 10-90 % rise time, overshoot past the target, settling time to
+-5 deg, steady-state error (mean of the last 5 s), turn radius (speed / yaw rate while turning);
for the sine the RMS error over its second period and the lag behind the target.

    --gains firmware|legacy  VEDC_SIM_HEADING_GAINS for the board (default firmware = app_config.h; legacy = before 13/11)
    --sysid                  no board: open-loop u steps straight onto the actuators, yaw rate per
                             throttle and u (Nomoto K) for the tuning (tools/heading_tune_model.py)
    --compare a.csv b.csv    overlay the samples of two runs in one PNG, no Gazebo
    --analyze x_samples.csv  the metrics CSV (and PNG) of a run again, no Gazebo

Run in WSL (source sim/env.sh first):
    python3 sim/tools/heading_step_test.py --gains firmware --out media/simulation/pid
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time
import tty
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
REPO = TOOLS.parent.parent
WORLD = "vedc_step"
H0 = 0.0                       # every case starts facing north
THROTTLES = (0.08, 0.15, 0.25, 0.35, 0.5)
STEPS = (30.0, -30.0, 90.0, -90.0, 180.0)
LEAD_S = 8.0                   # straight at the case's throttle before the step
SINE_AMP, SINE_PERIOD = 30.0, 20.0
SETTLE_BAND = 5.0


def wrap(deg: float) -> float:
    return math.fmod(deg + 540.0, 360.0) - 180.0


# ---- metrics (pure, no Gazebo) -----------------------------------------------------------------
def step_metrics(t: list[float], err: list[float], step: float, speed: list[float], rate: list[float]) -> dict:
    """t from the step (s), err = wrap(target - heading) deg, speed m/s, rate deg/s clockwise."""
    e0, a = abs(step), [abs(e) for e in err]
    i10 = next((i for i, v in enumerate(a) if v <= 0.9 * e0), None)
    i90 = next((i for i, v in enumerate(a) if v <= 0.1 * e0), None)
    out = {"rise_s": math.nan, "overshoot_deg": math.nan, "settle_s": math.nan, "sse_deg": math.nan,
           "radius_m": math.nan, "peak_rate_dps": max((abs(r) for r in rate), default=math.nan)}
    tail = [e for ti, e in zip(t, err) if ti >= t[-1] - 5.0]
    out["sse_deg"] = sum(tail) / len(tail)
    if i10 is None or i90 is None:
        return out
    out["rise_s"] = t[i90] - t[i10]
    turn = math.copysign(1.0, err[i10])          # the side the target is on while the boat turns
    out["overshoot_deg"] = max(0.0, max(-turn * e for e in err[i90:]))
    last_out = max((i for i, v in enumerate(a) if v > SETTLE_BAND), default=-1)
    out["settle_s"] = t[last_out + 1] if last_out + 1 < len(t) and a[-1] <= SETTLE_BAND else math.nan
    v = sum(speed[i10:i90 + 1]) / (i90 + 1 - i10)
    r = sum(abs(x) for x in rate[i10:i90 + 1]) / (i90 + 1 - i10)
    out["radius_m"] = v / math.radians(r) if r > 0.5 else math.nan
    return out


def sine_metrics(t: list[float], err: list[float], target_rel: list[float], heading_rel: list[float]) -> dict:
    """RMS error over the second period, and the lag (s) that best lines the heading up with the target."""
    idx = [i for i, ti in enumerate(t) if ti >= SINE_PERIOD]
    rms = math.sqrt(sum(err[i] ** 2 for i in idx) / len(idx)) if idx else math.nan
    dt = (t[-1] - t[0]) / max(1, len(t) - 1)
    best, lag = math.inf, math.nan
    for k in range(int(6.0 / dt)):
        pairs = [(target_rel[i - k], heading_rel[i]) for i in idx if i - k >= 0]
        cost = sum((a - b) ** 2 for a, b in pairs) / len(pairs)
        if cost < best:
            best, lag = cost, k * dt
    return {"rms_deg": rms, "lag_s": lag, "max_err_deg": max(abs(err[i]) for i in idx) if idx else math.nan}


# ---- Gazebo ----------------------------------------------------------------------------------------
def make_world(run: Path) -> Path:
    """vedc_wide.sdf with only the waves and the buoy, no sensor rendering; the buoy without ArduPilot."""
    shutil.rmtree(run / "models", ignore_errors=True)
    shutil.copytree(REPO / "sim/models/vedc_buoy", run / "models/vedc_buoy")
    sdf = run / "models/vedc_buoy/model.sdf"
    sdf.write_text(re.sub(r'<plugin name="ArduPilotPlugin".*?</plugin>', "", sdf.read_text(), flags=re.S))
    text = (REPO / "sim/worlds/vedc_wide.sdf").read_text()
    text = re.sub(r"<include><name>(marker|victim)_\w+</name>.*?</include>", "", text)
    text = re.sub(r'<plugin filename="gz-sim-sensors-system".*?</plugin>', "", text, flags=re.S)
    text = text.replace('<world name="vedc_wide">', f'<world name="{WORLD}">')
    (run / "worlds").mkdir(parents=True, exist_ok=True)
    world = run / "worlds" / f"{WORLD}.sdf"
    world.write_text(text)
    return world


class Sim:
    """Gazebo, the board (unless sysid), our own odometry subscriber and the Pi's NAV/HBT stream."""

    def __init__(self, args: argparse.Namespace) -> None:
        os.environ["GZ_PARTITION"] = args.partition     # before the first Node: a partition of our own
        run = args.run
        run.mkdir(parents=True, exist_ok=True)
        world = make_world(run)
        env = dict(os.environ, GZ_SIM_RESOURCE_PATH=f"{run}/models:{run}/worlds:" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""))
        self.procs: list[subprocess.Popen] = []
        self.procs.append(subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "1", str(world)], env=env, start_new_session=True,
                                           stdout=open(run / "gz.log", "w"), stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL))
        sys.path.insert(0, str(TOOLS))
        import stm32_board_sim as bs                  # gz modules: only now, with the partition set
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.transport13 import Node
        self.bs, self.Double = bs, Double
        self.node = Node()
        self.lock = threading.Lock()
        self.stamp = -1.0
        self.state = (0.0, 0.0, 0.0, 0.0, 0.0)          # heading, rate, speed, x, y
        self.rows: list[list] | None = None
        self.case = ("", 0.0, 0.0)
        self.target = H0
        self.node.subscribe(Odometry, "/model/vedc_buoy/odometry", self.on_odom)
        deadline = time.monotonic() + 120
        while self.stamp < 0:
            if time.monotonic() > deadline:
                raise SystemExit(f"no odometry from Gazebo: see {run}/gz.log")
            time.sleep(0.5)
        self.wait_sim(3.0)
        self.nav = ("STOP", 0.0, H0)
        self.fd = -1
        self.pub = None
        if args.sysid:
            self.pub = {name: self.node.advertise(bs.TOPIC.format("vedc_buoy", joint, kind), Double) for name, joint, kind in (
                ("left", "prop_rear_left_joint", "cmd_thrust"), ("right", "prop_rear_right_joint", "cmd_thrust"),
                ("front", "prop_front_joint", "cmd_thrust"), ("pod_left", "pod_rear_left_joint", "cmd_pos"),
                ("pod_right", "pod_rear_right_joint", "cmd_pos"), ("pod_front", "pod_front_joint", "cmd_pos"))}
            return
        pty = run / "stm32_pty"
        board_env = dict(env, VEDC_SIM_HEADING_GAINS=args.gains)
        self.procs.append(subprocess.Popen([sys.executable, "-u", str(TOOLS / "stm32_board_sim.py"), "--pty", str(pty), "--dir", str(run),
                                            "--home", "10.883438,106.796019", "--world", WORLD], env=board_env,
                                           stdout=open(run / "stm32_sim.log", "w"), stderr=subprocess.STDOUT))
        for _ in range(60):
            if pty.exists():
                break
            time.sleep(0.25)
        self.fd = os.open(pty, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        tty.setraw(self.fd)
        threading.Thread(target=self.pi_stream, daemon=True).start()

    def on_odom(self, msg) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        q, p, v = msg.pose.orientation, msg.pose.position, msg.twist.linear
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        heading = (90.0 - math.degrees(yaw)) % 360.0
        rate = -math.degrees(msg.twist.angular.z)       # clockwise seen from above, as the board's gyro
        with self.lock:
            self.stamp = stamp
            self.state = (heading, rate, math.hypot(v.x, v.y), p.x, p.y)
            if self.rows is not None:
                self.rows.append([stamp, time.monotonic(), self.target, heading, rate, math.hypot(v.x, v.y), p.x, p.y])

    def pi_stream(self) -> None:
        """The Pi's side of the UART: NAV every 50 ms (TTL 300 ms), HBT every 100 ms; the board's lines are drained."""
        seq = 0
        while True:
            seq = (seq + 1) & 0xFFFF
            mode, speed, heading = self.nav
            lines = [f"NAV,{seq},{mode},{speed:.2f},{heading % 360.0:.1f},300"]
            if seq % 2 == 0:
                lines.append(f"HBT,{seq},heading_step")
            try:
                os.write(self.fd, "".join(f"${b}*{self.bs.checksum(b)}\n" for b in lines).encode())
                while os.read(self.fd, 4096):
                    pass
            except (BlockingIOError, OSError):
                pass
            time.sleep(0.05)

    def now(self) -> float:
        with self.lock:
            return self.stamp

    def wait_sim(self, seconds: float, each=None) -> None:
        end = self.now() + seconds
        last, stalled = self.now(), time.monotonic()
        while (t := self.now()) < end:
            if t != last:
                last, stalled = t, time.monotonic()
            elif time.monotonic() - stalled > 20.0:
                raise RuntimeError("Gazebo's clock stopped for 20 s: gz sim gone?")
            if each:
                each(t)
            time.sleep(0.01)

    def actuate(self, u: float, throttle: float) -> None:
        """Sysid: what the board publishes for u at this throttle, straight onto the joints."""
        bs = self.bs
        left, right, steer = bs.heading_mix(u, throttle) if throttle > 0 else (0.0, 0.0, 0.0)
        pod = steer * bs.RUDDER_SPAN_US * bs.POD_RAD_PER_US
        for name, value in (("left", left * 15.0), ("right", right * 15.0), ("front", throttle * bs.FRONT_THRUST_GAIN * 15.0),
                            ("pod_left", pod), ("pod_right", pod), ("pod_front", -pod * bs.RUDDER_FRONT_GAIN)):
            msg = self.Double()
            msg.data = value
            self.pub[name].publish(msg)

    def reset_pose(self) -> None:
        """Motors off, back to the origin facing north, still."""
        if self.pub:
            self.wait_sim(2.0, lambda t: self.actuate(0.0, 0.0))
        else:
            self.nav = ("STOP", 0.0, H0)
            self.wait_sim(2.0)
        for _ in range(5):
            if self.bs.set_start(self.node, WORLD, 0.0, 0.0, H0):
                break
        self.wait_sim(2.0, (lambda t: self.actuate(0.0, 0.0)) if self.pub else None)

    def record(self, name: str, throttle: float, profile, seconds: float, step: float) -> list[list]:
        """AUTO north at throttle for LEAD_S, then the target from profile(t) for seconds; rows from t = 0."""
        self.reset_pose()
        speed = throttle * 1.0
        self.target = H0
        self.nav = ("AUTO", speed, H0)
        self.wait_sim(LEAD_S)
        t0 = self.now()
        with self.lock:
            self.rows = []
        wall0 = time.monotonic()

        def tick(t: float) -> None:
            self.target = (H0 + profile(t - t0)) % 360.0
            self.nav = ("AUTO", speed, self.target)
        self.wait_sim(seconds, tick)
        with self.lock:
            rows, self.rows = self.rows, None
        rtf = (rows[-1][0] - rows[0][0]) / max(1e-6, time.monotonic() - wall0)
        print(f"  {name}: {len(rows)} samples, real-time factor {rtf:.2f}", flush=True)
        return [[name, throttle, step, r[0] - t0, *r[1:], rtf] for r in rows]

    def sysid(self, throttle: float, u: float, seconds: float = 12.0) -> list[list]:
        self.reset_pose()
        self.wait_sim(LEAD_S, lambda t: self.actuate(0.0, throttle))
        t0 = self.now()
        with self.lock:
            self.rows = []
        self.wait_sim(seconds, lambda t: self.actuate(u, throttle))
        with self.lock:
            rows, self.rows = self.rows, None
        self.actuate(0.0, 0.0)
        return [[f"u{u:+.2f}", throttle, u, r[0] - t0, *r[1:], 1.0] for r in rows]

    def close(self) -> None:
        self.nav = ("STOP", 0.0, H0)
        time.sleep(0.3)
        for p in reversed(self.procs):
            try:
                os.killpg(p.pid, 15) if p is self.procs[0] else p.terminate()
            except ProcessLookupError:
                pass
        for p in self.procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, 9) if p is self.procs[0] else p.kill()


COLS = ["case", "throttle", "step", "t", "wall", "target", "heading", "rate", "speed", "x", "y", "rtf"]


def load(path: Path) -> dict[tuple, dict]:
    """Samples CSV -> {(case, throttle): columns as lists}, with err and the unwrapped headings added."""
    out: dict[tuple, dict] = {}
    with path.open() as f:
        for row in csv.DictReader(f):
            d = out.setdefault((row["case"], float(row["throttle"])), {k: [] for k in COLS[2:]})
            for k in COLS[2:]:
                d[k].append(float(row[k]))
    for d in out.values():
        d["err"] = [wrap(a - b) for a, b in zip(d["target"], d["heading"])]
        rel, prev = [], 0.0
        for h in d["heading"]:
            prev += wrap(h - H0 - prev)
            rel.append(prev)
        d["rel"] = rel
        d["target_rel"] = [r + e for r, e in zip(rel, d["err"])]
        # The odometry's yaw rate swings +-10 deg/s with the waves: the turn rate is the heading's slope over 1 s.
        t, n = d["t"], len(rel)
        w = max(1, round(0.5 / max(1e-3, (t[-1] - t[0]) / max(1, n - 1))))
        d["rate_s"] = [(rel[min(n - 1, i + w)] - rel[max(0, i - w)]) / (t[min(n - 1, i + w)] - t[max(0, i - w)]) for i in range(n)]
    return out


def metrics(samples: dict[tuple, dict]) -> list[dict]:
    rows = []
    for (case, throttle), d in samples.items():
        m = {"case": case, "throttle": throttle, "step": d["step"][0], "speed_mps": sum(d["speed"]) / len(d["speed"]),
             "rtf": sum(d["rtf"]) / len(d["rtf"])}
        if case == "sine":
            m.update(sine_metrics(d["t"], d["err"], d["target_rel"], d["rel"]))
        elif case.startswith("u"):
            m.update(nomoto(d["t"], d["rel"], d["speed"], d["step"][0]))
        else:
            m.update(step_metrics(d["t"], d["err"], d["step"][0], d["speed"], d["rate_s"]))
        rows.append(m)
    return rows


def nomoto(t: list[float], rel: list[float], speed: list[float], u: float) -> dict:
    """Open-loop u step: the steady yaw rate (heading slope over the last 8 s), K = rate / u. The rate
    settles within the first second in every run (heading slope per 1 s window), so T is not fitted
    from these: under the waves' noise it is "below ~0.5 s"."""
    i = next(k for k, ti in enumerate(t) if ti >= t[-1] - 8.0)
    rss = (rel[-1] - rel[i]) / (t[-1] - t[i])
    v = sum(speed[i:]) / len(speed[i:])
    return {"rate_ss_dps": rss, "K_dps_per_u": rss / u, "turn_speed_mps": v,
            "radius_m": v / math.radians(abs(rss)) if abs(rss) > 0.5 else math.nan}


def write_csv(path: Path, header: list[str], rows: list) -> None:
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([f"{x:.4f}" if isinstance(x, float) else x for x in (r if isinstance(r, list) else [r.get(h, "") for h in header])])


def plot(runs: dict[str, dict], png: Path, title: str) -> None:
    """Heading vs time (relative to the start), target dashed; one panel per case and throttle."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    keys = {k for s in runs.values() for k in s}
    cases = sorted({k[0] for k in keys}, key=lambda c: (c == "sine", 0.0 if c == "sine" else abs(float(c)), 0.0 if c == "sine" else -float(c)))
    throttles = sorted({k[1] for k in keys})
    fig, axes = plt.subplots(len(throttles), len(cases), figsize=(3.2 * len(cases), 2.4 * len(throttles)), squeeze=False, sharex="col")
    colors = ["#888888", "#1f6fd1", "#d1501f"]
    for i, th in enumerate(throttles):
        for j, case in enumerate(cases):
            ax = axes[i][j]
            for n, (label, s) in enumerate(runs.items()):
                d = s.get((case, th))
                if d is None:
                    continue
                if n == len(runs) - 1:
                    ax.plot(d["t"], d["target_rel"], "k--", lw=0.9, label="target")
                ax.plot(d["t"], d["rel"], color=colors[n % 3] if len(runs) > 1 else colors[1], lw=1.3, label=label)
            ax.grid(alpha=0.3)
            if i == 0:
                ax.set_title(case if case == "sine" else f"step {case}")
            if j == 0:
                ax.set_ylabel(f"throttle {th:.2f}\nheading (deg)")
            if i == len(throttles) - 1:
                ax.set_xlabel("t (s)")
    axes[0][0].legend(fontsize=7, loc="best")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(png, dpi=90)
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gains", default="firmware", help="VEDC_SIM_HEADING_GAINS for the board: firmware (app_config.h) or legacy (before 13/11)")
    parser.add_argument("--out", type=Path, default=REPO / "media/simulation/pid", help="where the CSV and PNG go")
    parser.add_argument("--run", type=Path, default=Path.home() / "vedc_sim/heading_step", help="Gazebo run directory")
    parser.add_argument("--partition", default="vedc_heading_step", help="GZ_PARTITION, apart from other runs")
    parser.add_argument("--throttles", default=",".join(map(str, THROTTLES)))
    parser.add_argument("--steps", default=",".join(f"{s:g}" for s in STEPS))
    parser.add_argument("--no-sine", action="store_true")
    parser.add_argument("--sysid", action="store_true", help="open-loop u steps instead of the board")
    parser.add_argument("--compare", nargs="+", type=Path, help="samples CSVs to overlay (label = file name), no Gazebo")
    parser.add_argument("--analyze", type=Path, help="a samples CSV: write its metrics (and PNG) again, no Gazebo")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    if args.compare:
        runs = {p.stem.replace("_samples", ""): load(p) for p in args.compare}
        png = args.out / ("_vs_".join(runs) + ".png")
        plot(runs, png, " vs ".join(runs))
        print(png)
        return 0

    if args.analyze:
        report(args.analyze, args.analyze.stem.replace("_samples", ""), args.out)
        return 0

    throttles = [float(x) for x in args.throttles.split(",")]
    label = "sysid" if args.sysid else args.gains
    sim = Sim(args)
    rows: list[list] = []
    try:
        for th in throttles:
            if args.sysid:
                for u in (0.3, 0.6, 1.0, -0.6):
                    print(f"throttle {th:.2f} u {u:+.2f}", flush=True)
                    rows += sim.sysid(th, u)
                continue
            for step in (float(x) for x in args.steps.split(",")):
                print(f"throttle {th:.2f} step {step:+g}", flush=True)
                rows += sim.record(f"{step:+g}", th, lambda t, s=step: s, 30.0 if abs(step) > 30 else 20.0, step)
            if not args.no_sine:
                print(f"throttle {th:.2f} sine", flush=True)
                rows += sim.record("sine", th, lambda t: SINE_AMP * math.sin(2 * math.pi * t / SINE_PERIOD), 2 * SINE_PERIOD, SINE_AMP)
    finally:
        sim.close()
    samples = args.out / f"{label}_samples.csv"
    write_csv(samples, COLS, rows)
    report(samples, label, args.out)
    return 0


def report(samples: Path, label: str, out: Path) -> None:
    data = load(samples)
    m = metrics(data)
    header = list(dict.fromkeys(k for r in m for k in r))
    write_csv(out / f"{label}_metrics.csv", header, m)
    if label != "sysid":
        plot({label: data}, out / f"{label}.png", f"heading step response, gains {label}")
    for r in m:
        print("  ".join(f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}" for k, v in r.items()))


if __name__ == "__main__":
    sys.exit(main())
