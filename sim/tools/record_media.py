#!/usr/bin/env python3
"""Pictures and a video of a simulator run, for reports. Run under sim/env.sh.

record   keeps the frames of Gazebo camera topics and of the Pi's viewer (Rasp_Pi/modules/viewer.py,
         --viewer PORT) as JPEG files named by their Unix time, one folder per stream:

    python3 record_media.py record --out rec/ --duration 150 \\
        --topic bank=/vedc_central/cam_bank:15 --topic buoy=/vedc_buoy/camera:3 --viewer 8090:15

compose  picks the moments of the run from the STM32 stand-in's log (sim/tools/f407_board_sim.py
         --log): the first VICTIM_FOUND and the boat coming to rest next to the person; writes
         numbered stills and a side-by-side video (left: --left stream, right: the Pi's viewer):

    python3 record_media.py compose --rec rec/ --log f407.csv --victim 14,-1 --img-out img/ \\
        --video-out run.mp4 --length 30
"""
from __future__ import annotations

import argparse
import bisect
import csv
import math
import sys
import threading
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


# ----------------------------------------------------------------------------- record

def record(args: argparse.Namespace) -> int:
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node

    args.out.mkdir(parents=True, exist_ok=True)
    node = Node()
    counts: dict[str, int] = {}
    stop = time.time() + args.duration

    def keeper(name: str, fps: float):
        folder = args.out / name
        folder.mkdir(exist_ok=True)
        last = [0.0]
        counts[name] = 0

        def on_image(msg: Image) -> None:
            now = time.time()
            if now - last[0] < 1.0 / fps or now > stop:
                return
            last[0] = now
            rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            cv2.imwrite(str(folder / f"{now:.3f}.jpg"), rgb[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 92])
            counts[name] += 1
        return on_image

    for spec in args.topic:
        name, rest = spec.split("=", 1)
        topic, _, fps = rest.rpartition(":")
        if not node.subscribe(Image, topic, keeper(name, float(fps))):
            print(f"cannot subscribe to {topic}", file=sys.stderr)
            return 1

    def viewer(port: int, fps: float) -> None:
        folder = args.out / "viewer"
        folder.mkdir(exist_ok=True)
        counts["viewer"] = 0
        last = 0.0
        while time.time() < stop:
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/stream", timeout=10) as stream:
                    buffer = b""
                    while time.time() < stop:
                        chunk = stream.read(16384)
                        if not chunk:
                            break
                        buffer += chunk
                        start = buffer.find(b"\xff\xd8")
                        end = buffer.find(b"\xff\xd9", start + 2)
                        while start >= 0 and end >= 0:
                            image, buffer = buffer[start:end + 2], buffer[end + 2:]
                            now = time.time()
                            if now - last >= 1.0 / fps:
                                last = now
                                (folder / f"{now:.3f}.jpg").write_bytes(image)
                                counts["viewer"] += 1
                            start = buffer.find(b"\xff\xd8")
                            end = buffer.find(b"\xff\xd9", start + 2)
            except OSError:
                time.sleep(1.0)        # not up yet, or restarted

    threads = []
    if args.viewer:
        port, _, fps = args.viewer.partition(":")
        threads.append(threading.Thread(target=viewer, args=(int(port), float(fps or 10)), daemon=True))
        threads[-1].start()
    while time.time() < stop:
        time.sleep(0.5)
    for thread in threads:
        thread.join(timeout=12)
    print("recorded: " + ", ".join(f"{name} {count}" for name, count in counts.items()))
    return 0


# ----------------------------------------------------------------------------- compose

class Stream:
    """The frames of one recorded stream, looked up by time."""

    def __init__(self, folder: Path) -> None:
        self.files = sorted(folder.glob("*.jpg"), key=lambda p: float(p.stem)) if folder.is_dir() else []
        self.times = [float(p.stem) for p in self.files]

    def __bool__(self) -> bool:
        return bool(self.files)

    def at(self, t: float) -> np.ndarray | None:
        if not self.files:
            return None
        i = bisect.bisect_left(self.times, t)
        if i == len(self.times) or (i > 0 and t - self.times[i - 1] < self.times[i] - t):
            i -= 1
        return cv2.imread(str(self.files[i]))


def read_log(path: Path) -> list[dict]:
    with path.open() as f:
        return [{k: (v if k == "mode" else float(v)) for k, v in row.items()} for row in csv.DictReader(f)]


def moments(rows: list[dict], victim: tuple[float, float]) -> dict[str, float]:
    """Unix times of the run's moments, from the stand-in's log."""
    def dist(row: dict) -> float:
        return math.hypot(row["x"] - victim[0], row["y"] - victim[1])
    moving = next((r["t"] for r in rows if r["speed"] > 0.15), rows[0]["t"])
    found = next((r["t"] for r in rows if r["victims"] > 0), None)
    closest = min(rows, key=dist)
    rest = None
    if found is not None:
        rest = next((r["t"] for r in rows if r["t"] > found and r["speed"] < 0.08 and dist(r) < 4.0), None)
    return {"start": rows[0]["t"], "moving": moving, "found": found if found is not None else closest["t"] - 8.0,
            "rest": rest if rest is not None else closest["t"], "closest": closest["t"], "end": rows[-1]["t"],
            "closest_m": dist(closest)}


def caption(image: np.ndarray, lines: list[tuple[str, int, bool]], top: bool = True) -> np.ndarray:
    """Draws text (Vietnamese needs a real font, not cv2.putText) on a dark band."""
    from PIL import Image, ImageDraw, ImageFont
    height = sum(size + 8 for _, size, _ in lines) + 10
    band = np.full((height, image.shape[1], 3), 18, np.uint8)
    pil = Image.fromarray(band[:, :, ::-1])
    draw = ImageDraw.Draw(pil)
    y = 6
    for text, size, bold in lines:
        draw.text((14, y), text, font=ImageFont.truetype(FONT_BOLD if bold else FONT, size), fill=(235, 235, 235))
        y += size + 8
    band = np.asarray(pil)[:, :, ::-1]
    return np.vstack([band, image] if top else [image, band])


def fit_height(image: np.ndarray, height: int) -> np.ndarray:
    return cv2.resize(image, (round(image.shape[1] * height / image.shape[0]), height), interpolation=cv2.INTER_AREA)


def overview_with_track(image: np.ndarray, rows: list[dict], until: float, victim: tuple[float, float]) -> np.ndarray:
    """The top-down camera of vedc_central.sdf (x 38, y 0, 58 m up, 0.62 rad over 640 px), turned
    so that east is right and north up, with the boat's track drawn on it."""
    turned = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    scale = 640 / (2 * 58 * math.tan(0.31))          # pixels per metre
    cx, cy = turned.shape[1] / 2, turned.shape[0] / 2

    def px(x: float, y: float) -> tuple[int, int]:
        return round(cx + (x - 38.0) * scale), round(cy - y * scale)
    points = [px(r["x"], r["y"]) for r in rows if r["t"] <= until]
    for a, b in zip(points, points[1:]):
        cv2.line(turned, a, b, (0, 200, 255), 3, cv2.LINE_AA)
    if points:
        cv2.circle(turned, points[0], 7, (0, 255, 0), -1, cv2.LINE_AA)
        cv2.circle(turned, points[-1], 7, (255, 255, 255), -1, cv2.LINE_AA)
    cv2.circle(turned, px(*victim), 12, (0, 0, 255), 3, cv2.LINE_AA)
    return turned


def compose(args: argparse.Namespace) -> int:
    rows = read_log(args.log)
    victim = tuple(float(v) for v in args.victim.split(","))
    m = moments(rows, victim)
    print("moments: " + ", ".join(f"{k} {v - m['start']:.1f}s" if k != "closest_m" else f"{k} {v:.2f}"
                                    for k, v in m.items() if k not in ("start",)))
    streams = {p.name: Stream(p) for p in args.rec.iterdir() if p.is_dir()}
    title = "Mô phỏng Gazebo — kiến trúc Raspberry Pi 4 + STM32F407"

    def at_t(name: str, t: float) -> np.ndarray | None:
        s = streams.get(name)
        return s.at(t) if s else None

    def row_at(t: float) -> dict:
        return min(rows, key=lambda r: abs(r["t"] - t))

    def status(t: float) -> str:
        r = row_at(t)
        d = math.hypot(r["x"] - victim[0], r["y"] - victim[1])
        phase = ("tìm kiếm theo lộ trình" if t < m["found"] else "phát hiện nạn nhân, tiếp cận" if t < m["rest"]
                 else "dừng cạnh nạn nhân, giữ vị trí")
        return f"t = {t - m['start']:5.1f} s   |   {phase}   |   cách nạn nhân {d:4.1f} m   |   {r['speed']:.2f} m/s"

    # ---- stills
    if args.img_out:
        args.img_out.mkdir(parents=True, exist_ok=True)
        shots = [
            ("01_toan_canh_tu_tren.jpg", "overview", m["rest"] + 2, "Toàn cảnh từ trên xuống: kênh ngập lũ miền Trung, quỹ đạo phao (vàng), nạn nhân (đỏ)"),
            ("02_xuat_phat.jpg", args.left, m["moving"], "Phao xuất phát ở đầu kênh, Pi gửi $NAV cho STM32F407"),
            ("03_tim_kiem.jpg", args.left, (m["moving"] + m["found"]) / 2, "Đi theo lộ trình tìm kiếm, camera quét ±60°"),
            ("04_camera_phao.jpg", "buoy", m["found"], "Ảnh camera trên phao lúc thấy nạn nhân dưới nước"),
            ("05_pi_phat_hien.jpg", "viewer", m["found"] + 0.5, "Màn hình Pi: YOLOv8n + MoveNet phát hiện người, LiDAR LD14, bản đồ quyết định"),
            ("06_tiep_can.jpg", args.left, (m["found"] + m["rest"]) / 2, "Pi chuyển sang FOLLOW, F407 giữ hướng bằng PID và lái tới nạn nhân"),
            ("07_can_canh_tiep_can.jpg", "close", (m["found"] + m["rest"]) / 2 + 1, "Cận cảnh: phao tiến về phía nạn nhân"),
            ("08_can_canh_dung.jpg", "close", m["rest"] + 3, "Cận cảnh: phao dừng ở khoảng cách an toàn cạnh nạn nhân"),
            ("09_pi_giu_vi_tri.jpg", "viewer", m["rest"] + 3, "Màn hình Pi lúc phao giữ vị trí cạnh nạn nhân"),
            ("10_ket_thuc.jpg", args.left, m["rest"] + 6, "Kết thúc tiếp cận: phao đứng yên, đã gửi VICTIM_FOUND về bờ"),
        ]
        saved = 0
        for name, stream, t, text in shots:
            frame = at_t(stream, t)
            if frame is None:
                print(f"{name}: no {stream} frame", file=sys.stderr)
                continue
            if stream == "overview":
                frame = overview_with_track(frame, rows, t, victim)
            frame = caption(frame, [(text, 20, True), (status(t), 16, False)], top=False)
            cv2.imwrite(str(args.img_out / name), frame, [cv2.IMWRITE_JPEG_QUALITY, 93])
            saved += 1
        print(f"stills: {saved} in {args.img_out}")

    # ---- video
    if args.video_out:
        # From a little before the person is found to a few seconds after the boat comes to rest.
        start = max(m["start"], min(m["found"] - args.lead, m["rest"] + args.tail - args.length))
        start = min(start, max(m["start"], m["end"] - args.length))
        fps, height = args.fps, 540
        size = None
        writer = None
        for k in range(int(args.length * fps)):
            t = start + k / fps
            left, right = at_t(args.left, t), at_t("viewer", t)
            if left is None or right is None:
                continue
            panel = np.hstack([fit_height(left, height), fit_height(right, height)])
            frame = caption(panel, [(title, 24, True), (status(t), 18, False)])
            if size is None:
                size = (frame.shape[1] // 2 * 2, frame.shape[0] // 2 * 2)
                writer = cv2.VideoWriter(str(args.video_out), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
            writer.write(cv2.resize(frame, size))
        if writer is None:
            print("no video: missing frames", file=sys.stderr)
            return 1
        writer.release()
        print(f"video: {args.video_out}, {args.length:.0f} s from t = {start - m['start']:.1f} s, {size[0]} x {size[1]}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    rec = sub.add_parser("record")
    rec.add_argument("--out", type=Path, required=True)
    rec.add_argument("--duration", type=float, default=150.0)
    rec.add_argument("--topic", action="append", default=[], help="name=/gz/topic:fps")
    rec.add_argument("--viewer", help="port[:fps] of the Pi's viewer")
    com = sub.add_parser("compose")
    com.add_argument("--rec", type=Path, required=True)
    com.add_argument("--log", type=Path, required=True)
    com.add_argument("--victim", default="14,-1", help="x,y of the person in Gazebo metres")
    com.add_argument("--left", default="bank", help="stream on the left of the video")
    com.add_argument("--img-out", type=Path)
    com.add_argument("--video-out", type=Path)
    com.add_argument("--length", type=float, default=30.0)
    com.add_argument("--lead", type=float, default=2.5, help="seconds of video before the person is found")
    com.add_argument("--tail", type=float, default=5.0, help="seconds of video after the boat comes to rest")
    com.add_argument("--fps", type=float, default=15.0)
    args = parser.parse_args()
    return record(args) if args.command == "record" else compose(args)


if __name__ == "__main__":
    sys.exit(main())
