"""One line a second of what the Pi knows, as CSV, for reading a run back afterwards
(rescue_main.py --log). A new file per run: logs/run_<date>_<time>.csv."""
from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
from time import monotonic

from modules.state_store import Snapshot

COLUMNS = ("time", "t_s", "mode", "armed", "phase", "fix", "lat", "lon", "speed_mps", "yaw_deg", "roll_deg", "pitch_deg", "mission_item",
           "wp_dist_m", "people_found", "person_bearing_deg", "person_range_m", "person_confidence", "obstacles", "nearest_obstacle_m",
           "lidar_age_s", "board", "battery_v", "current_a", "estop", "alarm", "lora_ok")


def row(snap: Snapshot, now: float, started: float, phase: str, found: int, board: object | None, board_online: bool) -> dict:
    """board: modules.helper_board.BoardState, or None without a helper board."""
    target = snap.human_target if snap.human_target is not None and now - snap.human_target.timestamp <= 1.0 else None
    nearest = min((o.distance_m for o in snap.obstacles), default=None)
    out = {
        "time": datetime.now().isoformat(timespec="milliseconds"), "t_s": f"{now - started:.1f}", "mode": snap.vehicle.mode,
        "armed": int(snap.vehicle.armed), "phase": phase, "fix": snap.gps.fix, "lat": f"{snap.gps.lat_deg:.7f}", "lon": f"{snap.gps.lon_deg:.7f}",
        "speed_mps": f"{snap.gps.speed_mps:.2f}", "yaw_deg": f"{snap.imu.yaw_deg:.1f}", "roll_deg": f"{snap.imu.roll_deg:.1f}",
        "pitch_deg": f"{snap.imu.pitch_deg:.1f}", "mission_item": snap.vehicle.mission_seq,
        "wp_dist_m": f"{snap.vehicle.wp_dist_m:.1f}" if snap.vehicle.wp_dist_m >= 0.0 else "", "people_found": found,
        "person_bearing_deg": f"{target.bearing_body_deg:.1f}" if target else "", "person_range_m": f"{target.distance_m:.2f}" if target else "",
        "person_confidence": f"{target.confidence:.2f}" if target else "", "obstacles": len(snap.obstacles),
        "nearest_obstacle_m": f"{nearest:.2f}" if nearest is not None else "",
        "lidar_age_s": f"{now - snap.scan_timestamp:.1f}" if snap.scan_timestamp > 0.0 else "",
        "board": "" if board is None else int(board_online),
    }
    if board is not None and board_online:
        out.update(battery_v=f"{board.battery_v:.2f}", current_a=f"{board.current_a:.2f}", estop=int(board.estop), alarm=int(board.alarm),
                   lora_ok=int(board.lora_ok))
    return out


class RunLog:
    def __init__(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / f"run_{datetime.now():%Y%m%d_%H%M%S}.csv"
        self._file = self.path.open("w", newline="", encoding="utf-8", buffering=1)     # line buffered: a power cut loses one line
        self._writer = csv.DictWriter(self._file, fieldnames=COLUMNS, restval="")
        self._writer.writeheader()
        self._started = monotonic()

    def write(self, snap: Snapshot, now: float, phase: str, found: int, board: object | None = None, board_online: bool = False) -> None:
        self._writer.writerow(row(snap, now, self._started, phase, found, board, board_online))

    def close(self) -> None:
        self._file.close()
