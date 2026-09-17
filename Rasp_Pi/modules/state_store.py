"""Thread-safe latest-value state. Never retain mutable scan arrays without copying."""
from __future__ import annotations

from dataclasses import dataclass, field
from threading import Lock
from time import monotonic


@dataclass(frozen=True)
class ImuState:
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    yaw_deg: float = 0.0
    imu_ok: bool = False
    overturned: bool = False
    timestamp: float = 0.0


@dataclass(frozen=True)
class GpsState:
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    fix: int = 0
    hdop: float = 99.0
    speed_mps: float = 0.0
    course_deg: float = 0.0
    timestamp: float = 0.0


@dataclass(frozen=True)
class SysState:
    battery_v: float = 0.0
    motor_fault: bool = False
    estop: bool = False
    link_ok: bool = False
    timestamp: float = 0.0


@dataclass(frozen=True)
class HumanTarget:
    bearing_body_deg: float
    distance_m: float
    confidence: float
    timestamp: float


@dataclass(frozen=True)
class Snapshot:
    imu: ImuState
    gps: GpsState
    system: SysState
    obstacles: tuple[object, ...]
    scan_timestamp: float
    waypoint: tuple[float, float] | None
    human_target: HumanTarget | None


class StateStore:
    def __init__(self) -> None:
        self._lock = Lock()
        self._imu = ImuState()
        self._gps = GpsState()
        self._system = SysState()
        self._obstacles: tuple[object, ...] = ()
        self._scan_timestamp = 0.0
        self._waypoint: tuple[float, float] | None = None
        self._human_target: HumanTarget | None = None

    def update_imu(self, pitch: float, roll: float, yaw: float, imu_ok: bool, overturned: bool) -> None:
        with self._lock:
            self._imu = ImuState(pitch, roll, yaw % 360.0, imu_ok, overturned, monotonic())

    def update_gps(self, lat: float, lon: float, fix: int, hdop: float, speed: float, course: float) -> None:
        with self._lock:
            self._gps = GpsState(lat, lon, fix, hdop, speed, course % 360.0, monotonic())

    def update_system(self, battery: float, motor_fault: bool, estop: bool, link_ok: bool) -> None:
        with self._lock:
            self._system = SysState(battery, motor_fault, estop, link_ok, monotonic())

    def update_obstacles(self, obstacles: tuple[object, ...], scan_timestamp: float | None = None) -> None:
        with self._lock:
            self._obstacles = obstacles
            self._scan_timestamp = scan_timestamp if scan_timestamp is not None else monotonic()

    def set_waypoint(self, lat: float, lon: float) -> None:
        with self._lock:
            self._waypoint = (lat, lon)

    def update_human_target(self, target: HumanTarget | None) -> None:
        with self._lock:
            self._human_target = target

    def snapshot(self) -> Snapshot:
        with self._lock:
            return Snapshot(self._imu, self._gps, self._system, self._obstacles, self._scan_timestamp, self._waypoint, self._human_target)
