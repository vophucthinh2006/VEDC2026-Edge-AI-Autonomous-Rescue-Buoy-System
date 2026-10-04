"""LiDAR filtering, simple clustering, and camera-to-LiDAR target association."""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sin, tan

from modules.lidar_ld14 import LidarPoint
from utils.geometry import median, signed_angle_deg


@dataclass(frozen=True)
class Obstacle:
    bearing_body_deg: float  # positive right/clockwise from bow
    distance_m: float
    width_m: float


def cluster_scan(points: tuple[LidarPoint, ...], min_distance_m: float = 0.15, max_distance_m: float = 8.0, gap_m: float = 0.25) -> tuple[Obstacle, ...]:
    """Cluster adjacent polar points. Angle 0 is assumed aligned to the bow after installation calibration."""
    valid = sorted((p for p in points if min_distance_m <= p.distance_m <= max_distance_m and p.intensity > 0), key=lambda p: p.angle_deg)
    groups: list[list[LidarPoint]] = []
    for point in valid:
        if not groups:
            groups.append([point])
            continue
        previous = groups[-1][-1]
        angular_gap = abs(signed_angle_deg(point.angle_deg - previous.angle_deg))
        chord = (point.distance_m ** 2 + previous.distance_m ** 2 - 2 * point.distance_m * previous.distance_m * cos(radians(angular_gap))) ** 0.5
        if angular_gap < 4.0 and chord <= gap_m:
            groups[-1].append(point)
        else:
            groups.append([point])
    obstacles = []
    for group in groups:
        if len(group) < 2:
            continue
        angle = median(p.angle_deg for p in group)
        distance = median(p.distance_m for p in group)
        if angle is None or distance is None:
            continue
        first, last = group[0], group[-1]
        angular_width = abs(signed_angle_deg(last.angle_deg - first.angle_deg))
        width = max(0.05, 2 * distance * sin(radians(angular_width / 2)))
        obstacles.append(Obstacle(signed_angle_deg(angle), distance, width))
    return tuple(obstacles)


def ray_obstacles(rays: list[tuple[float, float]], min_distance_m: float = 0.15, max_distance_m: float = 8.0,
                  beam_deg: float = 1.0, neighbour_m: float = 0.3) -> tuple[Obstacle, ...]:
    """One small obstacle per LiDAR ray, for the autopilot's avoidance: a wall keeps its shape,
    which a cluster summarised as one circle does not. rays: (bearing from the bow in degrees,
    clockwise; range in m). A return with no neighbour at a similar range is dropped: a wave crest
    or noise, not an object."""
    valid = sorted((signed_angle_deg(b), d) for b, d in rays if min_distance_m <= d <= max_distance_m)
    kept = []
    for index, (bearing, distance) in enumerate(valid):
        for other_bearing, other_distance in (valid[index - 1], valid[(index + 1) % len(valid)]):
            if 0.0 < abs(signed_angle_deg(other_bearing - bearing)) <= 2.0 * beam_deg and abs(other_distance - distance) <= neighbour_m:
                kept.append(Obstacle(bearing, distance, max(0.05, 2.0 * distance * sin(radians(beam_deg / 2.0)))))
                break
    return tuple(kept)


def estimate_distance_bbox(box_height_frac: float, vertical_fov_deg: float, person_height_m: float) -> float:
    """Pinhole range from the person box height (fraction of frame height). Rough: assumes a standing person."""
    span = 2.0 * tan(radians(vertical_fov_deg) / 2.0) * max(box_height_frac, 1e-3)
    return person_height_m / span


def box_bottom_elevation_deg(box_ymax_frac: float, vertical_fov_deg: float, pitch_deg: float) -> float:
    """Elevation of the bottom edge of a box above the horizon, in degrees. box_ymax_frac: 0 is the
    top of the frame, 1 the bottom. The camera looks along the bow, so the boat's pitch (bow up
    positive) is added back.

    The camera sits close to the water: anybody in the water has the bottom of their box below the
    horizon. Above it, the person stands on something (a roof) and shows their whole height."""
    return (0.5 - box_ymax_frac) * vertical_fov_deg + pitch_deg


class ScanPattern:
    """Search sweep of the camera servo: step to an angle, let it settle, look at a few frames, step on.

    Stepping, not sweeping: a frame taken while the servo moves is blurred, and the SG90 does not
    report its angle, so the bearing of anything seen in it would be a guess. Frames inside
    settle_s of a step are to be dropped (usable() is False). The dwell is counted in frames, so it
    follows whatever rate the detector manages on the Pi.
    """

    def __init__(self, angles_deg: list[float], settle_s: float, dwell_frames: int) -> None:
        self.angles = [float(a) for a in angles_deg]
        self.settle_s, self.dwell_frames = float(settle_s), max(1, int(dwell_frames))
        self._index, self._frames, self._moved_at = 0, 0, 0.0

    @property
    def target_deg(self) -> float:
        return self.angles[self._index]

    def start(self, now: float, pan_deg: float) -> float:
        """Begin (or resume) at the listed angle nearest to where the camera points now."""
        self._index = min(range(len(self.angles)), key=lambda i: abs(self.angles[i] - pan_deg))
        self._frames, self._moved_at = 0, now
        return self.target_deg

    def usable(self, now: float) -> bool:
        return now - self._moved_at >= self.settle_s

    def frame_done(self, now: float) -> float:
        """One usable frame looked at, nothing in it. Returns the angle to hold or move to."""
        self._frames += 1
        if self._frames >= self.dwell_frames:
            self._index = (self._index + 1) % len(self.angles)
            self._frames, self._moved_at = 0, now
        return self.target_deg


class SweepPattern:
    """Search sweep of the camera servo, the smooth way: back and forth between -limit and +limit at
    a constant rate. Every frame is used; the angle at capture is the commanded one, which a servo
    many times faster than the sweep follows closely. Same interface as ScanPattern.

    The slower the sweep, the longer a person stays in view, and the longer the camera is away
    from each side: at rate_deg_s a full cycle takes 4 * limit / rate seconds, during which the
    boat keeps moving.
    """

    def __init__(self, limit_deg: float, rate_deg_s: float) -> None:
        self.limit, self.rate = abs(float(limit_deg)), abs(float(rate_deg_s))
        self._angle, self._direction, self._at = 0.0, -1.0, 0.0

    @property
    def target_deg(self) -> float:
        return self._angle

    def start(self, now: float, pan_deg: float) -> float:
        """Begin (or resume) from where the camera points, heading for the nearer end first."""
        self._angle = max(-self.limit, min(self.limit, float(pan_deg)))
        self._direction = 1.0 if self._angle > 0 else -1.0
        self._at = now
        return self._angle

    def usable(self, _now: float) -> bool:
        return True

    def frame_done(self, now: float) -> float:
        """Advance to `now` and return the angle to command."""
        travel = self.rate * max(0.0, now - self._at)
        self._at = now
        while travel > 0.0:
            room = self.limit - self._angle if self._direction > 0 else self._angle + self.limit
            if travel < room:
                self._angle += self._direction * travel
                break
            self._angle = self.limit * self._direction
            self._direction, travel = -self._direction, travel - room
        return self._angle


def make_scan(scan: dict | None) -> ScanPattern | SweepPattern | None:
    """The search pattern of camera.scan, or None when it is off."""
    if not scan or not scan.get("enabled"):
        return None
    if str(scan.get("mode", "step")).lower() == "sweep":
        return SweepPattern(scan["sweep_limit_deg"], scan["sweep_rate_deg_s"])
    return ScanPattern(scan["angles_deg"], scan["settle_s"], scan["dwell_frames"])


def track_pan(pan_deg: float, bearing_cam_deg: float, gain: float, deadband_deg: float, limit_deg: float) -> float:
    """One P step that turns the camera servo toward the person; positive is right."""
    if abs(bearing_cam_deg) <= deadband_deg:
        return pan_deg
    return max(-limit_deg, min(limit_deg, pan_deg + gain * bearing_cam_deg))


def associate_person(bearing_body_deg: float, obstacles: tuple[Obstacle, ...], half_angle_deg: float, min_distance_m: float, max_distance_m: float) -> Obstacle | None:
    candidates = [o for o in obstacles if abs(signed_angle_deg(o.bearing_body_deg - bearing_body_deg)) <= half_angle_deg and min_distance_m <= o.distance_m <= max_distance_m]
    return min(candidates, key=lambda obstacle: obstacle.distance_m) if candidates else None
