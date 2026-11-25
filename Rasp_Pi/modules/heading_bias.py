"""The compass's error, learned from the GPS course while the boat runs straight.

The BNO055's heading is off by a bias that changes with the heading and the time (the motors' and
battery's field, the iron about, its own re-learning after power-up): 2026-11-22 it read 79 to 114 deg
off over four headings. A low-pass filter only smooths that; it does not remove it. On a straight run
the GPS course is the true heading, so: bias = low-passed (yaw - course), learned only then and held
otherwise. True heading = yaw - bias; the F407's PID steers on the IMU's yaw, so it gets heading + bias.
"""
from __future__ import annotations

from utils.geometry import signed_angle_deg


class HeadingBias:
    def __init__(self, settings: dict | None = None) -> None:
        s = settings or {}
        self.enabled = bool(s.get("enabled", True))
        self.tau_s = float(s.get("tau_s", 20.0))
        self.min_speed_mps = float(s.get("min_speed_mps", 0.3))
        self.max_deg = float(s.get("max_deg", 60.0))
        self.max_rate_dps = float(s.get("max_rate_dps", 3.0))
        self.straight_s = float(s.get("straight_s", 2.0))
        self.bias_deg = 0.0
        self._last: tuple[float, float] | None = None   # (time, yaw)
        self._turned_at = float("-inf")

    def update(self, now: float, yaw_deg: float, course_deg: float, speed_mps: float, fix: int, backing: bool = False) -> float:
        """One main loop: learns when the run is straight and fast enough; returns the bias."""
        if self._last is not None and now > self._last[0]:
            rate = signed_angle_deg(yaw_deg - self._last[1]) / (now - self._last[0])
            if abs(rate) > self.max_rate_dps:
                self._turned_at = now
            if (self.enabled and not backing and fix >= 3 and speed_mps >= self.min_speed_mps
                    and now - self._turned_at >= self.straight_s):
                error = signed_angle_deg(yaw_deg - course_deg)
                step = min(1.0, (now - self._last[0]) / self.tau_s)
                self.bias_deg += step * signed_angle_deg(error - self.bias_deg)
                self.bias_deg = max(-self.max_deg, min(self.max_deg, self.bias_deg))
        self._last = (now, yaw_deg)
        return self.bias_deg
