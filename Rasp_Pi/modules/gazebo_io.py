"""Simulator stand-ins for the Pi's camera, camera servo and LiDAR: Gazebo topics instead of hardware.

Only imported when the configuration asks for the simulator. Needs the Gazebo Python bindings
(gz.transport13, gz.msgs10), which come with the ROS 2 Jazzy vendor packages: run under sim/env.sh.
"""
from __future__ import annotations

import math
import threading

import numpy as np
from gz.msgs10.double_pb2 import Double
from gz.msgs10.image_pb2 import Image
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.transport13 import Node

from modules.perception import ray_obstacles
from modules.state_store import StateStore


class GazeboCapture:
    """The slice of cv2.VideoCapture that VisionWorker uses, fed by a Gazebo camera topic."""

    def __init__(self, topic: str) -> None:
        self._node = Node()
        self._frame: np.ndarray | None = None
        self._fresh = threading.Event()
        self._lock = threading.Lock()
        self._opened = bool(self._node.subscribe(Image, topic, self._on_image))

    def _on_image(self, msg: Image) -> None:
        rgb = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        with self._lock:
            self._frame = rgb[:, :, ::-1].copy()   # OpenCV order, like a real capture
        self._fresh.set()

    def isOpened(self) -> bool:  # noqa: N802 - cv2 name
        return self._opened

    def set(self, _prop: int, _value: float) -> bool:
        return False

    def read(self) -> tuple[bool, np.ndarray | None]:
        """Blocks for the next frame, like a camera; (False, None) after one second without any."""
        if not self._fresh.wait(timeout=1.0):
            return False, None
        self._fresh.clear()
        with self._lock:
            return True, self._frame

    def release(self) -> None:
        self._opened = False


class GazeboPan:
    """Camera pan servo of the simulated buoy. Angle from the bow in degrees, positive right."""

    def __init__(self, topic: str) -> None:
        self._node = Node()
        self._pub = self._node.advertise(topic, Double)

    def __call__(self, pan_deg: float) -> None:
        msg = Double()
        msg.data = math.radians(pan_deg)
        self._pub.publish(msg)


class GazeboLidar:
    """The simulated buoy's LiDAR: every scan becomes the obstacle list of the state store."""

    def __init__(self, topic: str, state: StateStore, min_m: float, max_m: float) -> None:
        self._node = Node()
        self._state, self._min_m, self._max_m = state, min_m, max_m
        self.opened = bool(self._node.subscribe(LaserScan, topic, self._on_scan))

    def _on_scan(self, msg: LaserScan) -> None:
        # Gazebo angles grow counter-clockwise seen from above; the Pi counts clockwise from the bow.
        rays = [(-math.degrees(msg.angle_min + index * msg.angle_step), distance)
                for index, distance in enumerate(msg.ranges) if math.isfinite(distance)]
        self._state.update_obstacles(ray_obstacles(rays, self._min_m, self._max_m, beam_deg=math.degrees(msg.angle_step)))
