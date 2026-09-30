import unittest
from time import monotonic

from modules.navigation import BapfNavigator
from modules.perception import Obstacle
from modules.state_store import GpsState, ImuState, Snapshot, SysState


class NavigationTest(unittest.TestCase):
    def setUp(self) -> None:
        self.vehicle = {"boat_radius_m": 0.35, "safety_margin_m": 0.4, "influence_distance_m": 3.0, "target_standoff_m": 1.5, "approach_speed_mps": 0.2, "max_speed_mps": 0.5, "arrive_radius_m": 1.0, "stop_distance_m": 0.8, "slow_distance_m": 2.0}
        self.planner = {"attractive_gain": 1.0, "repulsive_gain": 0.75, "bias_gain": 0.22}

    def test_close_obstacle_stops(self) -> None:
        now = monotonic()
        snap = Snapshot(ImuState(yaw_deg=0, imu_ok=True, timestamp=now), GpsState(lat_deg=10, lon_deg=106, fix=3, timestamp=now), SysState(timestamp=now), (Obstacle(0, 0.8, 0.2),), now, (10.001, 106), None)
        command = BapfNavigator(self.vehicle, self.planner).plan(snap)
        self.assertEqual(command.mode, "STOP")


def test_bbox_distance_and_pan_tracking():
    from modules.perception import estimate_distance_bbox, track_pan
    # Half the frame height at 70 deg vertical FOV, 1.7 m person.
    assert abs(estimate_distance_bbox(0.5, 70.0, 1.7) - 2.43) < 0.02
    assert estimate_distance_bbox(0.9, 70.0, 1.7) < estimate_distance_bbox(0.3, 70.0, 1.7)
    assert track_pan(10.0, 1.0, 0.5, 2.0, 80.0) == 10.0          # inside the deadband
    assert track_pan(10.0, 20.0, 0.5, 2.0, 80.0) == 20.0         # follows the person to the right
    assert track_pan(75.0, 40.0, 0.5, 2.0, 80.0) == 80.0         # limited
