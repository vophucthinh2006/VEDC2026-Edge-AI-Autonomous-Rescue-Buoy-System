import unittest

try:
    import numpy as np
    from modules.viewer import BAR, CAMERA_H, CAMERA_W, MAP, render
    from modules.world_map import WORLD_H, WORLD_W, WorldMemory, draw_world, steering_words
except ImportError:          # OpenCV or numpy missing: the viewer is optional
    render = None

from modules.perception import Obstacle
from modules.state_store import GpsState, HumanTarget, ImuState, NavState, Snapshot, SysState, VehicleState


@unittest.skipIf(render is None, "OpenCV not installed")
class ViewerRenderTest(unittest.TestCase):
    def _snap(self, target=None):
        return Snapshot(ImuState(yaw_deg=90.0, imu_ok=True), GpsState(lat_deg=10.88, lon_deg=106.79, fix=3), SysState(),
                        (Obstacle(0.0, 3.0, 0.05), Obstacle(1.0, 3.0, 0.05)), 0.0, None, target, VehicleState("AUTO", True, 1, 0.0, 2, 12.0))

    def test_renders_without_a_camera_frame(self):
        image = render(self._snap(), None, [801] * 72, 8.0, [], 60.0, "search")
        self.assertEqual(image.shape, (CAMERA_H + BAR, CAMERA_W + MAP, 3))

    def test_renders_a_frame_with_a_detection_sectors_and_a_keep_out(self):
        view = {"frame": np.zeros((480, 640, 3), np.uint8), "box": (0.3, 0.4, 0.7, 0.6), "confidence": 0.71,
                "distance_m": 4.2, "pan_deg": -60.0, "scanning": False, "age_s": 0.1}
        sector_cm = [300] * 3 + [801] * 69
        image = render(self._snap(HumanTarget(-55.0, 4.2, 0.71, 0.0)), view, sector_cm, 8.0, [(120.0, 5.0, 0.4)], 60.0, "approach")
        self.assertEqual(image.shape, (CAMERA_H + BAR, CAMERA_W + MAP, 3))
        self.assertGreater(int(image[:CAMERA_H, :CAMERA_W].max()), 0)        # the box and labels were drawn on the black frame


@unittest.skipIf(render is None, "OpenCV not installed")
class DecisionMapTest(unittest.TestCase):
    def _snap(self, steer=None, lat=10.88, wp_bearing=90.0, obstacles=()):
        from time import monotonic
        nav = NavState(steer, 90.0, ((1, 10.88, 106.7902), (2, 10.8802, 106.7904)), monotonic())
        return Snapshot(ImuState(yaw_deg=90.0, imu_ok=True), GpsState(lat_deg=lat, lon_deg=106.79, fix=3), SysState(), tuple(obstacles),
                        0.0, None, None, VehicleState("AUTO", True, 1, 0.0, 2, 22.0, wp_bearing), (), None, nav)

    def test_the_map_is_as_wide_as_the_two_panels_above_it(self):
        self.assertEqual(WORLD_W, CAMERA_W + MAP)
        world = draw_world(self._snap(), WorldMemory(), None, [801] * 72, 8.0, None, 60.0)
        self.assertEqual(world.shape, (WORLD_H, WORLD_W, 3))
        whole = render(self._snap(), None, [801] * 72, 8.0, [], 60.0, "search", world)
        self.assertEqual(whole.shape, (CAMERA_H + WORLD_H + BAR, WORLD_W, 3))

    def test_draws_the_people_the_route_and_what_the_lidar_remembers(self):
        memory = WorldMemory()
        decision = {"phase": "approach", "lines": ["APPROACH: person 6.0 m away at 090 deg"], "standoff_m": 2.0, "group_radius_m": 2.5,
                    "people": [(10.88, 106.79006, "target"), (10.88003, 106.79004, "attended"), (10.87998, 106.79008, "seen")],
                    "goto": (10.88, 106.79004)}
        obstacles = [Obstacle(-20.0, 4.0, 0.05), Obstacle(-19.0, 4.0, 0.05)]
        sector_cm = [801] * 72
        sector_cm[68] = 400
        for step in range(3):                                        # the boat moves: a trail, and returns kept where they were
            world = draw_world(self._snap(steer=120.0, lat=10.88 + step * 1e-5, obstacles=obstacles), memory, decision, sector_cm, 8.0,
                               {"pan_deg": -20.0, "scanning": False}, 60.0)
        self.assertEqual(len(memory.trail), 3)
        self.assertGreaterEqual(len(memory.hits), 2)
        self.assertGreater(int((world[:, :WORLD_W - 400] > 200).sum()), 50)       # something bright was drawn on the map

    def test_no_position_yet_is_said_not_drawn(self):
        snap = Snapshot(ImuState(), GpsState(fix=0), SysState(), (), 0.0, None, None)
        memory = WorldMemory()
        self.assertEqual(draw_world(snap, memory, None, [801] * 72, 8.0, None, 60.0).shape, (WORLD_H, WORLD_W, 3))
        self.assertIsNone(memory.origin)

    def test_the_words_say_whether_the_boat_is_going_round_something(self):
        from time import monotonic
        straight = [text for text, _colour in steering_words(self._snap(steer=92.0), monotonic())]
        self.assertTrue(any("straight for the waypoint" in text for text in straight))
        round_it = [text for text, _colour in steering_words(self._snap(steer=130.0, obstacles=[Obstacle(5.0, 3.2, 0.05)]), monotonic())]
        self.assertTrue(any("40 deg RIGHT of the waypoint" in text for text in round_it))
        self.assertTrue(any("going round: 3.2 m" in text for text in round_it))
        self.assertIn("no steering report", steering_words(self._snap(), monotonic())[0][0])


if __name__ == "__main__":
    unittest.main()
