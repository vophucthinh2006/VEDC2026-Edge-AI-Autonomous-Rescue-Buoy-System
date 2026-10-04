import unittest

try:
    import numpy as np
    from modules.viewer import BAR, CAMERA_H, CAMERA_W, MAP, render
except ImportError:          # OpenCV or numpy missing: the viewer is optional
    render = None

from modules.perception import Obstacle
from modules.state_store import GpsState, HumanTarget, ImuState, Snapshot, SysState, VehicleState


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


if __name__ == "__main__":
    unittest.main()
