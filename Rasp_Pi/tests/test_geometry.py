import unittest

from utils.geometry import signed_angle_deg, wrap_deg


class GeometryTest(unittest.TestCase):
    def test_angles(self) -> None:
        self.assertEqual(wrap_deg(-10), 350)
        self.assertEqual(signed_angle_deg(350), -10)
