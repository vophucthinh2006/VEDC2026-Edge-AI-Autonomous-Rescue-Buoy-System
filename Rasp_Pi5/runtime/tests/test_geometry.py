import unittest

from utils.geometry import heading_tenths, signed_angle_deg, wrap_deg


class GeometryTest(unittest.TestCase):
    def test_angles(self) -> None:
        self.assertEqual(wrap_deg(-10), 350)
        self.assertEqual(signed_angle_deg(350), -10)

    def test_heading_tenths_never_reaches_360(self) -> None:
        # The STM32F407 drops a NAV packet whose heading is 360.0 or more.
        self.assertEqual(heading_tenths(359.96), "0.0")
        self.assertEqual(heading_tenths(359.94), "359.9")
        self.assertEqual(heading_tenths(-0.04), "0.0")
        self.assertEqual(heading_tenths(-10.0), "350.0")
        self.assertEqual(heading_tenths(123.42), "123.4")
        for tenth in range(-3600, 7200):
            value = float(heading_tenths(tenth / 10.0 + 0.04))
            self.assertTrue(0.0 <= value < 360.0, value)
