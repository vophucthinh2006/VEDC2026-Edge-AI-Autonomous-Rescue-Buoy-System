import unittest

from modules.heading_bias import HeadingBias


def run(bias: HeadingBias, seconds: float, yaw: float, course: float, speed: float = 0.5, fix: int = 3, start: float = 0.0,
        turn_dps: float = 0.0, backing: bool = False) -> float:
    t = start
    while t < start + seconds:
        bias.update(t, (yaw + turn_dps * (t - start)) % 360.0, course, speed, fix, backing)
        t += 0.1
    return bias.bias_deg


class HeadingBiasTest(unittest.TestCase):
    def test_learns_a_steady_offset_in_three_time_constants(self):
        self.assertAlmostEqual(run(HeadingBias(), 65.0, yaw=130.0, course=100.0), 30.0, delta=2.0)

    def test_wraps_through_north(self):
        self.assertAlmostEqual(run(HeadingBias(), 65.0, yaw=350.0, course=10.0), -20.0, delta=2.0)

    def test_learns_nothing_slow_without_a_fix_backing_or_turning(self):
        for kwargs in ({"speed": 0.1}, {"fix": 2}, {"backing": True}, {"turn_dps": 10.0}):
            self.assertEqual(run(HeadingBias(), 30.0, yaw=130.0, course=100.0, **kwargs), 0.0, kwargs)

    def test_clamped(self):
        self.assertEqual(run(HeadingBias({"max_deg": 60.0}), 200.0, yaw=190.0, course=100.0), 60.0)

    def test_holds_after_a_turn_until_straight_again(self):
        b = HeadingBias()
        learned = run(b, 60.0, yaw=130.0, course=100.0)
        run(b, 1.5, yaw=130.0, course=100.0, start=60.0, turn_dps=20.0)
        self.assertAlmostEqual(b.bias_deg, learned, delta=0.1)   # one step learnt before the turn shows

    def test_disabled(self):
        self.assertEqual(run(HeadingBias({"enabled": False}), 65.0, yaw=130.0, course=100.0), 0.0)


if __name__ == "__main__":
    unittest.main()
