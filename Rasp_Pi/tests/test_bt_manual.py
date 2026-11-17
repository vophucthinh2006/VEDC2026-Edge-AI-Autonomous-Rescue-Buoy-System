import unittest

from modules.bt_manual import ManualState, apply, silenced


def feed(text, state=None, start=0.0, step=1.0):
    state = state or ManualState()
    now = start
    for ch in text:
        state = apply(state, ch, now)
        now += step
    return state


class ManualTest(unittest.TestCase):
    def test_drive_commands_ignored_in_auto(self):
        s = feed("9FLUW")
        self.assertFalse(s.active)
        self.assertEqual((s.rear, s.front, s.steer, s.lights, s.sweep), (0.0, 0.0, 0.0, 0, False))
        self.assertEqual(s.level, 9)

    def test_forward_astern_stop(self):
        s = feed("V9F")
        self.assertEqual((s.rear, s.front), (1.0, 1.0))
        s = feed("B", s, 10)
        self.assertEqual((s.rear, s.front), (0.0, -1.0))
        s = feed("3", s, 11)
        self.assertAlmostEqual(s.front, -3 / 9)
        s = feed("S", s, 12)
        self.assertEqual((s.rear, s.front), (0.0, 0.0))

    def test_steer_notches_clamped_and_kept_on_stop(self):
        s = feed("V" + "R" * 7)
        self.assertEqual(s.steer, 1.0)
        s = feed("LLS", s, 10)
        self.assertEqual(s.steer, 0.6)

    def test_repeated_notch_within_gap_counts_once(self):
        s = feed("V")
        s = feed("LLL", s, 1.0, 0.05)    # 0.1 s apart: inside NOTCH_GAP_S
        self.assertEqual(s.steer, -0.2)

    def test_lights_sweep_and_back_to_auto(self):
        s = feed("VUW2F")
        self.assertEqual((s.lights, s.sweep), (3, True))
        s = feed("v", s, 10)
        self.assertFalse(s.active)
        self.assertEqual((s.rear, s.front, s.steer, s.sweep), (0.0, 0.0, 0.0, False))
        self.assertEqual(feed("u", s, 11).lights, 3)   # the dashboard owns the lights in AUTO

    def test_unknown_characters_ignored(self):
        s = feed("V5F")
        self.assertEqual(feed("GIHJXxq\n", s, 10).motion, "F")

    def test_silence_stops_motors_but_stays_manual(self):
        s = feed("V9F")                     # last character at t=2
        self.assertEqual(silenced(s, 2.5, 1.0).motion, "F")
        quiet = silenced(s, 3.5, 1.0)
        self.assertTrue(quiet.active)
        self.assertEqual((quiet.rear, quiet.front), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
