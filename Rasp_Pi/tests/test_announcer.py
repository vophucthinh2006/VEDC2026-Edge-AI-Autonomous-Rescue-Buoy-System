import threading
import unittest

from modules.announcer import Announcer


class PeriodicTest(unittest.TestCase):
    """audio.periodic_sound every audio.periodic_s, the first at start, never over supplies_confirmed."""

    def setUp(self):
        self.announcer = Announcer({"periodic_sound": "rescue_boat", "periodic_s": 100}, threading.Event())

    def queued(self):
        names = []
        while not self.announcer._queue.empty():
            names.append(self.announcer._queue.get_nowait())
        self.announcer._pending.clear()     # as the player thread does when it takes them
        return names

    def test_first_at_start_then_every_period(self):
        self.assertTrue(self.announcer.periodic(5.0))
        self.assertEqual(self.queued(), ["rescue_boat"])
        self.assertFalse(self.announcer.periodic(60.0))
        self.assertTrue(self.announcer.periodic(105.0))
        self.assertEqual(self.queued(), ["rescue_boat"])

    def test_waits_while_supplies_confirmed_is_queued_or_playing(self):
        self.announcer.say("supplies_confirmed")
        self.assertFalse(self.announcer.periodic(0.0))
        self.assertEqual(self.queued(), ["supplies_confirmed"])
        self.announcer._playing = "supplies_confirmed"
        self.assertFalse(self.announcer.periodic(1.0))
        self.announcer._playing = None
        self.assertTrue(self.announcer.periodic(2.0))     # straight after, not a whole period later
        self.assertEqual(self.queued(), ["rescue_boat"])

    def test_zero_turns_it_off(self):
        quiet = Announcer({"periodic_s": 0}, threading.Event())
        self.assertFalse(quiet.periodic(0.0))
        self.assertTrue(quiet._queue.empty())


if __name__ == "__main__":
    unittest.main()
