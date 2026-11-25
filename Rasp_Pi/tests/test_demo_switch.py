import unittest

from modules.demo_switch import DemoSwitch


class DemoSwitchTest(unittest.TestCase):
    def test_the_last_answer_holds_when_the_cloud_is_out_of_reach(self):
        answers = [True, OSError("no internet"), False, KeyError("on")]

        def fetch(url):
            answer = answers.pop(0)
            if isinstance(answer, Exception):
                raise answer
            return answer
        switch = DemoSwitch(None, None, fetch)
        self.assertFalse(switch.on)
        for expected in (True, True, False, False):
            switch.poll()
            self.assertEqual(switch.on, expected)


if __name__ == "__main__":
    unittest.main()
