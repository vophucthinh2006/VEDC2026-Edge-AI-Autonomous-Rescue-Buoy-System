"""run_pi5.py: the Pi 5 overlays on top of boat_stm32.yaml, and they load."""
import runpy
import sys
import unittest
from pathlib import Path
from unittest import mock

from utils.config import load_config

HERE = Path(__file__).resolve().parents[1]
RUN_PATH = runpy.run_path   # the launcher's own call to it is replaced below


def launched(*args: str) -> list[str]:
    with mock.patch.object(sys, "argv", ["run_pi5.py", *args]), mock.patch("runpy.run_path"), mock.patch("os.chdir"):
        RUN_PATH(str(HERE / "run_pi5.py"), run_name="__main__")
        return sys.argv[1:]


class Pi5LauncherTest(unittest.TestCase):
    def test_system(self):
        self.assertEqual(launched("--viewer"), ["--overlay", "config/boat_stm32.yaml", "--overlay", "config/pi5_system.yaml", "--viewer"])

    def test_buoy_test(self):
        self.assertEqual(launched("--buoy-test"), ["--overlay", "config/boat_stm32.yaml", "--overlay", "config/pi5_buoy_test.yaml", "--no-lidar"])

    def test_overlays_load(self):
        for name in ("pi5_system", "pi5_buoy_test"):
            config = load_config(HERE / "config/settings.yaml", [HERE / "config/boat_stm32.yaml", HERE / f"config/{name}.yaml"])
            self.assertIn("by-path", config["serial"]["stm32_port"])
            self.assertIn("rapoo", str(config["camera"]["device"]))
            self.assertIn("model_path", config["camera"])   # settings.yaml's camera block survives the overlay


if __name__ == "__main__":
    unittest.main()
