import tempfile
import unittest
from datetime import datetime, timedelta
from pathlib import Path

from utils.log_files import run_log_path


class RunLogPathTest(unittest.TestCase):
    def test_a_file_per_run_the_newest_kept(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp) / "logs"            # made when missing
            start = datetime(2026, 10, 21, 8, 0, 0)
            made = []
            for minute in range(6):
                path = run_log_path(directory, "main", 3, start + timedelta(minutes=minute))
                path.write_text("run\n", encoding="utf-8")
                made.append(path.name)
            self.assertEqual(made[0], "main_20261021_080000.log")
            self.assertEqual(sorted(p.name for p in directory.glob("main_*.log")), made[-3:])

    def test_two_runs_in_the_same_second_get_two_files(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            now = datetime(2026, 10, 21, 8, 0, 0)
            first = run_log_path(Path(tmp), "main", 30, now)
            first.write_text("a", encoding="utf-8")
            second = run_log_path(Path(tmp), "main", 30, now)
            self.assertNotEqual(first, second)
            self.assertEqual(second.name, "main_20261021_080000_2.log")

    def test_other_files_are_left_alone(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            other = Path(tmp) / "lidar_capture.bin"
            other.write_bytes(b"x")
            for second in range(4):
                run_log_path(Path(tmp), "main", 1, datetime(2026, 1, 1, 0, 0, second)).write_text("", encoding="utf-8")
            self.assertTrue(other.exists())
            self.assertEqual(len(list(Path(tmp).glob("main_*.log"))), 1)


if __name__ == "__main__":
    unittest.main()
