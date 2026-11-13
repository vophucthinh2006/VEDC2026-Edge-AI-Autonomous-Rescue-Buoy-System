"""One log file per run of a Pi program, in logging.directory, the newest logging.keep_runs kept.

main.py logged to its terminal only: once the ssh window of run_all.bat was closed, nothing said
why the boat had stopped. boat.bat deploy leaves logs/ on the Pi alone.
"""
from __future__ import annotations

from datetime import datetime
from pathlib import Path


def run_log_path(directory: Path, prefix: str, keep_runs: int, now: datetime | None = None) -> Path:
    """The file for this run, <prefix>_YYYYmmdd_HHMMSS.log; older runs beyond keep_runs - 1 are
    deleted first, oldest first, so that this one makes keep_runs."""
    directory.mkdir(parents=True, exist_ok=True)
    runs = sorted(directory.glob(f"{prefix}_*.log"))
    for old in runs[:max(0, len(runs) - max(0, keep_runs - 1))]:
        old.unlink(missing_ok=True)
    stamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    path = directory / f"{prefix}_{stamp}.log"
    count = 2
    while path.exists():   # two runs within the same second
        path = directory / f"{prefix}_{stamp}_{count}.log"
        count += 1
    return path
