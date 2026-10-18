"""Spoken messages on the boat's loudspeaker: WAV files in sounds/, played with aplay on audio.device
(the Pi's 3.5 mm jack by default). Playing never blocks the caller; a message asked for while another
plays waits its turn, and the same message is not queued twice.

The WAVs are made on a Windows PC with the vi-VN voice "Microsoft An": tools/make_sounds.ps1.
"""
from __future__ import annotations

import logging
import queue
import subprocess
import threading
from pathlib import Path


class Announcer(threading.Thread):
    def __init__(self, audio: dict, stop_event: threading.Event) -> None:
        super().__init__(name="announcer", daemon=True)
        self.device = str(audio.get("device", "plughw:0,0"))
        self.folder = Path(audio.get("sounds_dir", "sounds"))
        self.stop = stop_event
        self.log = logging.getLogger(__name__)
        self._queue: queue.Queue[str] = queue.Queue()
        self._pending: set[str] = set()

    def say(self, name: str) -> None:
        """Queue sounds/<name>.wav."""
        if name not in self._pending:
            self._pending.add(name)
            self._queue.put(name)

    def run(self) -> None:
        while not self.stop.is_set():
            try:
                name = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            self._pending.discard(name)
            path = self.folder / f"{name}.wav"
            if not path.exists():
                self.log.error("no sound %s", path)
                continue
            try:
                subprocess.run(["aplay", "-q", "-D", self.device, str(path)], timeout=30, check=False)
            except (OSError, subprocess.TimeoutExpired) as exc:
                self.log.error("cannot play %s on %s: %s", path, self.device, exc)
