"""Spoken messages on the boat's loudspeaker: WAV files in sounds/, played with aplay on audio.device
(the Pi's 3.5 mm jack by default). Playing never blocks the caller; a message asked for while another
plays waits its turn, and the same message is not queued twice.

audio.periodic_sound (sounds/rescue_boat.wav, "phao cứu hộ") is played every audio.periodic_s
seconds, the first time at start, so people in the water know what is coming; 0 turns it off. It
waits while supplies_confirmed is queued or playing: that one is for the person at the boat.

The WAVs are made on a Windows PC with the vi-VN voice "Microsoft An": tools/make_sounds.ps1.
"""
from __future__ import annotations

import logging
import queue
import subprocess
import threading
from pathlib import Path
from time import monotonic

PRIORITY = "supplies_confirmed"      # the periodic message never goes in while this one waits or plays


class Announcer(threading.Thread):
    def __init__(self, audio: dict, stop_event: threading.Event) -> None:
        super().__init__(name="announcer", daemon=True)
        self.device = str(audio.get("device", "plughw:0,0"))
        self.folder = Path(audio.get("sounds_dir", "sounds"))
        self.periodic_sound = str(audio.get("periodic_sound", "rescue_boat"))
        self.periodic_s = float(audio.get("periodic_s", 100.0))
        self.stop = stop_event
        self.log = logging.getLogger(__name__)
        self._queue: queue.Queue[str] = queue.Queue()
        self._pending: set[str] = set()
        self._playing: str | None = None
        self._periodic_at: float | None = None     # when the periodic message is next due; None: at once

    def say(self, name: str) -> None:
        """Queue sounds/<name>.wav."""
        if name not in self._pending:
            self._pending.add(name)
            self._queue.put(name)

    def periodic(self, now: float) -> bool:
        """Queue the periodic message if it is due and supplies_confirmed is neither waiting nor
        playing (then it is tried again next time). True if it was queued."""
        if self.periodic_s <= 0 or (self._periodic_at is not None and now < self._periodic_at):
            return False
        if PRIORITY in self._pending or self._playing == PRIORITY:
            return False
        self.say(self.periodic_sound)
        self._periodic_at = now + self.periodic_s
        return True

    def run(self) -> None:
        while not self.stop.is_set():
            self.periodic(monotonic())
            try:
                name = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            self._pending.discard(name)
            path = self.folder / f"{name}.wav"
            if not path.exists():
                self.log.error("no sound %s", path)
                continue
            self._playing = name
            try:
                subprocess.run(["aplay", "-q", "-D", self.device, str(path)], timeout=30, check=False)
            except (OSError, subprocess.TimeoutExpired) as exc:
                self.log.error("cannot play %s on %s: %s", path, self.device, exc)
            finally:
                self._playing = None
