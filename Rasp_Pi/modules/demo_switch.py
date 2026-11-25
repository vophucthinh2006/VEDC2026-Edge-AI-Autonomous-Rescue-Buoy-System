"""The judges' demo switch: people are placed from the assumed position, whatever the GPS says.

On while the dashboard's button is on (asked of the cloud every few seconds; the Pi needs internet for it) or while
a file exists on the Pi (touch / rm, for a room without internet). With the cloud out of reach its last answer holds.
"""
from __future__ import annotations

import json
import logging
import threading
import urllib.request
from pathlib import Path

POLL_S = 5.0


def ask(url: str) -> bool:
    # A browser-like name: the plain urllib one is refused by some front ends
    request = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(request, timeout=4) as reply:
        return json.load(reply)["on"] is True


class DemoSwitch:
    def __init__(self, flag: str | None, url: str | None, fetch=ask) -> None:
        self._flag = Path(flag).expanduser() if flag else None
        self._url, self._fetch = url, fetch
        self._cloud = False
        self.log = logging.getLogger(__name__)
        if url:
            threading.Thread(target=self._run, name="demo-switch", daemon=True).start()

    @property
    def on(self) -> bool:
        return self._cloud or (self._flag is not None and self._flag.exists())

    def poll(self) -> None:
        try:
            self._cloud = self._fetch(self._url)
        except Exception as exc:  # no internet, a bad answer: keep the last one
            self.log.debug("demo switch: %s", exc)

    def _run(self) -> None:
        idle = threading.Event()
        while True:
            self.poll()
            idle.wait(POLL_S)
