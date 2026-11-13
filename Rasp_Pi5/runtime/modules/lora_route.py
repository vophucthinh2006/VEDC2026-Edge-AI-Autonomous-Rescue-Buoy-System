"""The route an admin sent from the dashboard, as the STM32F407 passes it on: $RTE lines.

    $RTE,seq,rev,first,total,loop,lat;lon,lat;lon,...

Up to 7 points a line, sent again every few seconds until the Pi's $HBT carries `rev` as its 4th
field (docs/lora-route-downlink.md). RouteAssembler puts the lines of one revision together and
hands the route over once, when the last point is in.
"""
from __future__ import annotations

from dataclasses import dataclass

MAX_POINTS = 300


@dataclass(frozen=True)
class LoraRoute:
    rev: int
    loop: bool
    points: tuple[tuple[float, float], ...]


class RouteAssembler:
    def __init__(self) -> None:
        self._rev = 0
        self._total = 0
        self._loop = False
        self._points: dict[int, tuple[float, float]] = {}
        self.done_rev = 0           # last revision handed over

    def feed(self, fields: tuple[str, ...]) -> LoraRoute | None:
        """The fields after "RTE" (seq first). Returns the route the first time it is complete.
        Raises ValueError on a malformed line."""
        if len(fields) < 5:
            raise ValueError("RTE needs seq, rev, first, total, loop")
        rev, first, total, loop = int(fields[1]), int(fields[2]), int(fields[3]), fields[4]
        if not 1 <= rev <= 0xFFFF or not 0 <= total <= MAX_POINTS or not 0 <= first <= total or loop not in ("0", "1"):
            raise ValueError(f"RTE out of range: rev {rev}, first {first}, total {total}, loop {loop}")
        points = []
        for pair in fields[5:]:
            lat, lon = (float(value) for value in pair.split(";"))
            if abs(lat) > 90.0 or abs(lon) > 180.0:
                raise ValueError(f"RTE point out of range: {pair}")
            points.append((lat, lon))
        if first + len(points) > total:
            raise ValueError("RTE has more points than its total")
        if rev == self.done_rev:
            return None             # the board repeats it until $HBT confirms
        if rev != self._rev or total != self._total:
            self._rev, self._total, self._loop, self._points = rev, total, loop == "1", {}
        for index, point in enumerate(points, start=first):
            self._points[index] = point
        if len(self._points) < self._total:
            return None
        self.done_rev = rev
        return LoraRoute(rev, self._loop, tuple(self._points[index] for index in range(self._total)))
