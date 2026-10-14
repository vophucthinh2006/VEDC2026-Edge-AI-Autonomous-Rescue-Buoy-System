#!/usr/bin/env python3
"""Record what the Pi sees during a simulator test: frames of the viewer's MJPEG stream
(Rasp_Pi/modules/viewer.py, rescue_main.py --viewer PORT) saved as JPEG files.

    python3 sim/tools/capture_view.py --port 8091 --out shots/ --every 2 --duration 300

Waits for the viewer to come up, then keeps one frame every --every seconds; files are named by
the second they were taken, from the start of the recording. Stops after --duration seconds or
when the viewer goes away.
"""
from __future__ import annotations

import argparse
import time
import urllib.request
from pathlib import Path


def frames(url: str):
    """JPEG images of a multipart MJPEG stream, as bytes."""
    with urllib.request.urlopen(url, timeout=10) as stream:
        buffer = b""
        while True:
            chunk = stream.read(16384)
            if not chunk:
                return
            buffer += chunk
            start = buffer.find(b"\xff\xd8")
            end = buffer.find(b"\xff\xd9", start + 2)
            while start >= 0 and end >= 0:
                yield buffer[start:end + 2]
                buffer = buffer[end + 2:]
                start = buffer.find(b"\xff\xd8")
                end = buffer.find(b"\xff\xd9", start + 2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=8091)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--every", type=float, default=2.0, help="seconds between saved frames")
    parser.add_argument("--duration", type=float, default=300.0)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    url = f"http://127.0.0.1:{args.port}/stream"
    start = time.monotonic()
    saved, last = 0, -1e9
    while time.monotonic() - start < args.duration:
        try:
            for image in frames(url):
                now = time.monotonic()
                if now - start >= args.duration:
                    break
                if now - last >= args.every:
                    last = now
                    (args.out / f"view_{now - start:06.1f}s.jpg").write_bytes(image)
                    saved += 1
            else:
                if saved:
                    break      # the viewer went away after it had been up: the test is over
        except OSError:
            if saved:
                break
            time.sleep(1.0)    # not up yet
    print(f"saved {saved} frames to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
