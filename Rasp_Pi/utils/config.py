"""Settings of the Pi runtimes: config/settings.yaml, with overlay files laid on top of it.

The two boats share settings.yaml; what differs between them lives in config/boat_pixhawk.yaml and
config/boat_stm32.yaml (run.sh adds the right one), and the simulator adds config/sim*.yaml.
"""
from __future__ import annotations

from pathlib import Path

import yaml


def merge(base: dict, overlay: dict) -> dict:
    """Overlay values win; nested sections are merged key by key."""
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value
    return base


def load_config(path: Path, overlays: list[Path] | tuple[Path, ...] = ()) -> dict:
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    for overlay in overlays:
        merge(config, yaml.safe_load(overlay.read_text(encoding="utf-8")) or {})   # an overlay may hold comments only
    return config
