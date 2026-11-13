"""Drawing helpers shared by the live viewer (modules/viewer.py) and its decision map
(modules/world_map.py): one palette, cards with a title strip, status chips, and text.

Text is drawn with a TrueType font through Pillow when both are there (DejaVu Sans comes with
Raspberry Pi OS and Ubuntu), each string rendered once and kept; without them it falls back to
OpenCV's built-in font, so the viewer never needs more than OpenCV to run.
"""
from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np

# BGR. A dark slate background, cards a step lighter, and one colour per meaning.
BG, CARD, CARD_EDGE, STRIP = (20, 17, 15), (32, 27, 24), (62, 55, 50), (40, 34, 30)
WHITE, TEXT, MUTED, DIM, GRID = (240, 240, 240), (215, 215, 215), (150, 145, 140), (95, 90, 85), (52, 47, 43)
GREEN, YELLOW, RED, BLUE, ORANGE, ROUTE, CYAN = ((110, 210, 90), (70, 205, 245), (85, 85, 235), (235, 160, 70),
                                                 (50, 150, 250), (230, 170, 90), (215, 200, 80))
GUTTER = 12
STRIP_H = 30

_FONTS = {
    False: ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "C:/Windows/Fonts/DejaVuSans.ttf", "C:/Windows/Fonts/segoeui.ttf"),
    True: ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "C:/Windows/Fonts/DejaVuSans-Bold.ttf", "C:/Windows/Fonts/segoeuib.ttf"),
}


@lru_cache(maxsize=None)
def _font(size: int, bold: bool):
    try:
        from PIL import ImageFont
    except ImportError:
        return None
    for path in _FONTS[bold]:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return None


@lru_cache(maxsize=1024)
def _sprite(text: str, size: int, bold: bool) -> tuple[np.ndarray, int] | None:
    """The text as an alpha mask, and how far its top sits above the baseline; None without Pillow."""
    font = _font(size, bold)
    if font is None:
        return None
    from PIL import Image, ImageDraw
    left, top, right, bottom = font.getbbox(text)
    mask = Image.new("L", (max(1, right - left), max(1, bottom - top)), 0)
    ImageDraw.Draw(mask).text((-left, -top), text, font=font, fill=255)
    ascent, _descent = font.getmetrics()
    return np.asarray(mask, np.float32) / 255.0, ascent - top


def _ascii(text: str) -> str:
    """OpenCV's own font has no degree sign, middle dot or ellipsis."""
    return text.replace("°", " deg").replace("·", "-").replace("…", "...").encode("ascii", "replace").decode()


def text_size(text: str, size: int = 14, bold: bool = False) -> tuple[int, int]:
    sprite = _sprite(text, size, bold)
    if sprite is None:
        (width, height), _ = cv2.getTextSize(_ascii(text), cv2.FONT_HERSHEY_SIMPLEX, size / 30.0, 2 if bold else 1)
        return width, height
    return sprite[0].shape[1], size


def put(image: np.ndarray, text: str, origin: tuple[int, int], size: int = 14, colour=TEXT, bold: bool = False,
        align: str = "left") -> int:
    """Draws text with its baseline at origin[1]; align left, centre or right of origin[0]. Returns
    the width drawn."""
    width, _ = text_size(text, size, bold)
    x, y = origin
    if align == "centre":
        x -= width // 2
    elif align == "right":
        x -= width
    sprite = _sprite(text, size, bold)
    if sprite is None:
        cv2.putText(image, _ascii(text), (x, y), cv2.FONT_HERSHEY_SIMPLEX, size / 30.0, colour, 2 if bold else 1, cv2.LINE_AA)
        return width
    mask, rise = sprite
    top = y - rise
    h, w = mask.shape
    y0, y1, x0, x1 = max(0, top), min(image.shape[0], top + h), max(0, x), min(image.shape[1], x + w)
    if y0 >= y1 or x0 >= x1:
        return width
    alpha = mask[y0 - top:y1 - top, x0 - x:x1 - x, None]
    region = image[y0:y1, x0:x1].astype(np.float32)
    image[y0:y1, x0:x1] = (region * (1.0 - alpha) + np.array(colour, np.float32) * alpha).astype(np.uint8)
    return width


def tag(image: np.ndarray, text: str, origin: tuple[int, int], colour, size: int = 13, filled: bool = True) -> tuple[int, int]:
    """A small label on a plate, its top-left corner at origin (kept inside the image). Returns its
    width and height."""
    width, _ = text_size(text, size, True)
    w, h = width + 12, size + 10
    x = min(max(0, origin[0]), image.shape[1] - w)
    y = min(max(0, origin[1]), image.shape[0] - h)
    if filled:
        cv2.rectangle(image, (x, y), (x + w, y + h), colour, -1)
        put(image, text, (x + 6, y + h - 6), size, (20, 20, 20), bold=True)
    else:
        cv2.rectangle(image, (x, y), (x + w, y + h), (18, 18, 18), -1)
        cv2.rectangle(image, (x, y), (x + w, y + h), colour, 1)
        put(image, text, (x + 6, y + h - 6), size, colour, bold=True)
    return w, h


def chip(image: np.ndarray, text: str, right: int, centre_y: int, colour, size: int = 13) -> int:
    """A rounded status chip whose right edge is at `right`; returns its left edge."""
    width, _ = text_size(text, size, True)
    w, h = width + 20, size + 12
    x0, y0 = right - w, centre_y - h // 2
    r = h // 2
    tint = tuple(int(c * 0.28) for c in colour)
    cv2.rectangle(image, (x0 + r, y0), (x0 + w - r, y0 + h), tint, -1)
    cv2.circle(image, (x0 + r, y0 + r), r, tint, -1, cv2.LINE_AA)
    cv2.circle(image, (x0 + w - r, y0 + r), r, tint, -1, cv2.LINE_AA)
    put(image, text, (x0 + 10, y0 + h - 7), size, colour, bold=True)
    return x0


def card(content: np.ndarray, title: str, meta: str = "", meta_colour=MUTED) -> np.ndarray:
    """content under a title strip, with a one-pixel edge round both."""
    h, w = content.shape[:2]
    out = np.empty((h + STRIP_H, w, 3), np.uint8)
    out[:STRIP_H] = STRIP
    out[STRIP_H:] = content
    put(out, title.upper(), (12, STRIP_H - 10), 12, MUTED, bold=True)
    if meta:
        put(out, meta, (w - 12, STRIP_H - 10), 13, meta_colour, bold=True, align="right")
    cv2.line(out, (0, STRIP_H - 1), (w - 1, STRIP_H - 1), CARD_EDGE, 1)
    cv2.rectangle(out, (0, 0), (w - 1, h + STRIP_H - 1), CARD_EDGE, 1)
    return out


def translucent(image: np.ndarray, points: np.ndarray, colour, opacity: float) -> np.ndarray:
    overlay = image.copy()
    cv2.fillPoly(overlay, [points], colour)
    return cv2.addWeighted(overlay, opacity, image, 1.0 - opacity, 0)
