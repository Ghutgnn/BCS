from __future__ import annotations

import math


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def normalize_yaw_rad(yaw: float) -> float:
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def parse_resolution(value: str) -> tuple[int, int]:
    width_text, height_text = value.lower().split("x", 1)
    return int(width_text), int(height_text)
