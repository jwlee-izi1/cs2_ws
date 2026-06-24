"""Polar/Cartesian helpers for sector-based coverage. Pure Python, no ROS."""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class Sector:
    phi_mid: float   # sector centerline angle, radians
    phi_half: float  # half-width, radians
    r_max: float     # hardware box constraint
    r_star: float    # offline target standoff
    q: float         # priority weight
    c: float         # constraint coefficient

    @property
    def phi_low(self) -> float:
        return self.phi_mid - self.phi_half

    @property
    def phi_high(self) -> float:
        return self.phi_mid + self.phi_half


def wrap_angle(phi: float) -> float:
    """Wrap to (-pi, pi]."""
    phi = (phi + math.pi) % (2.0 * math.pi) - math.pi
    if phi <= -math.pi:
        phi += 2.0 * math.pi
    return phi


def polar_to_world(r: float, phi: float, center_xy: Tuple[float, float]) -> Tuple[float, float]:
    cx, cy = center_xy
    return (cx + r * math.cos(phi), cy + r * math.sin(phi))


def world_to_polar(x: float, y: float, center_xy: Tuple[float, float]) -> Tuple[float, float]:
    cx, cy = center_xy
    dx, dy = x - cx, y - cy
    return (math.hypot(dx, dy), math.atan2(dy, dx))


def clamp_to_sector(phi: float, sector: Sector) -> float:
    """Clamp angle into sector wedge, handling wrap-around at +/-pi."""
    rel = wrap_angle(phi - sector.phi_mid)
    rel = max(-sector.phi_half, min(sector.phi_half, rel))
    return wrap_angle(sector.phi_mid + rel)
