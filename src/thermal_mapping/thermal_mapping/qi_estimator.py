"""Oracle q_i estimator: integrates `(T - ambient)` over each sector.

The estimator is a pure function. The Phase B ROS node will wrap this.
Used both by `scripts/preview_thermal_scenario.py` (offline) and by the
future `qi_estimator_node` (Phase B). Behavior must be identical between
offline and live to preserve the "tune in the previewer, run live" workflow.

Sectors are angular wedges around a common center (the arena origin),
parameterized by `phi_mid` / `phi_half` (radians) and an outer radius
`r_outer` over which the integral is taken. Integration is a coarse
uniform grid over the wedge's bounding box, masked to the wedge.
"""

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np

from .thermal_field import ThermalField


@dataclass
class SectorGeom:
    """Polar wedge for q_i integration.

    The wedge:
        d = ||(x,y) - center_xy||,   d <= r_outer
        phi = atan2(y - cy, x - cx),
        |wrap(phi - phi_mid)| <= phi_half
    """
    name: str
    phi_mid: float       # radians
    phi_half: float      # radians
    r_outer: float       # meters — integration radius
    center_xy: Tuple[float, float] = (0.0, 0.0)


def _wrap_to_pi(phi: np.ndarray) -> np.ndarray:
    return (phi + np.pi) % (2.0 * np.pi) - np.pi


def compute_sector_qi(
    field: ThermalField,
    t: float,
    sectors: Sequence[SectorGeom],
    grid_resolution: float = 0.05,
    threshold: float = 0.0,
) -> np.ndarray:
    """Compute q_i for each sector at time t.

    Returns a 1-D numpy array of length len(sectors), one q_i per sector.

    Two modes selected by `threshold`:
    - **threshold = 0 (default, mean over all cells):**
        q_i = qi_scale * mean(T - ambient)  over all cells in sector wedge
      Smooth signal, but cool cells dilute the mean.
    - **threshold > 0 (mean over hot cells only):**
        q_i = qi_scale * mean(T - ambient | T - ambient > threshold)
        if any cells exceed the threshold, else q_i = 0.
      Cold sectors give exactly zero → optimizer ignores them → budget
      concentrates on active sectors.

    Use threshold > 0 when q_i ratios need to be more concentrated for
    the optimizer to push the dominant drone toward r_star.
    """
    qi = np.zeros(len(sectors), dtype=np.float32)
    for i, sector in enumerate(sectors):
        cx, cy = sector.center_xy
        r = sector.r_outer
        n = max(16, int(np.ceil(2.0 * r / grid_resolution)))
        xs = np.linspace(cx - r, cx + r, n, dtype=np.float32)
        ys = np.linspace(cy - r, cy + r, n, dtype=np.float32)
        X, Y = np.meshgrid(xs, ys)
        DX = X - cx
        DY = Y - cy
        R2 = DX * DX + DY * DY
        PHI = np.arctan2(DY, DX)
        rel_phi = _wrap_to_pi(PHI - sector.phi_mid)
        wedge_mask = (R2 <= r * r) & (np.abs(rel_phi) <= sector.phi_half)
        if not wedge_mask.any():
            qi[i] = 0.0
            continue
        T = field.evaluate(X, Y, t)
        excess = T - field.ambient
        if threshold > 0.0:
            hot_mask = wedge_mask & (excess > threshold)
            if not hot_mask.any():
                qi[i] = 0.0
                continue
            qi[i] = field.qi_scale * float(excess[hot_mask].mean())
        else:
            qi[i] = field.qi_scale * float(excess[wedge_mask].mean())
    return qi
