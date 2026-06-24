"""FedDCSA optimizer wrapper for the env.

Wraps fed_dcsa.radial_coverage_algorithm.RadialCoverageFedDCSA (pure Python,
no ROS) so the env can fire a round at the optimizer's 0.5 Hz wall-clock
rate (one round every 2 s of sim time). Also implements the oracle q_i
estimator (mean(T - ambient) over sector wedge cells inside r_outer).
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

# Make fed_dcsa/thermal_mapping importable when this module runs outside a ROS workspace.
# parents[3] is cs2_ws/src; adding cs2_ws/src/<pkg> exposes the inner Python module.
_CS2_WS_SRC = Path(__file__).resolve().parents[3]
for sub in ("fed_dcsa", "thermal_mapping"):
    p = _CS2_WS_SRC / sub
    if p.is_dir() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

from fed_dcsa.radial_coverage_algorithm import (
    CoverageDrone,
    RadialCoverageFedDCSA,
)
from fed_dcsa.lagrangian_baseline import LagrangianBaseline
from thermal_mapping.qi_estimator import SectorGeom, compute_sector_qi


class OptimizerWrap:
    """Runs FedDCSA at a fixed wall-clock round rate (default 0.5 Hz)."""

    def __init__(
        self,
        drones_cfg: list[dict],
        budget_B: float,
        round_rate_hz: float = 0.5,
        T_inner: int = 5,
        c1: float = 0.015,
        c2: float = 0.18,
        noise_bound: float = 0.015,
        seed: int = 0,
        optimizer_type: str = "feddcsa",   # "feddcsa" | "lagrangian"
    ):
        # drones_cfg: list of dicts with q, c, r_star, r_max keyed in order.
        self.cfg = [dict(d) for d in drones_cfg]
        self.B = float(budget_B)
        self.round_period = 1.0 / float(round_rate_hz)
        self.T_inner = int(T_inner)
        self.c1 = float(c1)
        self.c2 = float(c2)
        self.noise_bound = float(noise_bound)
        self.seed = int(seed)
        self.optimizer_type = str(optimizer_type).lower()
        self._build_optimizer()

    def _build_optimizer(self):
        drones = [
            CoverageDrone(
                name=d.get("name", f"cf{i+1}"),
                q=float(d["q"]),
                c=float(d["c"]),
                r_star=float(d["r_star"]),
                r_max=float(d["r_max"]),
                r=float(d.get("r0", 0.0)),
            )
            for i, d in enumerate(self.cfg)
        ]
        if self.optimizer_type == "lagrangian":
            self.opt = LagrangianBaseline(
                drones=drones,
                B=self.B,
                T=self.T_inner,
                c1=self.c1,
                c2=self.c2,
                noise_bound=self.noise_bound,
                seed=self.seed,
            )
        else:
            self.opt = RadialCoverageFedDCSA(
                drones=drones,
                B=self.B,
                T=self.T_inner,
                c1=self.c1,
                c2=self.c2,
                noise_bound=self.noise_bound,
                seed=self.seed,
            )
        self.round_idx = 0
        self.last_round_t = -math.inf
        self.last_round = None  # CoverageRound or LagrangianRound

    def reset(self, drones_cfg: list[dict] | None = None, seed: int | None = None):
        if drones_cfg is not None:
            self.cfg = [dict(d) for d in drones_cfg]
        if seed is not None:
            self.seed = int(seed)
        self._build_optimizer()

    def maybe_step(self, t: float, q_values: list[float] | None = None) -> bool:
        """If round_period has elapsed since last round, advance the optimizer.

        If q_values provided, updates each drone's q before the round (matches
        the live behavior where /coverage/sector_weights is republished).
        Returns True if a round was run this call.
        """
        if t - self.last_round_t < self.round_period - 1e-9:
            return False
        if q_values is not None:
            for d, q in zip(self.opt.drones, q_values):
                d.q = float(q)
        self.last_round = self.opt.run_round(self.round_idx)
        self.round_idx += 1
        self.last_round_t = float(t)
        return True

    @property
    def leashes(self) -> np.ndarray:
        return np.array([d.r for d in self.opt.drones], dtype=np.float32)

    @property
    def gate_state(self) -> int:
        if self.last_round is None:
            return 1
        return int(getattr(self.last_round, "gate_state", 1))


def estimate_q_for_sector(
    field,
    t: float,
    phi_mid_rad: float,
    phi_half_rad: float,
    r_outer: float,
    ambient: float = None,           # kept for backward-compat; unused (field.ambient is used)
    scale: float = None,             # kept for backward-compat; unused (field.qi_scale is used)
    n_samples: int = None,           # kept for backward-compat; unused
) -> float:
    """Numerically EQUIVALENT to the previewer's compute_sector_qi.

    Uses the exact same function (cartesian grid + wedge mask, resolution 0.05m,
    threshold=0 for mean-over-all-cells). Critical for reproducing the documented
    Lagrangian violation behavior (max Σc_i r² ≈ 8.83).
    """
    sector = SectorGeom(
        name="rl_env_sector",
        phi_mid=float(phi_mid_rad),
        phi_half=float(phi_half_rad),
        r_outer=float(r_outer),
    )
    qi_arr = compute_sector_qi(field, t, [sector], grid_resolution=0.05, threshold=0.0)
    return float(qi_arr[0])
