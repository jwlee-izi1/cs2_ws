"""Lagrangian dual ascent baseline for the radial coverage problem.

For paper comparison against FedDCSA. Same I/O signature as
`RadialCoverageFedDCSA` (same drones, B, T, c_1, c_2, noise_bound, seed).
Same diminishing step schedules: γ_k = c_1/√(k+1), η_k = c_2/√(k+1).

**Key difference from FedDCSA:** NO gate. Single combined Lagrangian gradient
(objective + λ · constraint). Dual variable λ updated per round based on the
ACTUAL constraint value `G = Σc_i r_i² − B`.

Per round k:
    G_k = Σc_i r_i² - B
    For T inner steps:
        h_i = -2 q_i (r_star_i - r_i)      # objective gradient
              + λ_k · 2 c_i r_i              # constraint gradient
              + ζ_i                          # gradient noise
        r_i = clip[0, r_max]( r_i - γ_k · h_i )
    After inner loop:
        λ_k = max(0, λ_k + η_k · G(r_new))

**Per-iterate behavior under non-stationarity:** when q_i changes drastically,
the primal r_i values race toward the (new) optimum. If the new optimum requires
giving more budget to one drone, the primal may temporarily exceed Σc_i r² > B
while λ catches up (dual ascent has its own diminishing lag). This is the
realistic failure mode the paper documents: baseline outputs published per-round
to the planner may be infeasible during transients.

In contrast, FedDCSA's gate forces r values to stay within `B + η_k` at every
published iterate.
"""

import math
import random
from dataclasses import dataclass, field
from typing import List

from .radial_coverage_algorithm import CoverageDrone


@dataclass
class LagrangianRound:
    k: int
    G: float          # constraint value at start of round (Σc_i r² − B)
    eta_k: float
    gamma_k: float
    lambda_k: float   # dual variable AFTER this round's dual update
    radii: List[float] = field(default_factory=list)


def _project(r: float, r_max: float) -> float:
    return max(0.0, min(r_max, r))


class LagrangianBaseline:
    """Lagrangian dual ascent — same I/O signature as RadialCoverageFedDCSA.

    Difference: no gate, no two-mode behavior. Single Lagrangian gradient
    with dual variable λ that adapts to actual constraint violation.
    """

    def __init__(self, drones, B, T, c1, c2, noise_bound=0.0, seed=0,
                 lambda_init: float = 0.0):
        self.drones: List[CoverageDrone] = drones
        self.B = float(B)
        self.T = int(T)
        self.c1 = float(c1)
        self.c2 = float(c2)
        self.noise_bound = float(noise_bound)
        self.lambda_k = float(lambda_init)
        # one RNG per drone — same noise pattern as FedDCSA for fair comparison
        self._rngs = [random.Random(seed + i) for i in range(len(drones))]

    def gamma(self, k: int) -> float:
        return self.c1 / math.sqrt(k + 1)

    def eta(self, k: int) -> float:
        return self.c2 / math.sqrt(k + 1)

    def G(self) -> float:
        return sum(d.c * d.r ** 2 for d in self.drones) - self.B

    def _noise(self, i: int) -> float:
        if self.noise_bound <= 0.0:
            return 0.0
        return self._rngs[i].uniform(-self.noise_bound, self.noise_bound)

    def run_round(self, k: int) -> LagrangianRound:
        G_at_start = self.G()
        eta_k = self.eta(k)
        gamma_k = self.gamma(k)

        # Primal inner loop: single combined Lagrangian gradient.
        for _ in range(self.T):
            for i, d in enumerate(self.drones):
                grad_f = -2.0 * d.q * (d.r_star - d.r)
                grad_g = 2.0 * d.c * d.r
                h_i = grad_f + self.lambda_k * grad_g + self._noise(i)
                d.r = _project(d.r - gamma_k * h_i, d.r_max)

        # Dual ascent AFTER primal step, using updated r values.
        G_after = self.G()
        self.lambda_k = max(0.0, self.lambda_k + eta_k * G_after)

        return LagrangianRound(
            k=k,
            G=G_at_start,
            eta_k=eta_k,
            gamma_k=gamma_k,
            lambda_k=self.lambda_k,
            radii=[d.r for d in self.drones],
        )
