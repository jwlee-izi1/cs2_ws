"""FedDCSA Algorithm 1 for the radial coverage problem (CDC 2026 demo).

Direct port of the 2D matplotlib FedDCSAOptimizer at
  /home/rk32226/drone-rl-2d/multi_drone/optimizer/fed_alloc.py
adapted to no-numpy / no-config-dataclass so this module stays a pure-Python
algorithm (the ROS node provides the config).

Diminishing schedules (paper sec IV.A):
    gamma_k = c_1 / sqrt(k+1)   step size (same for feasible and infeasible)
    eta_k   = c_2 / sqrt(k+1)   gate tolerance (drift NOT in threshold, per latest paper)

Per round k:
    1. Compute G_k = sum_i c_i r_i^2 - B.
    2. Gate: b_k = 1 if G_k <= eta_k else 0.
    3. For T inner steps:
         if b_k == 1: h_i = -2 q_i (r_star_i - r_i) + zeta_i   (grad f + noise)
         if b_k == 0: h_i = 2 c_i r_i + zeta_i                 (grad g + noise)
         r_i <- clip[0, r_max_i](r_i - gamma_k h_i)
       Output averaging accumulator runs on feasible inner steps only.

zeta_i ~ Uniform[-noise_bound, +noise_bound] (per-drone independent RNG).
"""

import math
import random
from dataclasses import dataclass, field
from typing import List


@dataclass
class CoverageDrone:
    name: str
    q: float
    c: float
    r_star: float
    r_max: float
    r: float = 0.0


@dataclass
class CoverageRound:
    k: int
    gate_state: int
    G: float
    eta_k: float
    gamma_k: float
    F: float
    radii: List[float] = field(default_factory=list)


class RadialCoverageFedDCSA:

    def __init__(self, drones, B, T, c1, c2, noise_bound=0.0, seed=0):
        self.drones: List[CoverageDrone] = drones
        self.B = float(B)
        self.T = int(T)
        self.c1 = float(c1)
        self.c2 = float(c2)
        self.noise_bound = float(noise_bound)
        # one RNG per drone so streams are independent (matches 2D fed_alloc)
        self._rngs = [random.Random(seed + i) for i in range(len(drones))]
        # output-averaging accumulators (diagnostic)
        self.weighted_x_sum = [0.0] * len(drones)
        self.S_B = 0.0

    def gamma(self, k):
        return self.c1 / math.sqrt(k + 1)

    def eta(self, k):
        return self.c2 / math.sqrt(k + 1)

    def G(self):
        return sum(d.c * d.r ** 2 for d in self.drones) - self.B

    def F(self):
        return sum(d.q * (d.r_star - d.r) ** 2 for d in self.drones)

    def _noise(self, i):
        if self.noise_bound <= 0.0:
            return 0.0
        return self._rngs[i].uniform(-self.noise_bound, self.noise_bound)

    def run_round(self, k):
        G_k = self.G()
        eta_k = self.eta(k)
        gamma_k = self.gamma(k)
        b_k = 1 if G_k <= eta_k else 0

        for _ in range(self.T):
            # output averaging accumulates BEFORE the update on feasible rounds
            if b_k == 1:
                for i, d in enumerate(self.drones):
                    self.weighted_x_sum[i] += gamma_k * d.r
                self.S_B += gamma_k

            for i, d in enumerate(self.drones):
                if b_k == 1:
                    grad = -2.0 * d.q * (d.r_star - d.r)
                else:
                    grad = 2.0 * d.c * d.r
                h_i = grad + self._noise(i)
                d.r = _project(d.r - gamma_k * h_i, d.r_max)

        return CoverageRound(
            k=k,
            gate_state=b_k,
            G=G_k,
            eta_k=eta_k,
            gamma_k=gamma_k,
            F=self.F(),
            radii=[d.r for d in self.drones],
        )

    def output_average(self):
        if self.S_B == 0.0:
            return None
        return [w / self.S_B for w in self.weighted_x_sum]


def _project(r, r_max):
    return max(0.0, min(r_max, r))
