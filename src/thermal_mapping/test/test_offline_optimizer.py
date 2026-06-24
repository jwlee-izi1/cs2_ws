"""Tests that the FedDCSA optimizer runs end-to-end offline.

Focus: smoke tests on the iteration loop + KKT convergence check.
- Static q_i (from arena_4drone): optimizer converges near the KKT solution
- Gate stays mostly feasible at convergence
- Aggregate constraint Σc_i r_i² ≤ B holds approximately at convergence
"""

import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..'))  # workspace root
sys.path.insert(0, os.path.join(HERE, '..', '..', 'fed_dcsa'))

from fed_dcsa.radial_coverage_algorithm import CoverageDrone, RadialCoverageFedDCSA


def kkt_static(q, c, r_star, r_max, B, iters=80):
    """Bisection on dual variable λ for the static KKT solution."""
    q = np.asarray(q, dtype=np.float64)
    c = np.asarray(c, dtype=np.float64)
    r_star = np.asarray(r_star, dtype=np.float64)
    r_max = np.asarray(r_max, dtype=np.float64)

    def r_at(lam):
        denom = q + lam * c
        denom = np.where(denom <= 1e-12, 1e-12, denom)
        r = q * r_star / denom
        return np.clip(r, 0.0, r_max)

    r_unc = np.clip(r_star, 0.0, r_max)
    if np.sum(c * r_unc ** 2) <= B:
        return r_unc

    lo, hi = 0.0, 1e6
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        if np.sum(c * r_at(mid) ** 2) > B:
            lo = mid
        else:
            hi = mid
    return r_at(0.5 * (lo + hi))


def test_optimizer_converges_to_kkt_static_qi():
    """With q_i fixed for K rounds, optimizer should land near the KKT solution."""
    drones = [
        CoverageDrone('cf1', q=10.0, c=1.0, r_star=1.85, r_max=1.9),
        CoverageDrone('cf2', q=1.5,  c=1.0, r_star=1.85, r_max=1.9),
        CoverageDrone('cf3', q=1.0,  c=1.0, r_star=1.85, r_max=1.9),
        CoverageDrone('cf4', q=0.5,  c=1.0, r_star=1.85, r_max=1.9),
    ]
    opt = RadialCoverageFedDCSA(
        drones=drones, B=8.0, T=5, c1=0.015, c2=0.18, noise_bound=0.0, seed=0,
    )
    for k in range(300):
        opt.run_round(k)
    r_kkt = kkt_static(
        q=[d.q for d in drones], c=[d.c for d in drones],
        r_star=[d.r_star for d in drones], r_max=[d.r_max for d in drones],
        B=8.0,
    )
    r_final = np.array([d.r for d in drones])
    # By K=300 we should be within ~10 cm of the KKT solution
    assert np.max(np.abs(r_final - r_kkt)) < 0.10, \
        f"r_final={r_final}, r_kkt={r_kkt}"


def test_constraint_approximately_satisfied():
    """At convergence, Σc_i r_i² ≤ B + small tolerance (per FedDCSA gate)."""
    drones = [
        CoverageDrone('cf1', q=10.0, c=1.0, r_star=1.85, r_max=1.9),
        CoverageDrone('cf2', q=1.5,  c=1.0, r_star=1.85, r_max=1.9),
        CoverageDrone('cf3', q=1.0,  c=1.0, r_star=1.85, r_max=1.9),
        CoverageDrone('cf4', q=0.5,  c=1.0, r_star=1.85, r_max=1.9),
    ]
    B = 8.0
    opt = RadialCoverageFedDCSA(
        drones=drones, B=B, T=5, c1=0.015, c2=0.18, noise_bound=0.0, seed=0,
    )
    for k in range(300):
        opt.run_round(k)
    G = sum(d.c * d.r ** 2 for d in drones) - B
    # Per FedDCSA theory: G ≤ eta_k at every round. At K=300, eta_300 = c2/sqrt(301) ≈ 0.01
    eta_300 = 0.18 / math.sqrt(301)
    assert G <= eta_300 + 0.05, f"G={G} exceeds eta_K={eta_300}"


def test_optimizer_responds_to_qi_change():
    """If q_i shifts mid-run, the algorithm should track the new optimum."""
    drones = [
        CoverageDrone('cf1', q=10.0, c=1.0, r_star=1.85, r_max=1.9),
        CoverageDrone('cf2', q=1.0,  c=1.0, r_star=1.85, r_max=1.9),
    ]
    opt = RadialCoverageFedDCSA(
        drones=drones, B=4.0, T=5, c1=0.015, c2=0.18, noise_bound=0.0, seed=0,
    )
    # Phase 1: cf1 dominant
    for k in range(100):
        opt.run_round(k)
    r_after_phase1 = [d.r for d in drones]
    # Switch dominance: now cf2 wants the bigger share
    drones[0].q = 1.0
    drones[1].q = 10.0
    for k in range(100, 250):
        opt.run_round(k)
    r_after_phase2 = [d.r for d in drones]
    # After q_i swap, cf2 should now have a larger r than it did before
    assert r_after_phase2[1] > r_after_phase1[1] + 0.05, \
        f"cf2 r_i should grow after q_i swap: was {r_after_phase1[1]}, now {r_after_phase2[1]}"


def test_zero_qi_means_drone_pulled_to_zero():
    """If a drone's q_i is 0, it has no objective preference and gets pulled to zero
    by the constraint (when constraint is binding)."""
    drones = [
        CoverageDrone('cf1', q=10.0, c=1.0, r_star=1.85, r_max=1.9),
        CoverageDrone('cf2', q=0.0,  c=1.0, r_star=1.85, r_max=1.9),
    ]
    opt = RadialCoverageFedDCSA(
        drones=drones, B=3.0, T=5, c1=0.015, c2=0.18, noise_bound=0.0, seed=0,
    )
    for k in range(300):
        opt.run_round(k)
    # cf2 with q=0 should be near 0
    assert drones[1].r < 0.1, f"cf2 with q=0 should be ~0, got {drones[1].r}"
    # cf1 should fill the budget: c*r^2 ≈ B → r ≈ sqrt(B) ≈ sqrt(3)
    assert abs(drones[0].r - math.sqrt(3.0)) < 0.1, f"cf1 r={drones[0].r}"
