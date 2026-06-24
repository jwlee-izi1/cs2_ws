"""Three candidate reward functions, all research-grounded.

All operate on per-drone state at each 5 Hz tick.

Candidate A — Persistent-monitoring style (start here):
    Each cell carries a priority that grows with age × temperature.
    On visit, the priority is "collected" and reset.

Candidate B — A + Tampere-style step penalty:
    Same as A, plus a small per-second cost to discourage hovering.

Candidate C — Three-term cell-value (granular).
    Splits hot+unexplored, unexplored bonus, and stale-revisit into separate
    weights, for fine-grained tuning.

Reward terms common to all three: edge_pull, smoothness, soft leash.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class RewardWeights:
    # Coverage-priority weights
    w_hot: float = 1.0
    w_explore: float = 0.3
    w_stale_revisit: float = 0.1  # only used by Candidate C

    # Behavioral terms
    w_edge: float = 5.0
    # iter15-retry: anchor weight dropped 80 → 45 after observing that w=80
    # destabilizes the policy. iter13 from BC + w=80 → conservative at 0.97.
    # iter15 from iter12 + w=80 → wildly oscillating between r=0.2-1.7m at
    # 25k-50k steps, KL=0.075 (too aggressive PPO updates). Dropping to 45
    # to give the policy room to settle near 0.99 without violent gradient
    # pulling. Symmetric anchor kept (good fix from iter13). Both undershoot
    # AND overshoot are penalized.
    w_anchor: float = 45.0
    edge_target: float = 0.99
    w_smooth: float = 0.5
    w_leash: float = 10.0
    w_step: float = 0.01  # only used by Candidate B
    # iter12 NEW: θ-bias penalty. Penalize the policy if it lets the running EMA
    # of θ drift from zero (i.e. consistently sweeps one side of the wedge).
    # iter11 had this exact failure mode. With α=0.05 the EMA has a ~20-tick
    # (4 s) memory; penalty fires when the policy stays asymmetric > 4 s.
    w_theta_bias: float = 5.0
    theta_bias_alpha: float = 0.05

    # Caps
    max_age_hot: float = 30.0
    max_age_explore: float = 60.0
    edge_offset: float = 0.5  # max(0, r/r_i^k - edge_offset)
    density_amp: float = 0.0  # Fix 3: amplifier on edge_pull by hot density (0 = disabled)

    # 2026-05-28 iter10: sector-violation penalty. Quadratic in the angular
    # offset BEYOND the wedge boundary. With point-mass dynamics this was
    # implicitly enforced (target was always in-wedge → drone followed
    # target instantly), but cascaded PID has overshoot so the drone can
    # physically leave the wedge even when the policy commands a centered
    # target. Without this term, training >100k steps degrades because the
    # policy learns it can "tunnel" through neighbors' sectors for extra
    # coverage reward. Default 50.0 is calibrated so it dominates coverage
    # signal (~50-100/tick typical) at d_phi_excess = 0.1 rad (~6°).
    w_sector: float = 50.0

    # 2026-06-04 iter15: TIME-PHASED reward. The edge/sweep terms (edge, anchor,
    # theta_bias) are GATED by edge_gate(t) = clip((t-start)/(end-start), 0, 1):
    #   EARLY (t < phase_ramp_start): gate=0 → no edge pull → the policy maximizes
    #     heat-weighted coverage → flies INSIDE to the fire's hot spot (leash +
    #     sector still bound it; coverage + smooth still on).
    #   RAMP (start→end): gate 0→1, smooth blend outward toward the leash.
    #   LATE (t > phase_ramp_end): gate=1 → full edge_anchor (r→0.99·leash) + sweep
    #     → ride the leash + sweep, the BC behavior.
    # leash (soft_leash) + sector are NEVER gated — the leash is always the OUTER
    # BOUND; we only time-gate the ATTRACTION to the edge. coverage always-on.
    phase_ramp_start: float = 30.0
    phase_ramp_end: float = 50.0

    # 2026-06-04 iter16: COMMENSURATE-TERM reward. The raw coverage term is ~14-17k/tick
    # (spiking to millions when entering a fresh hot region) — 4-5 orders ABOVE the
    # edge/anchor/theta_bias terms (~1-5/tick). That dominance, combined with the iter15
    # edge-gate forcing leash-riding (r→0.99·leash, ~0.6 m OUTSIDE the fire's hot bulk) for
    # 86% of the episode, made coverage COLLAPSE late → the RAW-reward training curve DECLINED
    # (the policy getting better at a coverage-costly objective). Fix: BOUND + DOWN-SCALE
    # coverage to a SMALL secondary bonus, O(1-5)/tick, and make the leash-sweep terms
    # (edge/anchor/theta_bias — now ALWAYS-ON, no time gate) the DOMINANT rewarded behavior →
    # improvement == better leash-tracking/sweeping == higher reward == a rising-then-plateau
    # curve. coverage_contrib = w_cov * min(coverage_raw / cov_scale, cov_cap).
    w_cov: float = 1.0
    cov_scale: float = 3000.0  # raw ~14k on-fire / 3000 ≈ 4.7
    cov_cap: float = 5.0       # clip the fresh-region spikes (raw → millions) to this

    # 2026-06-04 iter17: ANGULAR-COVERAGE sweep reward. iter16 fixed the curve but rode the
    # leash RIGIDLY (corr 1.0) and barely swept (~50° of 90°) because capping coverage removed
    # what drove iter15's sweep. This term rewards covering the sector ARC (angular) — orthogonal
    # to the anchor (radial) — so the drone sweeps WIDE *at* the leash. The env divides the sector
    # into sweep_n_bins angular bins, ages them, and each tick collects the visited bin's
    # accumulated staleness (capped at sweep_age_cap) → sweeping the full arc (esp. the stale
    # sector-ends) maximizes it. Can't be gamed by oscillating in place (must visit stale bins).
    w_sweep: float = 0.5
    sweep_n_bins: int = 12
    sweep_age_cap: float = 10.0


@dataclass
class RewardContext:
    candidate: str = "A"
    weights: RewardWeights = field(default_factory=RewardWeights)
    t_ambient: float = 22.0
    dt: float = 0.2
    # iter12: running exponential moving average of θ. Updated in-place inside
    # compute_reward(). The env MUST reset this to 0.0 on each episode reset
    # so the bias term doesn't leak across episodes.
    theta_bias_ema: float = 0.0


def cell_value(
    temp: np.ndarray,
    age: np.ndarray,
    was_visited: np.ndarray,
    t_ambient: float,
    w: RewardWeights,
    candidate: str = "A",
) -> np.ndarray:
    """Per-cell scalar value used for the coverage signal.

    temp:    (..., ) temperature at cells (°C)
    age:     (..., ) seconds-since-last-observation
    was_visited: (..., ) bool, True if cell has ever been observed
    """
    hot_excess = np.clip(temp - t_ambient, 0.0, None)
    hot_term = w.w_hot * hot_excess * np.minimum(age, w.max_age_hot)
    not_visited = (~was_visited).astype(np.float32)
    explore_term = w.w_explore * not_visited * np.minimum(age, w.max_age_explore)
    if candidate == "C":
        stale_term = w.w_stale_revisit * np.minimum(age, w.max_age_hot)
        return hot_term + explore_term + stale_term
    return hot_term + explore_term


def coverage_collected(
    *,
    fov_cells_temps_before: np.ndarray,
    fov_cells_temps_after: np.ndarray,
    fov_cells_ages_before: np.ndarray,
    fov_cells_ages_after: np.ndarray,
    fov_cells_visited_before: np.ndarray,
    fov_cells_visited_after: np.ndarray,
    t_ambient: float,
    weights: RewardWeights,
    candidate: str = "A",
) -> float:
    """Sum of cell-value DROP this step over cells in (FOV ∩ sector).

    Drop = before − after (positive). For unvisited cells the age was at
    whatever the previous tick said; for cells that got visited this tick
    age resets to 0 → drop is the full previous value (we "collected" it).
    """
    v_before = cell_value(
        fov_cells_temps_before, fov_cells_ages_before, fov_cells_visited_before,
        t_ambient, weights, candidate,
    )
    v_after = cell_value(
        fov_cells_temps_after, fov_cells_ages_after, fov_cells_visited_after,
        t_ambient, weights, candidate,
    )
    return float((v_before - v_after).sum())


def edge_pull(
    r_drone: float,
    r_leash: float,
    weights: RewardWeights,
    hot_density: float = 0.0,
) -> float:
    """Reward for using the leash, optionally amplified by FOV hot density (Fix 3).

    edge_pull = w_edge × max(0, r/leash - offset) × (1 + density_amp × hot_density)

    When density_amp > 0, the edge bonus is only fully valuable when there's
    fire to track at the edge. If density_amp = 0, falls back to plain edge_pull.
    """
    if r_leash <= 1e-6:
        return 0.0
    # iter16: CAP the ratio at edge_target so edge_pull rewards approaching the leash but
    # gives NO incentive to overshoot past it. Without this cap edge_pull is unbounded in
    # r/leash, so (with coverage now small) the reward peaked at r/leash>1.0 → the policy
    # would overshoot the leash. The anchor (symmetric about edge_target) + soft_leash now
    # set the precise hold point at ~edge_target.
    ratio = min(r_drone / r_leash, weights.edge_target)
    base = weights.w_edge * max(0.0, ratio - weights.edge_offset)
    multiplier = 1.0 + weights.density_amp * float(max(0.0, min(1.0, hot_density)))
    return float(base * multiplier)


def edge_anchor_penalty(r_drone: float, r_leash: float, weights: RewardWeights) -> float:
    """iter13: SYMMETRIC quadratic penalty for |r/leash − edge_target|. Pulls
    drone to the edge as the DOMINANT behavior term. Penalizes BOTH undershoot
    (r/leash < edge_target) AND overshoot (r/leash > edge_target).

    Was asymmetric (max(0, deficit)²) in iter12 — only penalized undershoot,
    which let PPO discover the [0.99, 1.03] "no-man's-land" exploit where the
    policy could overshoot the leash with no anchor penalty + only weak
    soft_leash penalty above 1.0. Now symmetric: deviation in BOTH directions
    is penalized."""
    if weights.w_anchor <= 0.0 or r_leash <= 1e-6:
        return 0.0
    deviation = abs((r_drone / r_leash) - weights.edge_target)
    return float(weights.w_anchor * deviation * deviation)


def smoothness_penalty(action: np.ndarray, prev_action: np.ndarray | None, weights: RewardWeights) -> float:
    if prev_action is None:
        return 0.0
    delta = np.asarray(action, dtype=np.float32) - np.asarray(prev_action, dtype=np.float32)
    return float(weights.w_smooth * np.sum(delta * delta))


def soft_leash_penalty(r_drone: float, r_leash: float, weights: RewardWeights) -> float:
    if r_leash <= 1e-6:
        return 0.0
    over = max(0.0, (r_drone / r_leash) - 1.0)
    return float(weights.w_leash * over * over)


def step_penalty(dt: float, weights: RewardWeights) -> float:
    return float(weights.w_step * dt)


def theta_bias_penalty(ctx: 'RewardContext', weights: RewardWeights) -> float:
    """iter12: penalize sustained θ bias. Updates the EMA in-place on ctx
    BEFORE returning, so the caller doesn't need to. The EMA tracks the
    running mean of θ — if the policy stays on one side of the wedge the
    EMA drifts away from zero and the penalty grows quadratically.

    Note: ctx.theta_bias_ema MUST be updated by the caller using action[0]
    before this is called (we keep update separate so compute_reward can
    update before all terms see the new value).
    """
    return float(weights.w_theta_bias * ctx.theta_bias_ema * ctx.theta_bias_ema)


def sector_violation_penalty(
    *,
    phi_drone: float,
    phi_mid: float,
    phi_half: float,
    weights: RewardWeights,
) -> float:
    """Quadratic penalty for being OUTSIDE the assigned wedge.

    Returns 0 when ``|d_phi| ≤ phi_half`` (drone in-wedge), otherwise
    ``w_sector × (|d_phi| - phi_half)^2``. All angles in radians.

    The angular wrap-around to [-π, π] matters near phi_mid = ±π (cf3, cf4).
    """
    import math
    d_phi = ((phi_drone - phi_mid + math.pi) % (2 * math.pi)) - math.pi
    excess = max(0.0, abs(d_phi) - phi_half)
    return float(weights.w_sector * excess * excess)


def compute_reward(
    *,
    ctx: RewardContext,
    coverage_term: float,
    r_drone: float,
    r_leash: float,
    action: np.ndarray,
    prev_action: Optional[np.ndarray],
    hot_density: float = 0.0,
    phi_drone: float = 0.0,
    phi_mid: float = 0.0,
    phi_half: float = 0.0,
    t: float = 0.0,
    sweep_term: float = 0.0,
) -> tuple[float, dict[str, float]]:
    """Assemble the per-tick reward from individual terms.

    hot_density (Fix 3): fraction [0,1] of FOV cells that are 'hot'
    (T > T_ambient + 30°C). Used to amplify edge_pull so drone is rewarded
    for edge-riding ONLY when there's fire to track at the edge.

    phi_drone / phi_mid / phi_half (Fix 4): for the sector-violation penalty
    (only meaningful under cascaded PID dynamics where the drone can
    physically leave its wedge despite a clamped target).
    """
    w = ctx.weights
    # iter12: update theta_bias EMA in-place using THIS step's theta command.
    ema_alpha = float(w.theta_bias_alpha)
    ctx.theta_bias_ema = (1.0 - ema_alpha) * ctx.theta_bias_ema + ema_alpha * float(action[0])
    # iter16: COMMENSURATE terms. coverage is BOUNDED + DOWN-SCALED to a small secondary
    # bonus (O(1-5)/tick); the leash-sweep terms (edge/anchor/theta_bias) are ALWAYS-ON
    # (the iter15 time gate `t` is no longer used) so riding+sweeping the leash is the
    # DOMINANT rewarded behavior → reward RISES as the policy learns to track the (moving)
    # leash + stay in-sector + sweep, instead of declining as coverage collapses. Note the
    # early/inside behavior still emerges naturally: when the leash is small (early), edge
    # terms target r≈0.99·leash≈origin, i.e. ON the nascent fire → coverage and leash-riding
    # AGREE; they only diverge late, where coverage (small, capped) yields to leash-riding.
    coverage_contrib = w.w_cov * min(coverage_term / max(1e-6, w.cov_scale), w.cov_cap)
    parts = {
        "coverage": coverage_contrib,
        # iter17: angular-coverage sweep bonus (sweep_term = visited-bin staleness from the env).
        "sweep": w.w_sweep * float(sweep_term),
        "edge": edge_pull(r_drone, r_leash, w, hot_density=hot_density),
        "anchor": -edge_anchor_penalty(r_drone, r_leash, w),
        "smooth": -smoothness_penalty(action, prev_action, w),
        "leash": -soft_leash_penalty(r_drone, r_leash, w),
        "sector": -sector_violation_penalty(
            phi_drone=phi_drone, phi_mid=phi_mid, phi_half=phi_half, weights=w),
        "theta_bias": -theta_bias_penalty(ctx, w),
    }
    if ctx.candidate == "B":
        parts["step"] = -step_penalty(ctx.dt, w)
    total = float(sum(parts.values()))
    return total, parts
