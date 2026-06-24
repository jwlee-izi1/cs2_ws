#!/usr/bin/env python3
"""Offline previewer for the dynamic-thermal-field scenario (Phase A).

Pure Python, no ROS. Pipes:

  thermal_field.yaml      ->  T(x,y,t)
  arena_4drone.yaml       ->  sector geometry + FedDCSA optimizer params
  compute_sector_qi(...)  ->  q_i(t) sequence
  RadialCoverageFedDCSA   ->  r_i^k(t) trajectory
  frozen-q_i KKT solver   ->  r_i^*(t) instantaneous static optimum

Produces a 5-panel PNG: truth snapshots, q_i(t), r_i^k+r_i^* overlay,
lag-vs-K, constraint value Σc_i (r_i^k)^2 vs B.

Run:
    python3 scripts/preview_thermal_scenario.py
    python3 scripts/preview_thermal_scenario.py --duration 360 --round-rate 1.0
"""

import argparse
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List

import matplotlib.pyplot as plt
import numpy as np
import yaml

WORKSPACE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WORKSPACE / 'src' / 'thermal_mapping'))
sys.path.insert(0, str(WORKSPACE / 'src' / 'fed_dcsa'))

from thermal_mapping.thermal_field import ThermalField  # noqa: E402
from thermal_mapping.qi_estimator import SectorGeom, compute_sector_qi  # noqa: E402
from fed_dcsa.radial_coverage_algorithm import CoverageDrone, RadialCoverageFedDCSA  # noqa: E402
from fed_dcsa.lagrangian_baseline import LagrangianBaseline  # noqa: E402


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

@dataclass
class DroneCfg:
    name: str
    phi_mid: float
    phi_half: float
    q_fallback: float       # static q from YAML (only used if no estimator)
    c: float
    r_star: float
    r_max: float


def load_arena(path: Path):
    with open(path) as f:
        doc = yaml.safe_load(f)
    center_xy = tuple(doc['arena']['center_xy'])
    names = doc['drone_names']
    drones = []
    for name in names:
        s = doc['sectors'][name]
        drones.append(DroneCfg(
            name=name,
            phi_mid=math.radians(s['phi_mid_deg']),
            phi_half=math.radians(s['phi_half_deg']),
            q_fallback=float(s['q']),
            c=float(s['c']),
            r_star=float(s['r_star']),
            r_max=float(s['r_max']),
        ))
    opt = doc['optimizer']
    return {
        'center_xy': center_xy,
        'drones': drones,
        'B': float(opt['budget_B']),
        'T': int(opt['T']),
        'K': int(opt['K']),
        'c1': float(opt['c_1']),
        'c2': float(opt['c_2']),
        'noise_bound': float(opt['noise_bound']),
        'round_rate_hz': float(opt['round_rate_hz']),
    }


# ---------------------------------------------------------------------------
# Frozen-q_i KKT solver (reference r_i^*(t))
# ---------------------------------------------------------------------------

def solve_static_kkt(q, c, r_star, r_max, B, lambda_max=1e6, iters=80):
    """Solve min Σ q_i (r_star_i - r_i)^2  s.t.  Σ c_i r_i^2 ≤ B,  0 ≤ r_i ≤ r_max_i.

    Returns r* per drone. Uses bisection on the dual variable λ ≥ 0.
    Interior stationary condition: r_i = q_i r_star_i / (q_i + λ c_i).
    Box clipping handled by clipping at each λ trial.
    """
    q = np.asarray(q, dtype=np.float64)
    c = np.asarray(c, dtype=np.float64)
    r_star = np.asarray(r_star, dtype=np.float64)
    r_max = np.asarray(r_max, dtype=np.float64)

    def r_at(lam):
        denom = q + lam * c
        denom = np.where(denom <= 1e-12, 1e-12, denom)
        r = q * r_star / denom
        return np.clip(r, 0.0, r_max)

    # Quick: unconstrained box solution feasible?
    r_unc = np.clip(r_star, 0.0, r_max)
    if np.sum(c * r_unc ** 2) <= B:
        return r_unc

    lam_lo, lam_hi = 0.0, lambda_max
    if np.sum(c * r_at(lam_hi) ** 2) > B:
        return np.zeros_like(r_star)  # infeasible regime — degenerate

    for _ in range(iters):
        lam = 0.5 * (lam_lo + lam_hi)
        if np.sum(c * r_at(lam) ** 2) > B:
            lam_lo = lam
        else:
            lam_hi = lam
    return r_at(0.5 * (lam_lo + lam_hi))


# ---------------------------------------------------------------------------
# Diagnostic counters
# ---------------------------------------------------------------------------

def diagnostics(qi_history: np.ndarray, lag_history: np.ndarray) -> dict:
    """Compute summary diagnostics. qi_history shape (K, N), lag_history (K, N)."""
    K, N = qi_history.shape
    argmax = np.argmax(qi_history, axis=1)
    transitions = int(np.sum(argmax[1:] != argmax[:-1]))
    per_sector_range = qi_history.max(axis=0) - qi_history.min(axis=0)
    sigma = qi_history.std(axis=0)
    mean = np.where(qi_history.mean(axis=0) > 1e-9, qi_history.mean(axis=0), 1e-9)
    cv = sigma / mean
    # second half lag (deep-K regime)
    half = K // 2
    return {
        'K_total': K,
        'argmax_transitions': transitions,
        'per_sector_range': per_sector_range.tolist(),
        'per_sector_cov': cv.tolist(),
        'max_lag': float(np.max(lag_history)),
        'mean_lag_full': float(np.mean(lag_history)),
        'mean_lag_second_half': float(np.mean(lag_history[half:])),
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--thermal-yaml', type=Path,
                    default=WORKSPACE / 'src/thermal_mapping/config/thermal_field.yaml')
    ap.add_argument('--arena-yaml', type=Path,
                    default=WORKSPACE / 'src/cf_coverage_planner/config/arena_4drone.yaml')
    ap.add_argument('--duration', type=float, default=360.0,
                    help='Run duration in seconds (default: 360 = 6 min)')
    ap.add_argument('--round-rate', type=float, default=None,
                    help='Optimizer rounds per second (default: from arena yaml)')
    ap.add_argument('--qi-rate', type=float, default=None,
                    help='q_i estimator rate (default = round rate; slower → stair-step)')
    ap.add_argument('--r-integration', type=float, default=1.9,
                    help='Sector integration outer radius in meters. Default 1.9 = arena_radius '
                         '(only integrate over cells that can burn). Larger values dilute q_i '
                         'with always-ambient cells outside the fuel disc.')
    ap.add_argument('--out', type=Path,
                    default=WORKSPACE / 'cache/preview_thermal_scenario.png')
    args = ap.parse_args()

    # Load configs
    field = ThermalField.from_yaml(str(args.thermal_yaml))
    arena = load_arena(args.arena_yaml)
    # Read qi.threshold from YAML (default 0 = mean over all cells)
    with open(args.thermal_yaml) as f:
        _doc = yaml.safe_load(f)
    qi_threshold = float((_doc.get('qi') or {}).get('threshold', 0.0))
    print(f"  qi threshold   = {qi_threshold}")

    round_rate = args.round_rate if args.round_rate else arena['round_rate_hz']
    qi_rate = args.qi_rate if args.qi_rate else round_rate
    K_max = int(args.duration * round_rate)
    dt = 1.0 / round_rate
    qi_period = 1.0 / qi_rate

    print(f"Previewer scenario:")
    print(f"  duration       = {args.duration:.1f} s  ({args.duration/60:.1f} min)")
    print(f"  round_rate_hz  = {round_rate}")
    print(f"  qi_rate_hz     = {qi_rate}{'  [sample-and-hold]' if qi_rate < round_rate else ''}")
    print(f"  K_max          = {K_max}")
    print(f"  budget B       = {arena['B']}")
    print(f"  c_1, c_2       = {arena['c1']}, {arena['c2']}")
    print(f"  fire ignition  = ({field.fire.ignition_x:.2f}, {field.fire.ignition_y:.2f})  "
          f"v_front={field.fire.v_front}")
    print()

    # Sector geometry for q_i integration
    sectors = [
        SectorGeom(
            name=d.name,
            phi_mid=d.phi_mid,
            phi_half=d.phi_half,
            r_outer=args.r_integration,
            center_xy=arena['center_xy'],
        ) for d in arena['drones']
    ]

    # Build optimizer drones (q updated each round) — separate state per algorithm
    opt_drones = [
        CoverageDrone(name=d.name, q=d.q_fallback, c=d.c,
                       r_star=d.r_star, r_max=d.r_max, r=0.0)
        for d in arena['drones']
    ]
    optimizer = RadialCoverageFedDCSA(
        drones=opt_drones,
        B=arena['B'], T=arena['T'],
        c1=arena['c1'], c2=arena['c2'],
        noise_bound=arena['noise_bound'],
        seed=0,
    )
    # Baseline algorithm — same initial state, separate drone instances
    baseline_drones = [
        CoverageDrone(name=d.name, q=d.q_fallback, c=d.c,
                       r_star=d.r_star, r_max=d.r_max, r=0.0)
        for d in arena['drones']
    ]
    baseline = LagrangianBaseline(
        drones=baseline_drones,
        B=arena['B'], T=arena['T'],
        c1=arena['c1'], c2=arena['c2'],
        noise_bound=arena['noise_bound'],
        seed=0,
    )

    # Run loop
    N = len(opt_drones)
    qi_history = np.zeros((K_max, N), dtype=np.float32)
    rk_history = np.zeros((K_max, N), dtype=np.float32)
    rstar_history = np.zeros((K_max, N), dtype=np.float32)
    gate_history = np.zeros(K_max, dtype=np.int32)
    constraint_history = np.zeros(K_max, dtype=np.float32)
    # Baseline parallel history
    rk_baseline_history = np.zeros((K_max, N), dtype=np.float32)
    constraint_baseline_history = np.zeros(K_max, dtype=np.float32)
    lambda_baseline_history = np.zeros(K_max, dtype=np.float32)
    times = np.zeros(K_max, dtype=np.float32)

    # Sample-and-hold for slower-than-optimizer estimator rate
    last_qi_t = -1e9
    cached_qi = None

    c_array = np.array([d.c for d in arena['drones']])
    r_max_array = np.array([d.r_max for d in arena['drones']])
    r_star_array = np.array([d.r_star for d in arena['drones']])

    # Drone reach diagnostic: track three reach metrics per sector per round.
    # - r_fire_outer: max radial distance from origin to any hot cell in sector
    # - r_fire_mean : mean radial distance of hot cells in sector
    # - coverage_frac: fraction of hot cells in sector within drone's leash circle
    rfire_outer_history = np.zeros((K_max, N), dtype=np.float32)
    rfire_mean_history = np.zeros((K_max, N), dtype=np.float32)
    coverage_frac_history = np.zeros((K_max, N), dtype=np.float32)
    fire_threshold = 5.0  # degC above ambient to count as "hot"
    # Coarse grid for reach computation (cheap, called every round)
    reach_grid_n = 60
    reach_xs = np.linspace(-args.r_integration, args.r_integration, reach_grid_n)
    reach_ys = np.linspace(-args.r_integration, args.r_integration, reach_grid_n)
    RX, RY = np.meshgrid(reach_xs, reach_ys)
    RR = np.sqrt(RX * RX + RY * RY)
    RPHI = np.arctan2(RY, RX)
    sector_masks = []
    for sec in sectors:
        rel = (RPHI - sec.phi_mid + math.pi) % (2.0 * math.pi) - math.pi
        sector_masks.append((RR <= args.r_integration) & (np.abs(rel) <= sec.phi_half))

    for k in range(K_max):
        t = k * dt
        times[k] = t
        # Update q_i (sample-and-hold if estimator rate < optimizer rate)
        if cached_qi is None or (t - last_qi_t) >= qi_period - 1e-9:
            cached_qi = compute_sector_qi(field, t, sectors, threshold=qi_threshold)
            last_qi_t = t
        qi_history[k] = cached_qi
        for i, d in enumerate(opt_drones):
            d.q = float(cached_qi[i])

        # Frozen-q_i KKT reference
        rstar_history[k] = solve_static_kkt(
            cached_qi, c_array, r_star_array, r_max_array, arena['B'],
        )

        # Reach metrics per sector. NB: this uses the OPTIMIZER's r^k from the
        # PREVIOUS round (the leash currently in effect for this round's coverage).
        T_grid = field.evaluate(RX, RY, t)
        hot_mask = T_grid > (field.ambient + fire_threshold)
        prev_rk = rk_history[k - 1] if k > 0 else np.zeros(N)
        for i, mask in enumerate(sector_masks):
            combined = mask & hot_mask
            if combined.any():
                hot_radii = RR[combined]
                rfire_outer_history[k, i] = float(hot_radii.max())
                rfire_mean_history[k, i] = float(hot_radii.mean())
                within_leash = hot_radii <= prev_rk[i]
                coverage_frac_history[k, i] = float(within_leash.sum()) / float(hot_radii.size)
            else:
                rfire_outer_history[k, i] = 0.0
                rfire_mean_history[k, i] = 0.0
                coverage_frac_history[k, i] = 1.0  # no fire to cover

        # Update q on the baseline drones too (same q)
        for i, d in enumerate(baseline_drones):
            d.q = float(cached_qi[i])

        # Run one optimizer round
        result = optimizer.run_round(k)
        rk_history[k] = result.radii
        gate_history[k] = result.gate_state
        constraint_history[k] = sum(d.c * d.r ** 2 for d in opt_drones)

        # Run baseline round in parallel
        result_b = baseline.run_round(k)
        rk_baseline_history[k] = result_b.radii
        constraint_baseline_history[k] = sum(
            d.c * d.r ** 2 for d in baseline_drones
        )
        lambda_baseline_history[k] = result_b.lambda_k

    lag_history = np.abs(rk_history - rstar_history)
    diags = diagnostics(qi_history, lag_history)
    print("Diagnostics:")
    print(f"  K_total                = {diags['K_total']}")
    print(f"  argmax sector transitions = {diags['argmax_transitions']}")
    print(f"  per-sector q_i range   = {[f'{v:.2f}' for v in diags['per_sector_range']]}")
    print(f"  per-sector q_i cov     = {[f'{v:.2f}' for v in diags['per_sector_cov']]}")
    print(f"  max lag                = {diags['max_lag']:.3f} m")
    print(f"  mean lag (full run)    = {diags['mean_lag_full']:.3f} m")
    print(f"  mean lag (second half) = {diags['mean_lag_second_half']:.3f} m")
    print(f"  gate state mean (0=infeasible, 1=feasible) = {gate_history.mean():.3f}")
    print(f"  max constraint Σc_i r_i² = {constraint_history.max():.3f}  (B = {arena['B']})")
    print()
    # ---- Comparison vs Lagrangian baseline ----
    print("Constraint comparison — per-iterate Σc_i (r_i^k)^2 − B:")
    fed_violation = np.maximum(0.0, constraint_history - arena['B'])
    base_violation = np.maximum(0.0, constraint_baseline_history - arena['B'])
    print(f"  FedDCSA:  max viol = {float(fed_violation.max()):.3f}  "
          f"mean viol = {float(fed_violation.mean()):.4f}  "
          f"rounds violating = {int(np.sum(fed_violation > 1e-6))}/{K_max}")
    print(f"  Baseline: max viol = {float(base_violation.max()):.3f}  "
          f"mean viol = {float(base_violation.mean()):.4f}  "
          f"rounds violating = {int(np.sum(base_violation > 1e-6))}/{K_max}")
    print(f"  Baseline-vs-FedDCSA max violation ratio = "
          f"{(float(base_violation.max()) / max(float(fed_violation.max()), 1e-9)):.2f}x")
    print()

    # Reach diagnostics: per-sector summary
    print("Drone reach diagnostics (per sector):")
    for i, d in enumerate(arena['drones']):
        outer = rfire_outer_history[:, i]
        mean_r = rfire_mean_history[:, i]
        cov = coverage_frac_history[:, i]
        active_mask = outer > 0.01
        if active_mask.any():
            mean_cov_when_active = float(cov[active_mask].mean())
            max_outer = float(outer[active_mask].max())
            mean_mean = float(mean_r[active_mask].mean())
            print(f"  {d.name}: fire-active rounds = {int(active_mask.sum())}/{K_max}  "
                  f"|  mean coverage = {mean_cov_when_active*100:.0f}%  "
                  f"|  max r_fire_outer = {max_outer:.2f} m  "
                  f"|  mean r_fire_centroid = {mean_mean:.2f} m")
        else:
            print(f"  {d.name}: no fire in this sector during run")
    print()

    # ---------------------------------------------------------------------
    # Plot 6-panel figure (5 rows × 12 cols for clean subplot math)
    # ---------------------------------------------------------------------
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(17, 19))
    gs = fig.add_gridspec(6, 12, hspace=0.55, wspace=0.45)

    # Panel 1: 6 truth snapshots across the run
    snap_times = np.linspace(0, args.duration, 6)
    grid_n = 80
    arena_R = args.r_integration + 0.5
    xs = np.linspace(-arena_R, arena_R, grid_n)
    ys = np.linspace(-arena_R, arena_R, grid_n)
    X, Y = np.meshgrid(xs, ys)
    vmin = field.ambient
    vmax = field.ambient + field.fire.peak_temperature
    fuel_R = getattr(field.fire, 'arena_radius', 0.0) or args.r_integration
    for si, st in enumerate(snap_times):
        ax = fig.add_subplot(gs[0, si * 2:(si + 1) * 2])
        T = field.evaluate(X, Y, float(st))
        im = ax.imshow(T, extent=(-arena_R, arena_R, -arena_R, arena_R),
                       origin='lower', cmap='inferno', vmin=vmin, vmax=vmax)
        ax.scatter([field.fire.ignition_x], [field.fire.ignition_y],
                   s=20, c='cyan', marker='*', edgecolors='white', linewidth=0.5)
        # Sector wedges
        for sec in sectors:
            for sign in (-1, +1):
                phi = sec.phi_mid + sign * sec.phi_half
                ax.plot([0, args.r_integration * math.cos(phi)],
                        [0, args.r_integration * math.sin(phi)],
                        color='white', alpha=0.3, linewidth=0.5)
        # Fuel disc boundary (= drone r_max)
        if fuel_R > 0.0:
            theta = np.linspace(0, 2 * math.pi, 100)
            ax.plot(fuel_R * np.cos(theta), fuel_R * np.sin(theta),
                    color='cyan', alpha=0.5, linewidth=0.8, linestyle='--')
        ax.set_title(f't={st:.0f}s', fontsize=9)
        ax.set_xticks([])
        ax.set_yticks([])
    # colorbar
    cbar = fig.colorbar(im, ax=fig.axes[-6:], shrink=0.7, pad=0.02, aspect=8)
    cbar.set_label('°C', fontsize=8)

    # Panel 2: q_i(t) per sector
    ax2 = fig.add_subplot(gs[1, :])
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']
    for i, d in enumerate(arena['drones']):
        ax2.plot(times, qi_history[:, i], color=colors[i], label=f"{d.name}  q_i(t)", linewidth=1.5)
    ax2.set_xlabel('time (s)')
    ax2.set_ylabel('q_i(t)')
    ax2.set_title('q_i(t) per sector — sectors take turns as the wavefront sweeps + cells burn out')
    ax2.legend(loc='upper right', fontsize=8, ncol=4)
    ax2.grid(alpha=0.3)

    # Panel 3: r_i^k(t) chase vs r_i^*(t) reference
    ax3 = fig.add_subplot(gs[2, :])
    for i, d in enumerate(arena['drones']):
        ax3.plot(times, rk_history[:, i], color=colors[i], label=f"{d.name} r_i^k", linewidth=1.5)
        ax3.plot(times, rstar_history[:, i], color=colors[i],
                 linestyle='--', alpha=0.6, label=f"{d.name} r_i*", linewidth=1.0)
    ax3.set_xlabel('time (s)')
    ax3.set_ylabel('radius (m)')
    ax3.set_title('FedDCSA r_i^k(t)  (solid)  vs  instantaneous static optimum r_i^*(t)  (dashed)')
    ax3.legend(loc='upper right', fontsize=8, ncol=4)
    ax3.grid(alpha=0.3)
    ax3.set_ylim(bottom=0)

    # Panel 4: lag-vs-K with γ_K annotation
    ax4 = fig.add_subplot(gs[3, :8])
    K_axis = np.arange(K_max)
    for i, d in enumerate(arena['drones']):
        ax4.plot(K_axis, lag_history[:, i], color=colors[i], label=f"{d.name}", linewidth=1.2)
    ax4.set_xlabel('K (optimizer round)')
    ax4.set_ylabel('|r_i^k - r_i^*|  (m)')
    ax4.set_title('Lag vs K — bounded lag = chase succeeds')
    ax4.legend(loc='upper right', fontsize=8, ncol=4)
    ax4.grid(alpha=0.3)
    # Step size annotation on right axis
    ax4_r = ax4.twinx()
    gamma_K = arena['c1'] / np.sqrt(K_axis + 1)
    ax4_r.plot(K_axis, gamma_K, color='gray', linestyle=':', alpha=0.5, linewidth=1)
    ax4_r.set_ylabel('γ_K  (gray dotted)', color='gray', fontsize=8)
    ax4_r.tick_params(axis='y', colors='gray', labelsize=7)

    # Panel 5: constraint value vs B
    ax5 = fig.add_subplot(gs[3, 8:])
    ax5.plot(K_axis, constraint_history, color='black', linewidth=1.2, label='Σ c_i r_i^2')
    ax5.axhline(arena['B'], color='red', linestyle='--', alpha=0.7, label=f'B = {arena["B"]}')
    ax5.set_xlabel('K')
    ax5.set_ylabel('Σ c_i (r_i^k)^2')
    ax5.set_title('Constraint value — FedDCSA feasibility')
    ax5.legend(loc='upper right', fontsize=8)
    ax5.grid(alpha=0.3)

    # ---- NEW: Row 5 — Baseline comparison panels ----
    K_axis = np.arange(K_max)
    eta_history = arena['c2'] / np.sqrt(K_axis + 1)
    # Panel 7a: Σc_i r² for both algorithms, with B and B+η_K reference
    ax7a = fig.add_subplot(gs[5, :8])
    ax7a.plot(K_axis, constraint_history, color='tab:blue', linewidth=1.5,
              label='FedDCSA: Σc_i (r_i^k)²')
    ax7a.plot(K_axis, constraint_baseline_history, color='tab:red',
              linewidth=1.5, linestyle='--',
              label='Lagrangian baseline: Σc_i (r_i^k)²')
    ax7a.axhline(arena['B'], color='black', linestyle='-', alpha=0.7,
                 linewidth=1, label=f'B = {arena["B"]}')
    ax7a.plot(K_axis, arena['B'] + eta_history, color='gray', linestyle=':',
              alpha=0.7, linewidth=1, label='B + η_K (FedDCSA gate bound)')
    ax7a.set_xlabel('K')
    ax7a.set_ylabel('Σ c_i (r_i^k)^2')
    ax7a.set_title('Per-iterate constraint value — FedDCSA stays at/under B; baseline overshoots')
    ax7a.legend(loc='upper left', fontsize=8)
    ax7a.grid(alpha=0.3)
    # Panel 7b: per-round violation magnitude (only positive overshoot)
    ax7b = fig.add_subplot(gs[5, 8:])
    ax7b.plot(K_axis, fed_violation, color='tab:blue', linewidth=1.5,
              label='FedDCSA violation')
    ax7b.plot(K_axis, base_violation, color='tab:red', linewidth=1.5,
              linestyle='--', label='Baseline violation')
    ax7b.fill_between(K_axis, 0, base_violation, color='tab:red', alpha=0.15)
    ax7b.set_xlabel('K')
    ax7b.set_ylabel('max(0, Σc_i r² − B)')
    ax7b.set_title('Per-iterate VIOLATION magnitude (the headline paper figure)')
    ax7b.legend(loc='upper left', fontsize=8)
    ax7b.grid(alpha=0.3)

    # Panel 6: drone reach vs fire extent + coverage fraction — 4 subplots, one per sector
    for i, d in enumerate(arena['drones']):
        ax6 = fig.add_subplot(gs[4, i * 3:(i + 1) * 3])
        # Primary axis: radii
        ax6.plot(times, rfire_outer_history[:, i], color=colors[i], linestyle='-',
                 linewidth=1.0, alpha=0.4, label='r_fire outer')
        ax6.plot(times, rfire_mean_history[:, i], color=colors[i], linestyle='-',
                 linewidth=1.5, label='r_fire centroid')
        ax6.plot(times, rk_history[:, i], color=colors[i], linestyle='--',
                 linewidth=1.5, label='r_i^k (leash)')
        ax6.axhline(r_max_array[i], color='black', alpha=0.4, linestyle=':',
                    linewidth=0.8, label=f'r_max={r_max_array[i]:.1f}')
        ax6.set_xlabel('time (s)')
        ax6.set_ylabel('radius (m)')
        ax6.set_title(f'{d.name}: reach vs fire + coverage', fontsize=9)
        ax6.set_ylim(0, max(2.0, float(r_max_array[i]) + 0.1))
        ax6.legend(loc='upper left', fontsize=6)
        ax6.grid(alpha=0.3)
        # Secondary axis: coverage fraction
        ax6_r = ax6.twinx()
        ax6_r.fill_between(times, 0, coverage_frac_history[:, i],
                           color=colors[i], alpha=0.15)
        ax6_r.plot(times, coverage_frac_history[:, i], color=colors[i],
                   linestyle=':', linewidth=0.8, alpha=0.6)
        ax6_r.set_ylim(0, 1.05)
        ax6_r.set_ylabel('coverage frac', fontsize=7, color=colors[i], alpha=0.7)
        ax6_r.tick_params(axis='y', labelsize=6, colors=colors[i])

    fig.suptitle(
        f'Phase A preview — fire model + q_i + FedDCSA   '
        f'(K_max={K_max}, transitions={diags["argmax_transitions"]}, '
        f'mean lag (2nd half)={diags["mean_lag_second_half"]:.3f} m)',
        fontsize=11, y=0.995,
    )

    plt.savefig(args.out, dpi=120, bbox_inches='tight')
    print(f"Saved 5-panel figure → {args.out}")


if __name__ == '__main__':
    main()
