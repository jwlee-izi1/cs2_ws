#!/usr/bin/env python3
"""Drone simulation plots for CDC 2026 paper (Section IV).

4-drone coverage problem:
  fi(xi) = qi * (1 - xi)^2    (each drone wants full coverage)
  gi(xi) = ei * xi - b/m      (resource contribution)
  G(x) = sum gi(xi) <= 0      (coupled resource constraint)

Gate condition (paper Eq. 2):
  b_k = 1 if G_tilde <= eta_k else 0

Produces 6 plots in fed_dcsa_logs/drone_simulation/:
  exact_feasibility_N300K, exact_feasibility_zoom, exact_rate
  stoch_feasibility_N300K,  stoch_feasibility_zoom,  stoch_rate

Usage:
    python3 run_drone_simulation.py
"""

import math
import os
import time

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Drone parameters (Section IV of paper)
# ---------------------------------------------------------------------------
DRONES = [
    {'q': 0.5, 'e': 0.3, 'b_over_m': 0.315},
    {'q': 0.4, 'e': 0.4, 'b_over_m': 0.315},
    {'q': 0.3, 'e': 0.5, 'b_over_m': 0.315},
    {'q': 0.2, 'e': 0.6, 'b_over_m': 0.315},
]
# b = 1.26, m = 4, b/m = 0.315
# x0 = (1, 1, 1, 1), G(x0) = 0.54 > eta_0 = 0.4 (correction transient visible)

T = 15
C1 = 0.003
C2 = 0.4
NOISE_HALF = 0.3   # stochastic noise ~ Uniform[-0.3, +0.3], independent per agent/step
SEED = 42

# Drift factor = 2 * L_G * M / mu
# Exact:      L_G=0.9274, M=1.536  -> 2*0.9274*1.536 = 2.851
# Stochastic: L_G=0.9274, M_bar=2.127 -> 2*0.9274*2.127 = 3.945
DRIFT_FACTOR_EXACT = 2.851
DRIFT_FACTOR_STOCH = 3.945

N_FEAS = 300_000
N_VALUES_RATE = [30_000, 100_000, 300_000, 1_000_000, 3_000_000]

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'fed_dcsa_logs', 'drone_simulation')


# ---------------------------------------------------------------------------
# KKT solution
# ---------------------------------------------------------------------------
def compute_kkt(drones):
    """Solve KKT for fi(xi)=qi(1-xi)^2, gi(xi)=ei*xi - b/m.

    KKT: 2qi(xi*-1) + lambda*ei = 0  => xi* = 1 - lambda*ei/(2qi)
    G(x*)=0: sum ei*(1 - lambda*ei/(2qi)) = b
             sum ei - lambda*sum(ei^2/(2qi)) = b
             lambda* = (sum_ei - b) / sum(ei^2/(2qi))
    """
    sum_ei = sum(d['e'] for d in drones)
    b_total = sum(d['b_over_m'] for d in drones)  # = 1.26
    num = sum_ei - b_total
    den = sum(d['e'] ** 2 / (2.0 * d['q']) for d in drones)
    lambda_star = num / den

    x_star = [1.0 - lambda_star * d['e'] / (2.0 * d['q']) for d in drones]
    F_star = sum(d['q'] * (xs - 1.0) ** 2 for d, xs in zip(drones, x_star))
    G_star = sum(d['e'] * xs - d['b_over_m'] for d, xs in zip(drones, x_star))

    assert all(0.0 < xs < 1.0 for xs in x_star), f'x* not in (0,1): {x_star}'
    assert abs(G_star) < 1e-10, f'G(x*)={G_star} not zero'

    return {
        'x_star': x_star,
        'lambda_star': lambda_star,
        'F_star': F_star,
        'G_star': G_star,
    }


# ---------------------------------------------------------------------------
# Algorithm 1
# ---------------------------------------------------------------------------
def run_algorithm(drones, K, T_val, kkt, c1, c2, x0_val, drift_factor,
                  noise_half=0.0, seed=None):
    """Run Algorithm 1 with the new gate: b_k = 1 if G_tilde <= eta_k.

    Parameters
    ----------
    drones       : list of drone parameter dicts
    K            : number of rounds
    T_val        : inner steps per round
    kkt          : KKT solution dict
    c1, c2       : stepsize / tolerance constants
    x0_val       : initial point (scalar, same for all drones)
    drift_factor : 2*L_G*M_bar/mu (exact or stochastic)
    noise_half   : half-width of uniform noise (0 = exact)
    seed         : RNG seed (used only if noise_half > 0)

    Returns
    -------
    dict with log_G, log_eta, log_eta_plus_drift, running_gaps,
              final_gap, final_G_bar, feas_count
    """
    rng = np.random.default_rng(seed) if noise_half > 0 else None

    m = len(drones)
    q = [d['q'] for d in drones]
    e = [d['e'] for d in drones]
    bm = [d['b_over_m'] for d in drones]
    N = K * T_val
    F_star = kkt['F_star']

    log_G = np.empty(N, dtype=np.float64)
    log_eta = np.empty(N, dtype=np.float64)
    log_eta_plus_drift = np.empty(N, dtype=np.float64)

    s_round = math.ceil(K / 2)
    x_sum = [0.0] * m
    feas_count = 0
    running_gaps = []

    x = [x0_val] * m

    idx = 0
    for k in range(K):
        sqrt_k1 = math.sqrt(k + 1)
        gamma_k = c1 / sqrt_k1
        eta_k = c2 / sqrt_k1
        r_drift_k = drift_factor * T_val * gamma_k

        # Gate: compare G_tilde to eta_k only (new paper Eq. 2)
        G_tilde = sum(e[i] * x[i] - bm[i] for i in range(m))
        b_k = 1 if G_tilde <= eta_k else 0

        for t in range(T_val):
            log_G[idx] = sum(e[i] * x[i] - bm[i] for i in range(m))
            log_eta[idx] = eta_k
            log_eta_plus_drift[idx] = eta_k + r_drift_k

            if k >= s_round and b_k == 1:
                for i in range(m):
                    x_sum[i] += x[i]
                feas_count += 1

            for i in range(m):
                noise_i = rng.uniform(-noise_half, noise_half) if rng else 0.0
                if b_k == 1:
                    h = 2.0 * q[i] * (x[i] - 1.0) + noise_i   # grad fi + noise
                else:
                    h = e[i] + noise_i                           # grad gi + noise
                x[i] = max(0.0, min(1.0, x[i] - gamma_k * h))

            idx += 1

        # Running gap tracking (from round s onward)
        if k >= s_round and feas_count > 0:
            x_bar_k = [x_sum[i] / feas_count for i in range(m)]
            gap_k = sum(q[i] * (x_bar_k[i] - 1.0) ** 2 for i in range(m)) - F_star
            running_gaps.append(((k + 1) * T_val, gap_k))

    # Final average
    if feas_count > 0:
        x_bar = [x_sum[i] / feas_count for i in range(m)]
        final_gap = sum(q[i] * (x_bar[i] - 1.0) ** 2 for i in range(m)) - F_star
        final_G_bar = sum(e[i] * x_bar[i] - bm[i] for i in range(m))
    else:
        final_gap = float('nan')
        final_G_bar = float('nan')

    return {
        'log_G': log_G,
        'log_eta': log_eta,
        'log_eta_plus_drift': log_eta_plus_drift,
        'running_gaps': running_gaps,
        'final_gap': final_gap,
        'final_G_bar': final_G_bar,
        'feas_count': feas_count,
    }


# ---------------------------------------------------------------------------
# Plot: feasibility
# ---------------------------------------------------------------------------
def plot_feasibility(log_G, log_eta, log_eta_plus_drift, N, title, fname,
                     output_dir, zoom=False, zoom_end=20_000):
    """Plot G(x^{k,t}), eta_k, and eta_k + r_drift_k vs iteration."""
    fig, ax = plt.subplots(figsize=(8, 4.5))

    if zoom:
        end = min(zoom_end, N)
        ns = np.arange(end)
        G_plot = log_G[:end]
        eta_plot = log_eta[:end]
        eta_drift_plot = log_eta_plus_drift[:end]
    else:
        if N > 50_000:
            step = max(1, N // 50_000)
            ns = np.arange(0, N, step)
            G_plot = log_G[::step]
            eta_plot = log_eta[::step]
            eta_drift_plot = log_eta_plus_drift[::step]
        else:
            ns = np.arange(N)
            G_plot = log_G
            eta_plot = log_eta
            eta_drift_plot = log_eta_plus_drift

    ax.plot(ns, G_plot, 'k-', linewidth=0.5, alpha=0.8, label=r'$G(x^{k,t})$')
    ax.plot(ns, eta_plot, 'r--', linewidth=1.2, label=r'$\eta_k$')
    ax.plot(ns, eta_drift_plot, 'b:', linewidth=1.2,
            label=r'$\eta_k + r_k$')
    ax.axhline(y=0.0, color='gray', linestyle='-', linewidth=0.5)

    ax.legend(fontsize=9, loc='upper right')
    ax.set_xlabel('Iteration', fontsize=12)
    ax.set_ylabel(r'$G(x^{k,t})$', fontsize=12)
    ax.set_title(title, fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=10)

    plt.tight_layout()
    for ext in ['pdf', 'png']:
        fig.savefig(os.path.join(output_dir, f'{fname}.{ext}'),
                    dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved {fname}.pdf/.png')


# ---------------------------------------------------------------------------
# Plot: convergence rate
# ---------------------------------------------------------------------------
def plot_rate(N_vals, gaps, fname, output_dir, title):
    """Log-log plot of |F(x_bar) - F*| vs N with -1/2 reference line."""
    Ns = np.array(N_vals, dtype=float)
    vals = np.abs(np.array(gaps, dtype=float))

    pos = vals > 0
    if not np.any(pos):
        print(f'  WARNING: No positive values for rate plot {fname}')
        return

    fig, ax = plt.subplots(figsize=(7, 5))

    ax.loglog(Ns[pos], vals[pos], 'bo-', linewidth=1.5, markersize=7,
              label=r'$|F(\bar{x}_\eta) - F^\star|$')

    c_fit = np.median([v * math.sqrt(n) for n, v in zip(Ns[pos], vals[pos])])
    n_ref = np.logspace(np.log10(Ns.min() * 0.5), np.log10(Ns.max() * 2.0), 200)
    ax.loglog(n_ref, c_fit / np.sqrt(n_ref), 'k--', linewidth=1.0, alpha=0.6,
              label=r'$\mathcal{O}(1/\sqrt{N})$')

    ax.set_xlabel(r'$N$ (total inner updates)', fontsize=12)
    ax.set_ylabel(r'$|F(\bar{x}_\eta) - F^\star|$', fontsize=12)
    ax.set_title(title, fontsize=12)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, which='both')
    ax.tick_params(labelsize=10)

    plt.tight_layout()
    for ext in ['pdf', 'png']:
        fig.savefig(os.path.join(output_dir, f'{fname}.{ext}'),
                    dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved {fname}.pdf/.png')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    output_dir = os.path.abspath(OUTPUT_DIR)
    os.makedirs(output_dir, exist_ok=True)

    kkt = compute_kkt(DRONES)
    x0_G = sum(d['e'] * 1.0 - d['b_over_m'] for d in DRONES)

    print('=' * 60)
    print('Drone Simulation — CDC 2026 Paper Figures')
    print('=' * 60)
    print(f'  lambda* = {kkt["lambda_star"]:.4f}')
    for i, xs in enumerate(kkt['x_star']):
        print(f'  x{i+1}*    = {xs:.4f}')
    print(f'  F*      = {kkt["F_star"]:.6f}')
    print(f'  G(x*)   = {kkt["G_star"]:.2e}')
    print(f'  G(x0)   = {x0_G:.4f}  (x0 = all-ones)')
    print(f'  eta_0   = {C2:.4f}')
    print(f'  r_drift_0 (exact) = {DRIFT_FACTOR_EXACT*T*C1:.4f}')
    print(f'  r_drift_0 (stoch) = {DRIFT_FACTOR_STOCH*T*C1:.4f}')
    print()

    # -----------------------------------------------------------------------
    # Exact subgradients
    # -----------------------------------------------------------------------
    print('--- Exact subgradients ---')

    # Feasibility run (N=300K)
    K_feas = math.ceil(N_FEAS / T)
    N_actual = K_feas * T
    print(f'  Running N={N_actual:,} for feasibility plots...')
    t0 = time.time()
    res_feas = run_algorithm(DRONES, K_feas, T, kkt, C1, C2,
                             x0_val=1.0, drift_factor=DRIFT_FACTOR_EXACT,
                             noise_half=0.0, seed=None)
    print(f'  Elapsed: {time.time()-t0:.1f}s')
    print(f'  F gap = {res_feas["final_gap"]:.6f}, '
          f'G(x_bar) = {res_feas["final_G_bar"]:.6f}, '
          f'feas_count = {res_feas["feas_count"]:,}')

    plot_feasibility(
        res_feas['log_G'], res_feas['log_eta'], res_feas['log_eta_plus_drift'],
        N_actual,
        title=r'Feasibility (exact) — $N=300{,}000$',
        fname='exact_feasibility_N300K',
        output_dir=output_dir, zoom=False)

    plot_feasibility(
        res_feas['log_G'], res_feas['log_eta'], res_feas['log_eta_plus_drift'],
        N_actual,
        title=r'Feasibility zoomed (exact) — first 20K iterations',
        fname='exact_feasibility_zoom',
        output_dir=output_dir, zoom=True)

    # Rate sweep
    print('  Running rate sweep (exact)...')
    final_gaps_exact = []
    final_Ns_exact = []
    for N_target in N_VALUES_RATE:
        K_r = math.ceil(N_target / T)
        N_r = K_r * T
        if N_r == N_actual:
            final_gaps_exact.append(res_feas['final_gap'])
            final_Ns_exact.append(N_r)
            print(f'    N={N_r:,}: F gap = {res_feas["final_gap"]:.6f}  (reused)')
            continue
        res_r = run_algorithm(DRONES, K_r, T, kkt, C1, C2,
                              x0_val=1.0, drift_factor=DRIFT_FACTOR_EXACT,
                              noise_half=0.0, seed=None)
        final_gaps_exact.append(res_r['final_gap'])
        final_Ns_exact.append(N_r)
        print(f'    N={N_r:,}: F gap = {res_r["final_gap"]:.6f}')

    plot_rate(final_Ns_exact, final_gaps_exact,
              fname='exact_rate', output_dir=output_dir,
              title='Rate (exact subgradients)')
    print()

    # -----------------------------------------------------------------------
    # Stochastic subgradients
    # -----------------------------------------------------------------------
    print('--- Stochastic subgradients ---')

    # Feasibility run (N=300K)
    print(f'  Running N={N_actual:,} for feasibility plots...')
    t0 = time.time()
    res_feas_s = run_algorithm(DRONES, K_feas, T, kkt, C1, C2,
                               x0_val=1.0, drift_factor=DRIFT_FACTOR_STOCH,
                               noise_half=NOISE_HALF, seed=SEED)
    print(f'  Elapsed: {time.time()-t0:.1f}s')
    print(f'  F gap = {res_feas_s["final_gap"]:.6f}, '
          f'G(x_bar) = {res_feas_s["final_G_bar"]:.6f}, '
          f'feas_count = {res_feas_s["feas_count"]:,}')

    plot_feasibility(
        res_feas_s['log_G'], res_feas_s['log_eta'], res_feas_s['log_eta_plus_drift'],
        N_actual,
        title=r'Feasibility (stochastic) — $N=300{,}000$',
        fname='stoch_feasibility_N300K',
        output_dir=output_dir, zoom=False)

    plot_feasibility(
        res_feas_s['log_G'], res_feas_s['log_eta'], res_feas_s['log_eta_plus_drift'],
        N_actual,
        title=r'Feasibility zoomed (stochastic) — first 3K iterations',
        fname='stoch_feasibility_zoom',
        output_dir=output_dir, zoom=True, zoom_end=3_000)

    # Rate sweep
    print('  Running rate sweep (stochastic)...')
    final_gaps_stoch = []
    final_Ns_stoch = []
    for N_target in N_VALUES_RATE:
        K_r = math.ceil(N_target / T)
        N_r = K_r * T
        if N_r == N_actual:
            final_gaps_stoch.append(res_feas_s['final_gap'])
            final_Ns_stoch.append(N_r)
            print(f'    N={N_r:,}: F gap = {res_feas_s["final_gap"]:.6f}  (reused)')
            continue
        res_r = run_algorithm(DRONES, K_r, T, kkt, C1, C2,
                              x0_val=1.0, drift_factor=DRIFT_FACTOR_STOCH,
                              noise_half=NOISE_HALF, seed=SEED)
        final_gaps_stoch.append(res_r['final_gap'])
        final_Ns_stoch.append(N_r)
        print(f'    N={N_r:,}: F gap = {res_r["final_gap"]:.6f}')

    plot_rate(final_Ns_stoch, final_gaps_stoch,
              fname='stoch_rate', output_dir=output_dir,
              title='Rate (stochastic subgradients)')
    print()

    print('=' * 60)
    print(f'All plots saved to: {output_dir}/')
    print('=' * 60)


if __name__ == '__main__':
    main()
