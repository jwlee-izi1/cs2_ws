#!/usr/bin/env python3
"""Analysis plots comparing Regime A (negative threshold) vs Regime B (positive threshold).

Same algorithm and agents as run_synthetic.py, just different (c1, c2) choices.
Produces 8 plots in fed_dcsa_logs/resultanalysis/:
  Regime A (c2=0.2):  feasibility, feasibility_zoom, convergence_vs_iter, rate
  Regime B (c2=0.30): feasibility, feasibility_zoom, convergence_vs_iter, rate

Usage:
    python3 run_analysis_plots.py
"""

import math
import os
import sys
import time

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Agent parameters (identical to run_synthetic.py)
# ---------------------------------------------------------------------------
AGENTS = [
    {'alpha': 10.0, 'a': 0.5, 'e': 0.3, 'b': 0.1},
    {'alpha': 8.0,  'a': 0.5, 'e': 0.4, 'b': 0.1},
]

DELTA = 0.1
T = 15
X0 = 0.6  # initial point x_i = 0.6 for all agents, G(0.6,0.6) = 0.22

REGIMES = {
    'A': {'c1': 0.003, 'c2': 0.2,  'x0': 0.6,   'label': 'Regime A ($c_2=0.2$, threshold $< 0$)'},
    'B': {'c1': 0.003, 'c2': 0.30, 'x0': 0.857, 'label': 'Regime B ($c_2=0.30$, threshold $> 0$)'},
}

N_FEAS = 300_000  # N for feasibility + convergence-vs-iter plots
N_VALUES_RATE = [30_000, 100_000, 300_000, 1_000_000, 3_000_000]

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', '..',
                          'fed_dcsa_logs', 'resultanalysis')


# ---------------------------------------------------------------------------
# KKT solution and constants
# ---------------------------------------------------------------------------
def compute_constants(agents):
    m = len(agents)
    G_a = sum(ag['e'] * ag['a'] - ag['b'] for ag in agents)

    if G_a > 0:
        num = sum(ag['e'] * ag['a'] - ag['b'] for ag in agents)
        den = sum(ag['e'] ** 2 / ag['alpha'] for ag in agents)
        lambda_star = num / den
        x_star = [ag['a'] - lambda_star * ag['e'] / ag['alpha'] for ag in agents]
    else:
        lambda_star = 0.0
        x_star = [ag['a'] for ag in agents]

    F_star = sum(0.5 * ag['alpha'] * (xs - ag['a']) ** 2
                 for ag, xs in zip(agents, x_star))
    G_star = sum(ag['e'] * xs - ag['b'] for ag, xs in zip(agents, x_star))

    L_g = [ag['e'] for ag in agents]
    L_G = math.sqrt(sum(e ** 2 for e in L_g))
    L_f = [ag['alpha'] * max(ag['a'], 1.0 - ag['a']) for ag in agents]
    M_i = [max(lf, lg) for lf, lg in zip(L_f, L_g)]
    M = math.sqrt(sum(mi ** 2 for mi in M_i))
    D_x_sq = sum(max(0.5 * xs ** 2, 0.5 * (xs - 1.0) ** 2) for xs in x_star)
    D_x = math.sqrt(D_x_sq)
    mu = 1.0
    T_max = M / (4.0 * DELTA * L_G)

    return {
        'x_star': x_star, 'lambda_star': lambda_star,
        'F_star': F_star, 'G_star': G_star,
        'L_G': L_G, 'M': M, 'D_x': D_x, 'mu': mu, 'T_max': T_max,
    }


# ---------------------------------------------------------------------------
# Algorithm 1 — with running gap tracking
# ---------------------------------------------------------------------------
def run_algorithm(agents, K, T, constants, c1, c2, x0_val):
    """Run Algorithm 1, returning per-iteration G, eta, and per-round running F gap."""
    m = len(agents)
    alpha = [ag['alpha'] for ag in agents]
    a_vals = [ag['a'] for ag in agents]
    e = [ag['e'] for ag in agents]
    b = [ag['b'] for ag in agents]
    N = K * T
    F_star = constants['F_star']
    L_G = constants['L_G']
    M = constants['M']
    mu = constants['mu']

    log_G = np.empty(N, dtype=np.float64)
    log_eta = np.empty(N, dtype=np.float64)

    s_round = math.ceil(K / 2)
    x_sum = [0.0] * m
    feas_count = 0

    # Running gap: (iteration, gap) pairs
    running_gaps = []

    x = [x0_val for _ in agents]

    idx = 0
    for k in range(K):
        sqrt_k1 = math.sqrt(k + 1)
        gamma_k = c1 / sqrt_k1
        eta_k = c2 / sqrt_k1
        r_drift_k = (2.0 * L_G * M / mu) * T * gamma_k

        G_tilde = sum(e[i] * x[i] - b[i] for i in range(m))
        b_k = 1 if G_tilde <= eta_k - r_drift_k else 0

        for t in range(T):
            log_G[idx] = sum(e[i] * x[i] - b[i] for i in range(m))
            log_eta[idx] = eta_k

            if idx >= s_round * T and b_k == 1:
                for i in range(m):
                    x_sum[i] += x[i]
                feas_count += 1

            for i in range(m):
                if b_k == 1:
                    h = alpha[i] * (x[i] - a_vals[i])
                else:
                    h = e[i]
                x[i] = max(0.0, min(1.0, x[i] - gamma_k * h))

            idx += 1

        # Track running gap at end of each round (after accumulation)
        if k >= s_round and feas_count > 0:
            x_bar_k = [x_sum[i] / feas_count for i in range(m)]
            gap_k = sum(0.5 * alpha[i] * (x_bar_k[i] - a_vals[i]) ** 2
                        for i in range(m)) - F_star
            running_gaps.append(((k + 1) * T, gap_k))

    # Final average
    if feas_count > 0:
        x_bar = [x_sum[i] / feas_count for i in range(m)]
        final_gap = sum(0.5 * alpha[i] * (x_bar[i] - a_vals[i]) ** 2
                        for i in range(m)) - F_star
        final_G_bar = sum(e[i] * x_bar[i] - b[i] for i in range(m))
    else:
        final_gap = float('nan')
        final_G_bar = float('nan')

    return {
        'log_G': log_G,
        'log_eta': log_eta,
        'running_gaps': running_gaps,
        'final_gap': final_gap,
        'final_G_bar': final_G_bar,
        'feas_count': feas_count,
    }


# ---------------------------------------------------------------------------
# Plot functions
# ---------------------------------------------------------------------------
def plot_feasibility(log_G, log_eta, N, regime_key, regime_label, output_dir,
                     zoom=False):
    """Plot G(x^{k,t}) and eta_k vs iteration."""
    fig, ax = plt.subplots(figsize=(8, 4.5))

    if zoom:
        end = min(20000, N)
        ns = np.arange(end)
        G_plot = log_G[:end]
        eta_plot = log_eta[:end]
        suffix = '_zoom'
        title_extra = ' (zoomed)'
    else:
        if N > 50000:
            step = max(1, N // 50000)
            ns = np.arange(0, N, step)
            G_plot = log_G[::step]
            eta_plot = log_eta[::step]
        else:
            ns = np.arange(N)
            G_plot = log_G
            eta_plot = log_eta
        suffix = ''
        title_extra = ''

    ax.plot(ns, G_plot, 'k-', linewidth=0.5, alpha=0.8, label='$G(x^{k,t})$')
    ax.plot(ns, eta_plot, 'r--', linewidth=1.2, label=r'$\eta_k$')
    ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)

    ax.legend(fontsize=9, loc='upper right')
    ax.set_xlabel('Iteration', fontsize=12)
    ax.set_ylabel(r'$G(x^{k,t})$', fontsize=12)
    ax.set_title(f'Feasibility{title_extra} — {regime_label}', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=10)

    plt.tight_layout()
    fname = f'reg{regime_key}_feasibility_N300K{suffix}'
    for ext in ['pdf', 'png']:
        fig.savefig(os.path.join(output_dir, f'{fname}.{ext}'),
                    dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved {fname}.pdf/.png')


def plot_convergence_vs_iter(running_gaps, regime_key, regime_label, output_dir,
                             use_log=True):
    """Plot F(x_bar) - F* vs iteration (running average gap over time)."""
    if not running_gaps:
        print(f'  WARNING: No running gaps for regime {regime_key}')
        return

    iters = np.array([rg[0] for rg in running_gaps])
    gaps = np.array([rg[1] for rg in running_gaps])

    fig, ax = plt.subplots(figsize=(8, 4.5))

    if use_log:
        # Log-log for positive gaps (Regime A)
        pos = gaps > 0
        if np.any(pos):
            ax.loglog(iters[pos], gaps[pos], 'b-', linewidth=1.0, alpha=0.8)
        ax.set_ylabel(r'$F(\bar{x}_\eta) - F^\star$', fontsize=12)
        ax.set_title(f'Convergence — {regime_label}', fontsize=12)
    else:
        # Linear scale for negative gaps (Regime B)
        ax.plot(iters, gaps, 'b-', linewidth=1.0, alpha=0.8)
        ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
        ax.set_ylabel(r'$F(\bar{x}_\eta) - F^\star$', fontsize=12)
        ax.set_title(f'Convergence — {regime_label}', fontsize=12)

    ax.set_xlabel('Iteration', fontsize=12)
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=10)

    plt.tight_layout()
    fname = f'reg{regime_key}_convergence_vs_iter_N300K'
    for ext in ['pdf', 'png']:
        fig.savefig(os.path.join(output_dir, f'{fname}.{ext}'),
                    dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved {fname}.pdf/.png')


def plot_rate(N_vals, final_gaps, regime_key, regime_label, output_dir,
              use_abs=False):
    """Plot final F gap vs N on log-log with -1/2 reference line."""
    Ns = np.array(N_vals)
    gaps = np.array(final_gaps)

    if use_abs:
        plot_vals = np.abs(gaps)
        ylabel = r'$|F(\bar{x}_\eta) - F^\star|$'
    else:
        plot_vals = gaps
        ylabel = r'$F(\bar{x}_\eta) - F^\star$'

    # Filter positive values for log-log
    pos = plot_vals > 0
    if not np.any(pos):
        print(f'  WARNING: No positive values for rate plot regime {regime_key}')
        return

    fig, ax = plt.subplots(figsize=(7, 5))

    ax.loglog(Ns[pos], plot_vals[pos], 'bo-', linewidth=1.5, markersize=7,
              label=regime_label)

    # Fitted -1/2 reference line
    c_fit = np.median([v * math.sqrt(n) for n, v in zip(Ns[pos], plot_vals[pos])])
    n_ref = np.logspace(np.log10(Ns.min() * 0.5), np.log10(Ns.max() * 2), 200)
    ax.loglog(n_ref, c_fit / np.sqrt(n_ref), 'k--', linewidth=1.0, alpha=0.5,
              label=r'$\mathcal{O}(1/\sqrt{N})$')

    ax.set_xlabel(r'$N$ (total inner updates)', fontsize=12)
    ax.set_ylabel(ylabel, fontsize=12)
    ax.set_title(f'Rate — {regime_label}', fontsize=12)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, which='both')
    ax.tick_params(labelsize=10)

    plt.tight_layout()
    fname = f'reg{regime_key}_rate'
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

    constants = compute_constants(AGENTS)
    factor = 2.0 * constants['L_G'] * constants['M'] / constants['mu']

    print('=' * 60)
    print('Analysis Plots: Regime A vs Regime B')
    print('=' * 60)
    print(f'  x* = {constants["x_star"]}')
    print(f'  F* = {constants["F_star"]:.6f}')
    print(f'  factor = {factor:.4f}')
    print()

    for reg_key, reg in REGIMES.items():
        c1, c2, x0_val = reg['c1'], reg['c2'], reg['x0']
        threshold_0 = c2 - factor * T * c1
        G_x0 = sum(ag['e'] * x0_val - ag['b'] for ag in AGENTS)
        print(f'--- Regime {reg_key}: c1={c1}, c2={c2}, x0={x0_val}, '
              f'G(x0)={G_x0:.4f}, threshold_0={threshold_0:.4f} ---')

        # Plot 1: Feasibility (N=300K)
        K_feas = math.ceil(N_FEAS / T)
        N_actual = K_feas * T
        print(f'  Running N={N_actual:,} for feasibility + convergence-vs-iter...')
        t0 = time.time()
        res_feas = run_algorithm(AGENTS, K_feas, T, constants, c1, c2, x0_val)
        print(f'  Elapsed: {time.time() - t0:.1f}s')
        print(f'  F(x_bar)-F* = {res_feas["final_gap"]:.6f}, '
              f'G(x_bar) = {res_feas["final_G_bar"]:.6f}, '
              f'feas_count = {res_feas["feas_count"]:,}')

        plot_feasibility(res_feas['log_G'], res_feas['log_eta'], N_actual,
                         reg_key, reg['label'], output_dir, zoom=False)
        plot_feasibility(res_feas['log_G'], res_feas['log_eta'], N_actual,
                         reg_key, reg['label'], output_dir, zoom=True)

        # Plot 2: Convergence vs iteration
        use_log = (reg_key == 'A')  # log-log for positive gap, linear for negative
        plot_convergence_vs_iter(res_feas['running_gaps'], reg_key, reg['label'],
                                output_dir, use_log=use_log)

        # Plot 3: Rate (multiple N values)
        print(f'  Running rate sweep: N = {N_VALUES_RATE}')
        final_gaps = []
        final_Ns = []
        for N_target in N_VALUES_RATE:
            K_r = math.ceil(N_target / T)
            N_r = K_r * T
            if N_r == N_actual:
                # Reuse the feasibility run
                final_gaps.append(res_feas['final_gap'])
                final_Ns.append(N_r)
                continue
            res_r = run_algorithm(AGENTS, K_r, T, constants, c1, c2, x0_val)
            final_gaps.append(res_r['final_gap'])
            final_Ns.append(N_r)
            print(f'    N={N_r:,}: F gap = {res_r["final_gap"]:.6f}')

        use_abs = (reg_key == 'B')  # |F gap| for negative gaps
        plot_rate(final_Ns, final_gaps, reg_key, reg['label'], output_dir,
                  use_abs=use_abs)
        print()

    print(f'All plots saved to: {output_dir}')


if __name__ == '__main__':
    main()
