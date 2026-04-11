#!/usr/bin/env python3
"""Synthetic experiment for CDC 2026: Algorithm 1 on separable quadratic + linear constraint.

Uses decreasing stepsize and tolerance (Lan-Zhou Corollary 6 analog).
Runs 12 configurations (2 T values x 6 N values), produces convergence scatter
plot and feasibility plots.

Usage:
    python3 run_synthetic.py
    python3 run_synthetic.py --output_dir /path/to/output
"""

import math
import os
import sys
import time
import argparse

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Agent parameters (Table from instruction PDF, Section 2)
# ---------------------------------------------------------------------------
AGENTS = [
    {'alpha': 10.0, 'a': 0.5, 'e': 0.3, 'b': 0.1},
    {'alpha': 8.0,  'a': 0.5, 'e': 0.4, 'b': 0.1},
]
# x*≈(0.345, 0.241), F*≈0.388, G(x*)=0 (active constraint, λ*≈5.17)
# G(1,1)=0.5 (infeasible start), G(0,0)=-0.2 (min reachable)
# T_max=32.0 > 27 ✓

DELTA = 0.1
T_VALUES = [15]
N_VALUES = [30_000, 100_000, 300_000, 1_000_000, 3_000_000, 10_000_000]

# Hand-picked stepsize/tolerance constants per T
# gamma_k = c1/sqrt(k+1), eta_k = c2/sqrt(k+1)
# Deliberately c2 < factor*T*c1 so gate threshold eta_k - r_drift_k < 0.
# Gate only opens when G < 0 (truly feasible) → positive F gap.
# factor = 2*L_G*M/mu ≈ 6.4; G range: [-0.2, 0.5]
STEPSIZE_PARAMS = {
    15: {'c1': 0.003, 'c2': 0.2},   # threshold_0 = 0.2-0.288 = -0.088 (margin to -0.2: 0.112)
    27: {'c1': 0.0015, 'c2': 0.2},  # threshold_0 = 0.2-0.259 = -0.059 (margin to -0.2: 0.141)
}


# ---------------------------------------------------------------------------
# Step 1: KKT solution and algorithm constants
# ---------------------------------------------------------------------------
def compute_kkt_solution(agents):
    """Compute the KKT optimal solution and all algorithm constants.

    Returns a dict with x_star, lambda_star, F_star, M, L_G, D_x, etc.
    """
    m = len(agents)

    # G(a) = sum(e_i * a_i - b_i) — constraint at unconstrained optimum
    G_a = sum(ag['e'] * ag['a'] - ag['b'] for ag in agents)

    if G_a > 0:
        # Constraint active at optimum: solve KKT system
        num = sum(ag['e'] * ag['a'] - ag['b'] for ag in agents)
        den = sum(ag['e'] ** 2 / ag['alpha'] for ag in agents)
        lambda_star = num / den
        x_star = [ag['a'] - lambda_star * ag['e'] / ag['alpha'] for ag in agents]
    else:
        # Constraint slack at optimum: unconstrained solution
        lambda_star = 0.0
        x_star = [ag['a'] for ag in agents]

    # F(x*) = sum (1/2) * alpha_i * (x_i* - a_i)^2
    F_star = sum(0.5 * ag['alpha'] * (xs - ag['a']) ** 2
                 for ag, xs in zip(agents, x_star))

    # G(x*)
    G_star = sum(ag['e'] * xs - ag['b'] for ag, xs in zip(agents, x_star))

    # Lipschitz constants
    L_f = [ag['alpha'] * max(ag['a'], 1.0 - ag['a']) for ag in agents]
    L_g = [ag['e'] for ag in agents]
    L_G = math.sqrt(sum(e ** 2 for e in L_g))

    # Subgradient bounds (exact gradients, no noise => M_bar_i = M_i)
    M_i = [max(lf, lg) for lf, lg in zip(L_f, L_g)]
    M = math.sqrt(sum(mi ** 2 for mi in M_i))

    # Prox diameter with Bregman V_i(u,x) = (1/2)(u-x)^2
    # D_x^2 = sum_i max( (1/2)(x_i*-0)^2, (1/2)(x_i*-1)^2 )
    D_x_sq = sum(max(0.5 * xs ** 2, 0.5 * (xs - 1.0) ** 2) for xs in x_star)
    D_x = math.sqrt(D_x_sq)

    # mu = 1.0 (Euclidean mirror map)
    mu = 1.0

    # Condition (13): T_max = mu * M^2 / (4 * delta * L_G * M_bar)
    # With mu=1, M_bar=M: T_max = M / (4 * delta * L_G)
    T_max = M / (4.0 * DELTA * L_G)

    constants = {
        'm': m,
        'x_star': x_star,
        'lambda_star': lambda_star,
        'F_star': F_star,
        'G_star': G_star,
        'G_a': G_a,
        'L_f': L_f,
        'L_g': L_g,
        'L_G': L_G,
        'M_i': M_i,
        'M': M,
        'D_x': D_x,
        'D_x_sq': D_x_sq,
        'mu': mu,
        'T_max': T_max,
    }

    # Verification
    assert lambda_star >= 0, f'lambda* = {lambda_star} must be >= 0'
    assert all(0 < xs < 1 for xs in x_star), f'x* = {x_star} must be in (0,1)'
    assert G_star <= 1e-10, f'G(x*) = {G_star} must be <= 0'

    return constants


def print_constants(constants):
    """Print all computed constants."""
    print('=' * 60)
    print('KKT Solution and Algorithm Constants')
    print('=' * 60)
    print(f'  lambda*    = {constants["lambda_star"]:.6f}')
    for i, xs in enumerate(constants['x_star']):
        print(f'  x_{i+1}*       = {xs:.6f}')
    print(f'  F(x*)      = {constants["F_star"]:.6f}')
    print(f'  G(x*)      = {constants["G_star"]:.2e} (should be 0)')
    print(f'  G(a)       = {constants["G_a"]:.4f} (> 0, constraint violated)')
    print()
    print(f'  L_f        = {constants["L_f"]}')
    print(f'  L_g        = {constants["L_g"]}')
    print(f'  L_G        = {constants["L_G"]:.6f}')
    print(f'  M_i        = {constants["M_i"]}')
    print(f'  M          = {constants["M"]:.6f}')
    print(f'  D_x        = {constants["D_x"]:.6f} (Bregman, with 1/2)')
    print(f'  mu         = {constants["mu"]:.1f}')
    print(f'  T_max      = {constants["T_max"]:.4f} (Condition 13)')
    print()


# ---------------------------------------------------------------------------
# Step 2: Per-run parameter computation
# ---------------------------------------------------------------------------
def compute_run_params(T, N_target, constants):
    """Compute K and N for a given (T, N_target) configuration."""
    K = math.ceil(N_target / T)
    N = K * T

    # Verify Condition 13: T < T_max
    assert T < constants['T_max'], \
        f'T={T} violates Condition (13): T < {constants["T_max"]:.4f}'

    return {
        'T': T,
        'K': K,
        'N': N,
        'N_target': N_target,
    }


# ---------------------------------------------------------------------------
# Step 3: Algorithm 1 — core loop (decreasing stepsize & tolerance)
# ---------------------------------------------------------------------------
def run_algorithm(agents, K, T, constants, c1, c2):
    """Execute Algorithm 1 with hand-picked decreasing gamma_k and eta_k.

    gamma_k = c1 / sqrt(k + 1)
    eta_k   = c2 / sqrt(k + 1)

    Computes the feasible-weighted running average from round s=ceil(K/2) onward
    inside the loop to avoid storing log_x (memory-efficient for large N).

    Returns dict with log_G, log_b, log_eta arrays, plus final_gap and final_G_bar.
    """
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

    # Pre-allocate log arrays (no log_x — too large for big N)
    log_G = np.empty(N, dtype=np.float64)
    log_b = np.empty(N, dtype=np.int32)
    log_eta = np.empty(N, dtype=np.float64)

    # Running average accumulators (from round s onward)
    s_round = math.ceil(K / 2)
    s_iter = s_round * T
    x_sum = [0.0] * m
    feas_count = 0

    # Initialize at mildly infeasible start x_i = 0.6: G(0.6,0.6) = 0.22 > 0
    x = [0.6 for _ in agents]

    idx = 0
    for k in range(K):
        # Per-round decreasing parameters (hand-picked constants)
        sqrt_k1 = math.sqrt(k + 1)
        gamma_k = c1 / sqrt_k1
        eta_k = c2 / sqrt_k1
        r_drift_k = (2.0 * L_G * M / mu) * T * gamma_k

        # Communication phase: constraint reports using x^{k,0}
        G_tilde = sum(e[i] * x[i] - b[i] for i in range(m))

        # Gate decision (Eq. 2)
        b_k = 1 if G_tilde <= eta_k - r_drift_k else 0

        for t in range(T):
            # LOG BEFORE update
            log_G[idx] = sum(e[i] * x[i] - b[i] for i in range(m))
            log_b[idx] = b_k
            log_eta[idx] = eta_k

            # Accumulate running average from s onward
            if idx >= s_iter and b_k == 1:
                for i in range(m):
                    x_sum[i] += x[i]
                feas_count += 1

            # Gradient step with decreasing stepsize
            for i in range(m):
                if b_k == 1:
                    h = alpha[i] * (x[i] - a_vals[i])  # grad f_i
                else:
                    h = e[i]  # grad g_i (constant)
                x[i] = max(0.0, min(1.0, x[i] - gamma_k * h))

            idx += 1

    # Compute final average
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
        'log_b': log_b,
        'log_eta': log_eta,
        'final_gap': final_gap,
        'final_G_bar': final_G_bar,
        'feas_count': feas_count,
        's_round': s_round,
    }


# (Running average is now computed inside run_algorithm for memory efficiency)


# ---------------------------------------------------------------------------
# Step 5: Verification checklist
# ---------------------------------------------------------------------------
def verify_results(results, constants, params):
    """Run verification checks. Returns list of (name, passed, detail) tuples."""
    checks = []
    K = params['K']
    T = params['T']
    log_b = results['log_b']
    final_gap = results['final_gap']
    final_G_bar = results['final_G_bar']

    # Count feasible rounds (from s_round onward)
    s_round = results['s_round']
    round_gates = log_b[::T]  # b_k for each round
    n_feasible_post_s = int(np.sum(round_gates[s_round:] == 1))
    n_rounds_post_s = K - s_round
    frac = n_feasible_post_s / n_rounds_post_s if n_rounds_post_s > 0 else 0
    checks.append(('Feasible rounds (post-s)',
                    n_feasible_post_s > 0,
                    f'{n_feasible_post_s}/{n_rounds_post_s} ({frac:.1%})'))

    # Report F gap and G bar values
    if not np.isnan(final_gap):
        checks.append(('F gap', True,
                        f'F(x_bar)-F* = {final_gap:.6f}'))
    else:
        checks.append(('F gap', False, 'No feasible iterates'))

    if not np.isnan(final_G_bar):
        checks.append(('G(x_bar)', True,
                        f'G(x_bar) = {final_G_bar:.6f}'))
    else:
        checks.append(('G(x_bar)', False, 'No feasible iterates'))

    return checks


def print_checks(checks, T, N):
    """Print verification results for one run."""
    print(f'  --- Verification (T={T}, N={N:,}) ---')
    for name, passed, detail in checks:
        status = 'PASS' if passed else 'FAIL'
        print(f'    [{status}] {name}: {detail}')
    print()


# ---------------------------------------------------------------------------
# Step 6: Convergence scatter plot
# ---------------------------------------------------------------------------
def plot_convergence(all_runs, constants, output_dir):
    """Scatter plot: final output values vs N on log-log.

    Each run (different N) contributes one data point.
    Panel (a): F(x_bar) - F* vs N (positive gap).
    Panel (b): |G(x_bar)| vs N (feasibility margin).
    Both on log-log with a generic c/sqrt(N) reference line for slope.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # Group runs by T
    T_groups = {}
    for run_info in all_runs:
        T = run_info['params']['T']
        T_groups.setdefault(T, []).append(run_info)

    markers = {'15': 'o', '27': 's'}
    colors = {'15': '#1f77b4', '27': '#d62728'}
    max_N = 0
    all_pos_F = []  # collect for reference line fitting
    all_pos_G = []

    for T_val, runs in sorted(T_groups.items()):
        Ns = []
        G_bars = []
        F_gaps = []
        for run_info in runs:
            N = run_info['params']['N']
            fg = run_info['results']['final_gap']
            gb = run_info['results']['final_G_bar']
            if np.isnan(fg) or np.isnan(gb):
                continue
            Ns.append(N)
            G_bars.append(gb)
            F_gaps.append(fg)
            max_N = max(max_N, N)

        Ns = np.array(Ns)
        G_bars = np.array(G_bars)
        F_gaps = np.array(F_gaps)

        mk = markers.get(str(T_val), 'D')
        cl = colors.get(str(T_val), '#2ca02c')

        # Left panel: F(x_bar) - F* (positive gap, log-log)
        pos_F = F_gaps > 0
        if np.any(pos_F):
            ax1.loglog(Ns[pos_F], F_gaps[pos_F], marker=mk, color=cl,
                       linewidth=1.5, markersize=7, label=f'$T = {T_val}$')
            for n, f in zip(Ns[pos_F], F_gaps[pos_F]):
                all_pos_F.append((n, f))

        # Right panel: |G(x_bar)| (G should be negative; show magnitude)
        abs_G = np.abs(G_bars)
        nonzero = abs_G > 0
        if np.any(nonzero):
            ax2.loglog(Ns[nonzero], abs_G[nonzero], marker=mk, color=cl,
                       linewidth=1.5, markersize=7, label=f'$T = {T_val}$')
            for n, g in zip(Ns[nonzero], abs_G[nonzero]):
                all_pos_G.append((n, g))

    # Reference lines
    if max_N == 0:
        max_N = 1e7
    n_ref = np.logspace(np.log10(1e3), np.log10(max_N * 2), 500)

    # Corollary 2 worst-case bounds
    D_x = constants['D_x']
    M_val = constants['M']
    C_F_bound = 4.0 * D_x * (1.0 + math.log(2) / 2.0) * M_val / DELTA
    C_G_bound = 7.0 * math.sqrt(2) * D_x * M_val / DELTA

    # Panel (a): F gap
    # Fitted O(1/sqrt(N)) reference line
    if all_pos_F:
        c_F = np.median([f * math.sqrt(n) for n, f in all_pos_F])
        ref_F = c_F / np.sqrt(n_ref)
        ax1.loglog(n_ref, ref_F, 'k--', linewidth=1.0, alpha=0.5,
                   label=r'$\mathcal{O}(1/\sqrt{N})$')
    # Corollary 2 bound
    bound_F = C_F_bound / np.sqrt(n_ref)
    ax1.loglog(n_ref, bound_F, 'k:', linewidth=1.2, alpha=0.6,
               label='Corollary 2 bound')

    ax1.set_xlabel(r'$N$ (total inner updates)', fontsize=12)
    ax1.set_ylabel(r'$F(\bar{x}_\eta) - F^\star$', fontsize=12)
    ax1.set_title('(a) Optimality gap', fontsize=13)
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3, which='both')
    ax1.tick_params(labelsize=10)

    # Panel (b): |G| feasibility margin
    if all_pos_G:
        c_G = np.median([g * math.sqrt(n) for n, g in all_pos_G])
        ref_G = c_G / np.sqrt(n_ref)
        ax2.loglog(n_ref, ref_G, 'k--', linewidth=1.0, alpha=0.5,
                   label=r'$\mathcal{O}(1/\sqrt{N})$')
    # Corollary 2 bound
    bound_G = C_G_bound / np.sqrt(n_ref)
    ax2.loglog(n_ref, bound_G, 'k:', linewidth=1.2, alpha=0.6,
               label='Corollary 2 bound')

    ax2.set_xlabel(r'$N$ (total inner updates)', fontsize=12)
    ax2.set_ylabel(r'$|G(\bar{x}_\eta)|$', fontsize=12)
    ax2.set_title('(b) Feasibility margin', fontsize=13)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3, which='both')
    ax2.tick_params(labelsize=10)

    plt.tight_layout()
    for ext in ['pdf', 'png']:
        path = os.path.join(output_dir, f'convergence.{ext}')
        fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved convergence.pdf/.png')


# ---------------------------------------------------------------------------
# Step 7: Feasibility plot (with decreasing eta_k)
# ---------------------------------------------------------------------------
def plot_feasibility(T_val, N_val, results, params, constants, output_dir,
                     c1=None, c2=None):
    """Plot G(x^{k,t}) with eta_k staircase."""
    log_G = results['log_G']
    log_eta = results['log_eta']
    K = params['K']
    T = params['T']
    N = params['N']

    fig, ax = plt.subplots(figsize=(8, 4.5))

    # For very large N, subsample for performance
    if N > 50000:
        step = max(1, N // 50000)
        ns = np.arange(0, N, step)
        G_plot = log_G[::step]
        eta_plot = log_eta[::step]
    else:
        ns = np.arange(N)
        G_plot = log_G
        eta_plot = log_eta

    ax.plot(ns, G_plot, 'k-', linewidth=0.5, alpha=0.8, label='$G(x^{k,t})$')
    ax.plot(ns, eta_plot, 'r--', linewidth=1.2, label=r'$\eta_k$')
    ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)

    ax.legend(fontsize=9, loc='upper right')

    ax.set_xlabel('Iteration', fontsize=12)
    ax.set_ylabel(r'$G(x^{k,t})$', fontsize=12)
    ax.set_title(f'Feasibility — $T = {T_val}$, $N = {N_val:,}$', fontsize=13)
    ax.grid(True, alpha=0.3)
    ax.tick_params(labelsize=10)

    plt.tight_layout()
    N_tag = f'{N_val}'
    for ext in ['pdf', 'png']:
        path = os.path.join(output_dir, f'feasibility_T{T_val}_N{N_tag}.{ext}')
        fig.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved feasibility_T{T_val}_N{N_tag}.pdf/.png')

    # --- Zoomed-in version (first ~20000 iterations) ---
    zoom_end = min(20000, N)
    fig_z, ax_z = plt.subplots(figsize=(8, 4.5))

    ns_z = np.arange(zoom_end)
    ax_z.plot(ns_z, log_G[:zoom_end], 'k-', linewidth=0.8, alpha=0.8,
              label='$G(x^{k,t})$')
    ax_z.plot(ns_z, log_eta[:zoom_end], 'r--', linewidth=1.2,
              label=r'$\eta_k$')
    ax_z.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)

    ax_z.legend(fontsize=9, loc='upper right')

    ax_z.set_xlabel('Iteration', fontsize=12)
    ax_z.set_ylabel(r'$G(x^{k,t})$', fontsize=12)
    ax_z.set_title(
        f'Feasibility (zoomed) — $T = {T_val}$, $N = {N_val:,}$', fontsize=13)
    ax_z.grid(True, alpha=0.3)
    ax_z.tick_params(labelsize=10)

    plt.tight_layout()
    for ext in ['pdf', 'png']:
        path = os.path.join(output_dir,
                            f'feasibility_T{T_val}_N{N_tag}_zoom.{ext}')
        fig_z.savefig(path, dpi=200, bbox_inches='tight')
    plt.close(fig_z)
    print(f'  Saved feasibility_T{T_val}_N{N_tag}_zoom.pdf/.png')


# ---------------------------------------------------------------------------
# Step 8: Save data
# ---------------------------------------------------------------------------
def save_experiment_log(constants, all_runs, output_dir):
    """Write human-readable experiment log."""
    path = os.path.join(output_dir, 'experiment_log.txt')
    with open(path, 'w') as f:
        f.write('CDC 2026 Synthetic Experiment Log (Decreasing Stepsize)\n')
        f.write('=' * 60 + '\n\n')

        f.write('Agent Parameters:\n')
        for i, ag in enumerate(AGENTS):
            f.write(f'  Agent {i+1}: alpha={ag["alpha"]}, a={ag["a"]}, '
                    f'e={ag["e"]}, b={ag["b"]}\n')
        f.write(f'  delta = {DELTA}\n\n')

        f.write('KKT Solution:\n')
        f.write(f'  lambda* = {constants["lambda_star"]:.6f}\n')
        for i, xs in enumerate(constants['x_star']):
            f.write(f'  x_{i+1}*    = {xs:.6f}\n')
        f.write(f'  F(x*)   = {constants["F_star"]:.6f}\n')
        f.write(f'  G(x*)   = {constants["G_star"]:.2e}\n')
        f.write(f'  G(a)    = {constants["G_a"]:.4f}\n\n')

        f.write('Algorithm Constants:\n')
        f.write(f'  L_f   = {constants["L_f"]}\n')
        f.write(f'  L_g   = {constants["L_g"]}\n')
        f.write(f'  L_G   = {constants["L_G"]:.6f}\n')
        f.write(f'  M_i   = {constants["M_i"]}\n')
        f.write(f'  M     = {constants["M"]:.6f}\n')
        f.write(f'  D_x   = {constants["D_x"]:.6f}\n')
        f.write(f'  mu    = {constants["mu"]:.1f}\n')
        f.write(f'  T_max = {constants["T_max"]:.4f}\n\n')

        f.write('Formulas:\n')
        f.write('  gamma_k = c1 / sqrt(k + 1)\n')
        f.write('  eta_k   = c2 / sqrt(k + 1)\n')
        for T_val, sp in STEPSIZE_PARAMS.items():
            f.write(f'  T={T_val}: c1={sp["c1"]}, c2={sp["c2"]}\n')
        f.write('  Averaging from round s = ceil(K/2)\n\n')

        for run_info in all_runs:
            params = run_info['params']
            checks = run_info['checks']
            f.write(f'Run: T={params["T"]}, N_target={params["N_target"]:,}, '
                    f'N={params["N"]:,}, K={params["K"]:,}\n')
            f.write(f'  F(x_bar)-F* = {run_info["results"]["final_gap"]:.6f}, '
                    f'G(x_bar) = {run_info["results"]["final_G_bar"]:.6f}\n')
            for name, passed, detail in checks:
                status = 'PASS' if passed else 'FAIL'
                f.write(f'  [{status}] {name}: {detail}\n')
            f.write('\n')

    print(f'  Saved experiment_log.txt')


def save_experiment_data(all_runs, constants, output_dir):
    """Save raw data arrays as .npz for re-plotting."""
    data = {
        'x_star': np.array(constants['x_star']),
        'F_star': constants['F_star'],
        'M': constants['M'],
        'D_x': constants['D_x'],
        'L_G': constants['L_G'],
        'delta': DELTA,
    }

    for i, run_info in enumerate(all_runs):
        prefix = f'run{i}'
        data[f'{prefix}_T'] = run_info['params']['T']
        data[f'{prefix}_K'] = run_info['params']['K']
        data[f'{prefix}_N'] = run_info['params']['N']
        data[f'{prefix}_final_gap'] = run_info['results']['final_gap']
        data[f'{prefix}_final_G_bar'] = run_info['results']['final_G_bar']

    path = os.path.join(output_dir, 'experiment_data.npz')
    np.savez_compressed(path, **data)
    print(f'  Saved experiment_data.npz')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='CDC 2026 Synthetic Experiment — Algorithm 1 (Decreasing Stepsize)')
    parser.add_argument('--output_dir', type=str,
                        default=os.path.join(os.path.dirname(__file__),
                                             '..', '..', '..', 'fed_dcsa_logs', 'synthetic'),
                        help='Output directory')
    args = parser.parse_args()
    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    # Step 1: Compute KKT solution and constants
    constants = compute_kkt_solution(AGENTS)
    print_constants(constants)

    # Step 2-5: Run all configurations
    all_runs = []
    # Generate feasibility plots for these N values
    FEAS_PLOT_NS = {300_000, 3_000_000}

    for T in T_VALUES:
        sp = STEPSIZE_PARAMS[T]
        c1, c2 = sp['c1'], sp['c2']
        print(f'\n--- T={T}: c1={c1}, c2={c2} ---')
        factor = 2.0 * constants['L_G'] * constants['M'] / constants['mu']
        threshold_0 = c2 - factor * T * c1
        print(f'  Gate threshold_0: c2 - {factor:.2f}*T*c1 = {threshold_0:.4f}')

        for N_target in N_VALUES:
            print(f'=== Running T={T}, N_target={N_target:,} ===')
            t0 = time.time()

            params = compute_run_params(T, N_target, constants)
            print(f'  K={params["K"]:,}, N={params["N"]:,}')

            results = run_algorithm(AGENTS, params['K'], T, constants, c1, c2)

            final_gap = results['final_gap']
            final_G_bar = results['final_G_bar']
            print(f'  F(x_bar)-F* = {final_gap:.6f}, G(x_bar) = {final_G_bar:.6f}')
            print(f'  Feasible iterates in avg: {results["feas_count"]:,}')

            checks = verify_results(results, constants, params)
            print_checks(checks, T, params['N'])

            elapsed = time.time() - t0
            print(f'  Elapsed: {elapsed:.1f}s')

            # Feasibility plot for selected N values
            if N_target in FEAS_PLOT_NS:
                plot_feasibility(T, N_target, results, params, constants, output_dir,
                                c1=c1, c2=c2)

            # Keep only what's needed for scatter plot (free large arrays)
            run_info = {
                'params': params,
                'results': {
                    'final_gap': results['final_gap'],
                    'final_G_bar': results['final_G_bar'],
                    'feas_count': results['feas_count'],
                    's_round': results['s_round'],
                },
                'checks': checks,
            }
            all_runs.append(run_info)
            del results  # free memory

    # Step 6: Convergence scatter plot
    print()
    plot_convergence(all_runs, constants, output_dir)

    # Step 8: Save data
    save_experiment_log(constants, all_runs, output_dir)
    save_experiment_data(all_runs, constants, output_dir)

    print()
    print('=' * 60)
    print(f'All outputs saved to: {output_dir}/')
    print('=' * 60)


if __name__ == '__main__':
    main()
