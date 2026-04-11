#!/usr/bin/env python3
"""Generate paper-quality plots from Fed-DCSA experiment CSV logs.

Usage:
    python3 plot_results.py
    python3 plot_results.py --log_dir /path/to/logs --experiment_name baseline
"""

import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


def load_csv(path):
    """Load CSV file into list of dicts."""
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        return list(reader)


def plot_all(log_dir, experiment_name, output_dir=None):
    if output_dir is None:
        output_dir = log_dir

    prefix = os.path.join(log_dir, experiment_name)
    rounds_path = f'{prefix}_rounds.csv'
    batt_path = f'{prefix}_batteries.csv'
    swaps_path = f'{prefix}_swaps.csv'

    for p in [rounds_path, batt_path]:
        if not os.path.exists(p):
            print(f'ERROR: {p} not found')
            sys.exit(1)

    rounds_data = load_csv(rounds_path)
    batt_data = load_csv(batt_path)
    swaps_data = load_csv(swaps_path) if os.path.exists(swaps_path) else []

    K = len(rounds_data)
    ks = np.array([int(r['k']) for r in rounds_data])

    # Detect number of stations from columns
    n_stations = 0
    while f'x_before_{n_stations}' in rounds_data[0]:
        n_stations += 1

    station_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']

    # Per-drone style: paired colors (solid = initial active, dashed = standby)
    drone_styles = {
        'cf1': (station_colors[0], '-'),  'cf5': (station_colors[0], '--'),
        'cf2': (station_colors[1], '-'),  'cf6': (station_colors[1], '--'),
        'cf3': (station_colors[2], '-'),  'cf7': (station_colors[2], '--'),
        'cf4': (station_colors[3], '-'),  'cf8': (station_colors[3], '--'),
    }

    # Build per-drone lookup from battery data
    drone_batt = {}   # drone_name -> [(k, battery_wh)]
    drone_activity = {}  # drone_name -> [(k, x_after)]
    round_station_drone = {}  # (k, station_id) -> active_drone
    for row in batt_data:
        drone = row['active_drone']
        k_val = int(row['k'])
        sid = int(row['station_id'])
        drone_batt.setdefault(drone, []).append((k_val, float(row['battery_wh'])))
        round_station_drone[(k_val, sid)] = drone

    # Build swap lookup: (k, station_id) -> (old_active, new_active)
    swap_lookup = {}
    for sw in swaps_data:
        swap_lookup[(int(sw['k']), int(sw['station_id']))] = (sw['old_active'], sw['new_active'])

    # Build per-drone activity from rounds data + drone lookup
    # At swap rounds, x_after was computed by old_active (not new_active logged in battery CSV)
    for r in rounds_data:
        k_val = int(r['k'])
        for i in range(n_stations):
            x_val = float(r[f'x_after_{i}'])
            swap = swap_lookup.get((k_val, i))
            if swap:
                old_drone, new_drone = swap
                # x_after was computed by old_active during inner steps
                drone_activity.setdefault(old_drone, []).append((k_val, x_val))
                # new_active starts fresh at x=1.0 after swap
                drone_activity.setdefault(new_drone, []).append((k_val, 1.0))
            else:
                drone = round_station_drone.get((k_val, i))
                if drone:
                    drone_activity.setdefault(drone, []).append((k_val, x_val))

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    def insert_gap_nans(data_pairs):
        """Insert NaN between non-consecutive rounds to break the line."""
        if not data_pairs:
            return [], []
        out_k, out_v = [], []
        for idx, (k_val, v_val) in enumerate(data_pairs):
            if idx > 0 and k_val - data_pairs[idx - 1][0] > 1:
                out_k.append(data_pairs[idx - 1][0] + 0.5)
                out_v.append(float('nan'))
            out_k.append(k_val)
            out_v.append(v_val)
        return out_k, out_v

    # ── Plot 1: Activity levels per drone ──
    ax = axes[0, 0]
    for drone in sorted(drone_activity.keys()):
        d_ks, d_xs = insert_gap_nans(drone_activity[drone])
        color, ls = drone_styles.get(drone, ('gray', '-'))
        ax.plot(d_ks, d_xs, color=color, linestyle=ls, label=drone, linewidth=1.2)
    ax.set_xlabel('Round (k)')
    ax.set_ylabel('Activity level $x_i$')
    ax.set_title('Activity Levels Per Drone')
    ax.set_ylim(-0.05, 1.05)
    ax.axhline(y=0.7, color='gray', linestyle='--', linewidth=0.8, alpha=0.5,
               label='$\\tau=0.7$')
    ax.legend(fontsize=7, ncol=3)
    ax.grid(True, alpha=0.3)

    # ── Plot 2: Battery levels per drone ──
    ax = axes[0, 1]
    for drone in sorted(drone_batt.keys()):
        d_ks, d_batt = insert_gap_nans(drone_batt[drone])
        color, ls = drone_styles.get(drone, ('gray', '-'))
        ax.plot(d_ks, d_batt, color=color, linestyle=ls, label=drone, linewidth=1.2)

    # Mark swap events
    if swaps_data:
        for sw in swaps_data:
            sk = int(sw['k'])
            sid = int(sw['station_id'])
            batt = float(sw['battery_at_swap'])
            ax.scatter(sk, batt, marker='v', s=60,
                       color=station_colors[sid % len(station_colors)],
                       edgecolors='black', zorder=5)

    ax.set_xlabel('Round (k)')
    ax.set_ylabel('Battery (Wh)')
    ax.set_title('Battery Levels Per Drone — post-drain (▼ = swap)')
    ax.legend(fontsize=7, ncol=3)
    ax.grid(True, alpha=0.3)

    # ── Plot 3: Gate decisions over rounds ──
    ax = axes[1, 0]
    b_ks = np.array([int(r['b_k']) for r in rounds_data])
    bar_colors = ['#2ca02c' if b == 1 else '#d62728' for b in b_ks]
    ax.bar(ks, np.ones_like(ks), color=bar_colors, width=1.0, align='edge',
           edgecolor='none', alpha=0.7)
    ax.set_xlabel('Round (k)')
    ax.set_ylabel('Gate $b_k$')
    ax.set_title('Gate Decisions — start of round (green=feasible, red=infeasible)')
    ax.set_ylim(0, 1.3)
    ax.set_yticks([])
    green_patch = mpatches.Patch(color='#2ca02c', alpha=0.7, label='Feasible ($b_k=1$)')
    red_patch = mpatches.Patch(color='#d62728', alpha=0.7, label='Infeasible ($b_k=0$)')
    ax.legend(handles=[green_patch, red_patch], fontsize=8)
    ax.grid(True, alpha=0.3, axis='x')

    # ── Plot 4: Aggregate constraint vs tolerance ──
    ax = axes[1, 1]
    G_tildes = np.array([float(r['G_tilde']) for r in rounds_data])
    etas = np.array([float(r['eta_k']) for r in rounds_data])
    r_drifts = np.array([float(r['r_drift']) for r in rounds_data])

    ax.plot(ks, G_tildes, 'b-', label='$\\tilde{G}_k$ (aggregate constraint)', linewidth=1.5)
    ax.plot(ks, etas, 'r--', label='$\\eta_k$ (tolerance)', linewidth=1.2)
    ax.plot(ks, etas - r_drifts, 'g:', label='$\\eta_k - r^{drift}_k$ (gate threshold)', linewidth=1.2)
    ax.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
    ax.set_xlabel('Round (k)')
    ax.set_ylabel('Constraint value')
    ax.set_title('Aggregate Constraint vs Tolerance — start of round')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(output_dir, f'{experiment_name}_plots.png')
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f'Saved plots to {out_path}')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description='Plot Fed-DCSA experiment results')
    parser.add_argument('--log_dir', type=str,
                        default=os.path.join(os.path.dirname(__file__), '..', '..', '..', 'fed_dcsa_logs'),
                        help='Directory containing CSV logs')
    parser.add_argument('--experiment_name', type=str, default='numerical',
                        help='Experiment name prefix')
    args = parser.parse_args()

    log_dir = os.path.abspath(args.log_dir)
    plot_all(log_dir, args.experiment_name)


if __name__ == '__main__':
    main()
