#!/usr/bin/env python3
"""Standalone numerical simulation of Fed-DCSA (no ROS/Gazebo required).

Runs the algorithm instantly and produces CSV output for plotting.
Use this for fast parameter tuning before committing to a Gazebo run.

Usage:
    python3 run_numerical.py
    python3 run_numerical.py --K 100 --T 10
    python3 run_numerical.py --e_rates 10,12,8,15 --q_weights 1,1,1,1
"""

import argparse
import os
import sys

# Allow running from the scripts/ directory or the package root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fed_dcsa.algorithm import FedDCSA, StationState
from fed_dcsa.csv_logger import CSVLogger


def parse_float_list(s):
    return [float(x) for x in s.split(',')]


def main():
    parser = argparse.ArgumentParser(description='Fed-DCSA numerical simulation')
    parser.add_argument('--K', type=int, default=20, help='Outer rounds')
    parser.add_argument('--T', type=int, default=18, help='Inner steps per round')
    parser.add_argument('--tau', type=float, default=0.7, help='Rounding threshold')
    parser.add_argument('--delta', type=float, default=0.1, help='Probability parameter')
    parser.add_argument('--E_max', type=float, default=170.0, help='Full battery (Wh)')
    parser.add_argument('--E_thr', type=float, default=50.0, help='Safety threshold (Wh)')
    parser.add_argument('--e_rates', type=parse_float_list, default=[10.0, 12.0, 8.0, 15.0],
                        help='Per-station energy costs (comma-separated)')
    parser.add_argument('--q_weights', type=parse_float_list, default=[1.0, 1.0, 1.0, 1.0],
                        help='Per-station surveillance weights (comma-separated)')
    parser.add_argument('--log_dir', type=str,
                        default=os.path.join(os.path.dirname(__file__), '..', '..', '..', 'fed_dcsa_logs'),
                        help='Output directory for CSV logs')
    parser.add_argument('--experiment_name', type=str, default='numerical',
                        help='Experiment name prefix for CSV files')
    args = parser.parse_args()

    n_stations = len(args.e_rates)
    assert len(args.q_weights) == n_stations, \
        f'e_rates ({n_stations}) and q_weights ({len(args.q_weights)}) must have same length'

    # Station drone pairing (matches crazyflies_sim.yaml)
    drone_pairs = [
        ('cf1', 'cf5'), ('cf2', 'cf6'), ('cf3', 'cf7'), ('cf4', 'cf8'),
    ]

    # Initialize station states
    stations = []
    for i in range(n_stations):
        stations.append(StationState(
            station_id=i,
            x=1.0,
            battery=args.E_max,
            e_rate=args.e_rates[i],
            q_weight=args.q_weights[i],
            active_drone=drone_pairs[i][0],
            standby_drone=drone_pairs[i][1],
        ))

    # Initialize algorithm
    algo = FedDCSA(
        K=args.K, T=args.T, tau=args.tau, delta=args.delta,
        E_max=args.E_max, E_thr=args.E_thr,
        e_rates=args.e_rates, q_weights=args.q_weights,
    )

    # Initialize logger
    log_dir = os.path.abspath(args.log_dir)
    logger = CSVLogger(log_dir, args.experiment_name, n_stations)

    # Print algorithm constants
    print(f'Fed-DCSA Numerical Simulation')
    print(f'  K={args.K}, T={args.T}, N={args.K * args.T}')
    print(f'  gamma={algo.gamma:.6f}')
    print(f'  eta={algo.eta:.4f}')
    print(f'  r_drift={algo.r_drift:.4f}')
    print(f'  M_bar={algo.M_bar:.4f}, L_G={algo.L_G:.4f}')
    print(f'  e_rates={args.e_rates}')
    print(f'  q_weights={args.q_weights}')
    print(f'  E_max={args.E_max}, E_thr={args.E_thr}')
    print(f'  tau={args.tau}, delta={args.delta}')
    print()

    # Run simulation
    total_swaps = 0
    feasible_rounds = 0

    for k in range(args.K):
        # Record pre-swap battery for logging
        batteries_before = [s.battery for s in stations]

        result = algo.run_round(k, stations)

        # Log swap events
        for sid in result.swaps:
            # The swap already happened in process_round_end, so active/standby
            # are already flipped. The "new_active" is the current active_drone.
            logger.log_swap(
                k, sid,
                old_active=stations[sid].standby_drone,  # was active before swap
                new_active=stations[sid].active_drone,    # is active after swap
                battery_at_swap=batteries_before[sid],
            )

        logger.log_round(result, stations)

        total_swaps += len(result.swaps)
        if result.b_k == 1:
            feasible_rounds += 1

        # Print round summary
        gate_str = '\033[92mFEASIBLE\033[0m' if result.b_k == 1 else '\033[91mINFEASIBLE\033[0m'
        print(f'\u2550\u2550\u2550 Round {k}/{args.K} \u2550\u2550\u2550')
        print(f'  Gate: {gate_str} (b_k={result.b_k}) | '
              f'G\u0303={result.G_tilde:.4f}, \u03b7={result.eta_k:.4f}, '
              f'r_drift={result.r_drift:.4f}')
        for i, s in enumerate(stations):
            action_str = 'ACTIVE' if result.actions[i] == 1 else '\033[93mSWAP!\033[0m'
            swap_note = ''
            if i in result.swaps:
                swap_note = f' \u2192 {s.active_drone} takes over'
            print(f'  Station {i} ({s.active_drone}): '
                  f'x={result.x_before[i]:.3f}\u2192{result.x_after[i]:.3f} '
                  f'\u2192 {action_str} | '
                  f'Battery: {s.battery:.1f}/{args.E_max:.0f} Wh{swap_note}')
        print()

    logger.close()

    # Summary
    x_bar = algo.get_weighted_output()
    print(f'\u2550\u2550\u2550 Experiment Complete \u2550\u2550\u2550')
    print(f'  Feasible rounds: {feasible_rounds}/{args.K}')
    print(f'  Total swaps: {total_swaps}')
    print(f'  Weighted output x\u0304: {[f"{x:.4f}" for x in x_bar]}')
    print(f'  CSV files written to: {log_dir}/')
    print(f'    - {logger.rounds_path}')
    print(f'    - {logger.batteries_path}')
    print(f'    - {logger.swaps_path}')


if __name__ == '__main__':
    main()
