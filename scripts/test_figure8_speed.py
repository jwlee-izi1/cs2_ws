#!/usr/bin/env python3
"""Offline test for the figure8_speed parameter.

Loads the figure-8 polynomial trajectory CSV, evaluates it at multiple speed
multipliers, and produces a 4-panel diagnostic:

1. XY paths (should all be the same shape — speed only scales time, not space)
2. Position(t) over time (faster speeds compress in time)
3. Velocity magnitude profile per speed (peak velocity scales with speed)
4. Peak velocity vs speed multiplier (annotated with max_setpoint_velocity ceiling)

The output identifies the highest figure8_speed value where peak velocity
stays under the streamer's max_setpoint_velocity, so the drone can faithfully
trace the path without rate-limiting kicking in.

Run:
    python3 scripts/test_figure8_speed.py
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

WORKSPACE = Path(__file__).resolve().parent.parent


def load_pieces(csv_path: Path):
    pieces = []
    with open(csv_path) as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            row = [c for c in row if c.strip() != '']
            if len(row) < 33:
                continue
            duration = float(row[0])
            x = [float(c) for c in row[1:9]]
            y = [float(c) for c in row[9:17]]
            pieces.append((duration, x, y))
    return pieces


def eval_poly(coeffs, t):
    out = 0.0
    p = 1.0
    for c in coeffs:
        out += c * p
        p *= t
    return out


def eval_trajectory(pieces, t):
    total = sum(p[0] for p in pieces)
    if total <= 0:
        return (0.0, 0.0)
    t = t % total
    for duration, x, y in pieces:
        if t <= duration:
            return (eval_poly(x, t), eval_poly(y, t))
        t -= duration
    duration, x, y = pieces[-1]
    return (eval_poly(x, duration), eval_poly(y, duration))


def trajectory_samples(pieces, speed: float, duration_sec: float, sample_dt: float = 0.02):
    """Sample the trajectory at speed multiplier `speed` over `duration_sec` of wall clock."""
    ts = np.arange(0.0, duration_sec, sample_dt)
    xs = np.zeros_like(ts)
    ys = np.zeros_like(ts)
    for i, t in enumerate(ts):
        x, y = eval_trajectory(pieces, speed * t)
        xs[i] = x
        ys[i] = y
    # Velocity = finite difference / dt (wall-clock dt, not scaled-t dt)
    vx = np.gradient(xs, sample_dt)
    vy = np.gradient(ys, sample_dt)
    speed_mag = np.sqrt(vx * vx + vy * vy)
    return ts, xs, ys, speed_mag


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        '--csv', type=Path,
        default=Path('/home/rk32226/cs2_ws/src/crazyswarm2/crazyflie_examples/'
                     'crazyflie_examples/data/figure8.csv'),
    )
    ap.add_argument('--amp', type=float, default=0.375,
                    help='Figure-8 amplitude scaling (default 0.375m = beta=0.25 at leash=1.5m)')
    ap.add_argument('--max-velocity', type=float, default=1.2,
                    help='Streamer max_setpoint_velocity ceiling (m/s)')
    ap.add_argument('--speeds', type=float, nargs='+',
                    default=[1.0, 2.0, 3.0, 4.0],
                    help='figure8_speed multipliers to test')
    ap.add_argument('--duration', type=float, default=10.0,
                    help='Wall-clock duration to sample each speed (sec)')
    ap.add_argument('--out', type=Path,
                    default=WORKSPACE / 'cache/figure8_speed_test.png')
    args = ap.parse_args()

    pieces = load_pieces(args.csv)
    native_radius = 0.0
    for duration, x, y in pieces:
        n = 100
        for i in range(n):
            tt = duration * (i / (n - 1)) if n > 1 else 0
            xx, yy = eval_poly(x, tt), eval_poly(y, tt)
            native_radius = max(native_radius, abs(xx), abs(yy))
    if native_radius <= 0:
        native_radius = 1.0
    scale = args.amp / native_radius
    loop_period = sum(p[0] for p in pieces)
    print(f"Native loop period:  {loop_period:.3f} s")
    print(f"Native radius:        {native_radius:.3f} (unscaled)")
    print(f"Scale to amp={args.amp:.3f}m: {scale:.4f}")
    print()

    # Sample at each speed
    results = {}
    for sp in args.speeds:
        ts, xs, ys, vmag = trajectory_samples(pieces, sp, args.duration)
        results[sp] = {
            'ts': ts,
            'xs': xs * scale,
            'ys': ys * scale,
            'vmag': vmag * scale,
        }

    # Diagnostic numbers
    print(f"{'speed':>8} {'loop_per (s)':>14} {'peak_v (m/s)':>14} "
          f"{'under v_max?':>14}")
    print('-' * 56)
    peak_v_per_speed = []
    safe_speeds = []
    for sp in args.speeds:
        peak_v = float(results[sp]['vmag'].max())
        peak_v_per_speed.append(peak_v)
        loop_p = loop_period / sp
        under = peak_v <= args.max_velocity
        safe_speeds.append(under)
        flag = '✓' if under else '✗ exceeds!'
        print(f'{sp:>8.2f} {loop_p:>14.3f} {peak_v:>14.3f} {flag:>14}')
    # Identify highest safe speed
    highest_safe = max(
        [sp for sp, ok in zip(args.speeds, safe_speeds) if ok],
        default=None,
    )
    print()
    if highest_safe is not None:
        print(f"Highest safe figure8_speed (peak_v ≤ {args.max_velocity} m/s): {highest_safe}")
    else:
        print("WARNING: even speed=1.0 exceeds max_velocity. Reduce amp or "
              "raise max_setpoint_velocity.")
    print()

    # 4-panel figure
    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    colors = plt.cm.viridis(np.linspace(0.2, 0.95, len(args.speeds)))

    # Panel 1: XY paths (should be identical shape — all overlap)
    ax = axes[0, 0]
    for sp, c in zip(args.speeds, colors):
        ax.plot(results[sp]['xs'], results[sp]['ys'], color=c, alpha=0.6,
                linewidth=1.0, label=f'speed={sp}')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('Figure-8 XY path (all speeds — same shape, only time scales)')
    ax.set_aspect('equal')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # Panel 2: x position over time
    ax = axes[0, 1]
    for sp, c in zip(args.speeds, colors):
        ax.plot(results[sp]['ts'], results[sp]['xs'], color=c, linewidth=1.2,
                label=f'speed={sp}')
    ax.set_xlabel('wall-clock t (s)')
    ax.set_ylabel('x position (m)')
    ax.set_title('x(t) — faster speed compresses loops in time')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # Panel 3: Velocity magnitude over time
    ax = axes[1, 0]
    for sp, c in zip(args.speeds, colors):
        ax.plot(results[sp]['ts'], results[sp]['vmag'], color=c, linewidth=1.2,
                label=f'speed={sp}  peak={results[sp]["vmag"].max():.2f}')
    ax.axhline(args.max_velocity, color='red', linestyle='--', linewidth=1.5,
               label=f'max_setpoint_velocity = {args.max_velocity:.1f} m/s')
    ax.set_xlabel('wall-clock t (s)')
    ax.set_ylabel('|velocity| (m/s)')
    ax.set_title('Velocity magnitude — peaks scale ~linearly with speed')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(alpha=0.3)

    # Panel 4: Peak velocity vs speed
    ax = axes[1, 1]
    ax.plot(args.speeds, peak_v_per_speed, 'o-', linewidth=2, markersize=8,
            color='tab:blue', label='peak velocity at each speed')
    ax.axhline(args.max_velocity, color='red', linestyle='--', linewidth=1.5,
               label=f'max_setpoint_velocity ceiling')
    if highest_safe is not None:
        ax.axvline(highest_safe, color='green', linestyle=':', linewidth=1.5,
                   label=f'highest safe = {highest_safe}')
    ax.set_xlabel('figure8_speed multiplier')
    ax.set_ylabel('peak velocity (m/s)')
    ax.set_title('Peak velocity vs figure8_speed — find highest under ceiling')
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.suptitle(
        f'Figure-8 speed test  (amp={args.amp:.3f}m, native loop={loop_period:.2f}s)',
        fontsize=12, y=0.995,
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(args.out, dpi=120, bbox_inches='tight')
    print(f'Saved → {args.out}')


if __name__ == '__main__':
    main()
