#!/usr/bin/env python3
"""Real-time animation of the dynamic-thermal-field scenario (Phase A).

Renders the 6-min run as a GIF:
  - Truth field heatmap T(x,y,t) evolving
  - 4 colored sector wedges, each filled to radius r_i^k(t) — the drone's
    current leash. As the optimizer reallocates budget, you SEE the wedges
    grow and shrink in sync with q_i.
  - Sidebar: q_i bars per sector + r_i^k / r_i^* numerics + time

Same math as preview_thermal_scenario.py — just a different output format
to make the chase visually intuitive.

Run:
    python3 scripts/animate_thermal_scenario.py
    python3 scripts/animate_thermal_scenario.py --frames 180 --fps 12
"""

import argparse
import math
import sys
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

WORKSPACE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WORKSPACE / 'src' / 'thermal_mapping'))
sys.path.insert(0, str(WORKSPACE / 'src' / 'fed_dcsa'))

from thermal_mapping.thermal_field import ThermalField  # noqa: E402
from thermal_mapping.qi_estimator import SectorGeom, compute_sector_qi  # noqa: E402
from fed_dcsa.radial_coverage_algorithm import CoverageDrone, RadialCoverageFedDCSA  # noqa: E402
from fed_dcsa.lagrangian_baseline import LagrangianBaseline  # noqa: E402

# Reuse loader from the previewer
sys.path.insert(0, str(WORKSPACE / 'scripts'))
from preview_thermal_scenario import load_arena, solve_static_kkt  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--thermal-yaml', type=Path,
                    default=WORKSPACE / 'src/thermal_mapping/config/thermal_field.yaml')
    ap.add_argument('--arena-yaml', type=Path,
                    default=WORKSPACE / 'src/cf_coverage_planner/config/arena_4drone.yaml')
    ap.add_argument('--duration', type=float, default=360.0)
    ap.add_argument('--frames', type=int, default=180,
                    help='Number of animation frames (default 180, ~ 1 per K at 0.5 Hz)')
    ap.add_argument('--fps', type=int, default=12)
    ap.add_argument('--qi-threshold', type=float, default=None,
                    help='Override qi.threshold from YAML; 0 = mean over all cells')
    ap.add_argument('--out', type=Path,
                    default=WORKSPACE / 'cache/thermal_scenario.gif')
    ap.add_argument('--r-integration', type=float, default=1.9)
    args = ap.parse_args()

    field = ThermalField.from_yaml(str(args.thermal_yaml))
    arena = load_arena(args.arena_yaml)

    # Read qi threshold (YAML default) unless overridden
    qi_threshold = args.qi_threshold
    if qi_threshold is None:
        import yaml
        with open(args.thermal_yaml) as f:
            doc = yaml.safe_load(f)
        qi_threshold = float((doc.get('qi') or {}).get('threshold', 0.0))

    round_rate = arena['round_rate_hz']
    K_max = int(args.duration * round_rate)
    dt = 1.0 / round_rate
    N = len(arena['drones'])

    print(f"Animating {args.duration:.0f} s scenario:")
    print(f"  K_max         = {K_max}  ({round_rate} Hz × {args.duration:.0f}s)")
    print(f"  frames        = {args.frames} at {args.fps} fps  → {args.frames/args.fps:.1f} s GIF")
    print(f"  qi.threshold  = {qi_threshold}")
    print(f"  fire ignition = ({field.fire.ignition_x:.2f}, {field.fire.ignition_y:.2f})")
    print(f"  wind          = dir={field.fire.wind_direction_deg}° anisotropy={field.fire.anisotropy}")
    print()

    # Sector geometry
    sectors = [
        SectorGeom(d.name, d.phi_mid, d.phi_half, args.r_integration,
                   arena['center_xy']) for d in arena['drones']
    ]

    # Pre-simulate: BOTH FedDCSA and Lagrangian baseline on same q_i sequence
    opt_drones = [
        CoverageDrone(name=d.name, q=d.q_fallback, c=d.c,
                       r_star=d.r_star, r_max=d.r_max, r=0.0)
        for d in arena['drones']
    ]
    optimizer = RadialCoverageFedDCSA(
        drones=opt_drones,
        B=arena['B'], T=arena['T'],
        c1=arena['c1'], c2=arena['c2'],
        noise_bound=arena['noise_bound'], seed=0,
    )
    base_drones = [
        CoverageDrone(name=d.name, q=d.q_fallback, c=d.c,
                       r_star=d.r_star, r_max=d.r_max, r=0.0)
        for d in arena['drones']
    ]
    baseline = LagrangianBaseline(
        drones=base_drones,
        B=arena['B'], T=arena['T'],
        c1=arena['c1'], c2=arena['c2'],
        noise_bound=arena['noise_bound'], seed=0,
    )

    c_array = np.array([d.c for d in arena['drones']])
    r_max_array = np.array([d.r_max for d in arena['drones']])
    r_star_array = np.array([d.r_star for d in arena['drones']])

    qi_history = np.zeros((K_max, N), dtype=np.float32)
    rk_history = np.zeros((K_max, N), dtype=np.float32)
    rk_base_history = np.zeros((K_max, N), dtype=np.float32)
    rstar_history = np.zeros((K_max, N), dtype=np.float32)
    constraint_history = np.zeros(K_max, dtype=np.float32)
    constraint_base_history = np.zeros(K_max, dtype=np.float32)
    for k in range(K_max):
        t = k * dt
        qi_now = compute_sector_qi(field, t, sectors, threshold=qi_threshold)
        qi_history[k] = qi_now
        for i, d in enumerate(opt_drones):
            d.q = float(qi_now[i])
        for i, d in enumerate(base_drones):
            d.q = float(qi_now[i])
        rstar_history[k] = solve_static_kkt(
            qi_now, c_array, r_star_array, r_max_array, arena['B'],
        )
        result = optimizer.run_round(k)
        rk_history[k] = result.radii
        constraint_history[k] = sum(d.c * d.r ** 2 for d in opt_drones)
        result_b = baseline.run_round(k)
        rk_base_history[k] = result_b.radii
        constraint_base_history[k] = sum(d.c * d.r ** 2 for d in base_drones)
    print(f"Pre-simulation complete. q_i range: "
          f"min={qi_history.min():.2f}  max={qi_history.max():.2f}")
    print(f"FedDCSA  r range: {rk_history.min():.2f} - {rk_history.max():.2f}")
    print(f"Baseline r range: {rk_base_history.min():.2f} - {rk_base_history.max():.2f}")
    print(f"FedDCSA  max violation: {(constraint_history - arena['B']).max():.3f}")
    print(f"Baseline max violation: {(constraint_base_history - arena['B']).max():.3f}")
    print()

    # Truth-field grid (cached per-frame; smaller than previewer for speed)
    grid_n = 80
    arena_R = field.fire.arena_radius + 0.2 if field.fire.arena_radius > 0 else args.r_integration
    fuel_R = field.fire.arena_radius
    xs = np.linspace(-arena_R, arena_R, grid_n)
    ys = np.linspace(-arena_R, arena_R, grid_n)
    X, Y = np.meshgrid(xs, ys)

    # Set up figure: two field panels side-by-side (FedDCSA, Baseline) + violation chart + sidebar
    fig = plt.figure(figsize=(18, 9))
    gs = fig.add_gridspec(
        2, 4, width_ratios=[1.5, 1.5, 0.05, 1.2],
        height_ratios=[2, 1],
        hspace=0.30, wspace=0.15,
    )
    ax_fed = fig.add_subplot(gs[0, 0])       # FedDCSA field+leashes
    ax_base = fig.add_subplot(gs[0, 1])      # Baseline field+leashes
    cax = fig.add_subplot(gs[0, 2])
    ax_side = fig.add_subplot(gs[0, 3])
    ax_violation = fig.add_subplot(gs[1, :3])  # Σc_i r² over time, both algorithms
    ax_legend = fig.add_subplot(gs[1, 3])
    ax_legend.axis('off')

    # Field heatmap (same for both — fire is shared)
    vmin = field.ambient
    vmax = field.ambient + field.fire.peak_temperature
    T_init = field.evaluate(X, Y, 0.0)
    im_fed = ax_fed.imshow(
        T_init, extent=(-arena_R, arena_R, -arena_R, arena_R),
        origin='lower', cmap='inferno', vmin=vmin, vmax=vmax,
        interpolation='bilinear',
    )
    im_base = ax_base.imshow(
        T_init, extent=(-arena_R, arena_R, -arena_R, arena_R),
        origin='lower', cmap='inferno', vmin=vmin, vmax=vmax,
        interpolation='bilinear',
    )
    fig.colorbar(im_fed, cax=cax, label='°C')

    # Sector wedge patches for BOTH axes (FedDCSA and baseline)
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']
    wedge_fed = []
    wedge_base = []
    for i, sec in enumerate(sectors):
        deg_mid = math.degrees(sec.phi_mid)
        deg_half = math.degrees(sec.phi_half)
        wf = mpatches.Wedge(
            center=sec.center_xy, r=0.0,
            theta1=deg_mid - deg_half, theta2=deg_mid + deg_half,
            facecolor=colors[i], edgecolor='white', linewidth=1.0,
            alpha=0.35, zorder=3,
        )
        ax_fed.add_patch(wf)
        wedge_fed.append(wf)
        wb = mpatches.Wedge(
            center=sec.center_xy, r=0.0,
            theta1=deg_mid - deg_half, theta2=deg_mid + deg_half,
            facecolor=colors[i], edgecolor='white', linewidth=1.0,
            alpha=0.35, zorder=3,
        )
        ax_base.add_patch(wb)
        wedge_base.append(wb)

    # Sector boundary lines + fuel disc + ignition marker (BOTH axes)
    for ax_panel in (ax_fed, ax_base):
        for sec in sectors:
            for sign in (-1, +1):
                phi = sec.phi_mid + sign * sec.phi_half
                ax_panel.plot([0, args.r_integration * math.cos(phi)],
                              [0, args.r_integration * math.sin(phi)],
                              color='white', alpha=0.25, linewidth=0.6, zorder=2)
        if fuel_R > 0:
            theta = np.linspace(0, 2 * math.pi, 200)
            ax_panel.plot(fuel_R * np.cos(theta), fuel_R * np.sin(theta),
                          color='cyan', alpha=0.6, linewidth=1.0,
                          linestyle='--', zorder=2)
        ax_panel.scatter(
            [field.fire.ignition_x], [field.fire.ignition_y],
            s=40, c='cyan', marker='*', edgecolors='white', linewidth=0.8, zorder=4,
        )
        ax_panel.set_xlim(-arena_R, arena_R)
        ax_panel.set_ylim(-arena_R, arena_R)
        ax_panel.set_xlabel('x (m)')
        ax_panel.set_aspect('equal')
        ax_panel.set_xticks([])
        ax_panel.set_yticks([])
    ax_fed.set_ylabel('y (m)')
    title_fed = ax_fed.set_title('FedDCSA — t = 0.0 s', color='tab:blue', fontweight='bold')
    title_base = ax_base.set_title('Lagrangian baseline — t = 0.0 s', color='tab:red', fontweight='bold')

    # Bottom panel: constraint value Σc_i r² over time, BOTH algorithms
    K_axis = np.arange(K_max)
    ax_violation.plot(K_axis, constraint_history, color='tab:blue',
                      linewidth=1.5, label='FedDCSA  Σc_i (r_i^k)²')
    ax_violation.plot(K_axis, constraint_base_history, color='tab:red',
                      linewidth=1.5, linestyle='--',
                      label='Baseline Σc_i (r_i^k)²')
    ax_violation.axhline(arena['B'], color='black', linewidth=1.0,
                          alpha=0.8, label=f'B = {arena["B"]}')
    # Fill violation regions for baseline (where it exceeds B)
    ax_violation.fill_between(
        K_axis, arena['B'], constraint_base_history,
        where=(constraint_base_history > arena['B']),
        color='red', alpha=0.25, label='Baseline VIOLATION'
    )
    ax_violation.set_xlabel('K (optimizer round)')
    ax_violation.set_ylabel('Σ c_i (r_i^k)²')
    ax_violation.set_title('Per-iterate constraint value — red shading = baseline violating B')
    ax_violation.legend(loc='upper left', fontsize=8, ncol=2)
    ax_violation.grid(alpha=0.3)
    # Vertical time-cursor that moves with the animation
    time_cursor = ax_violation.axvline(0, color='black', linewidth=1.0, alpha=0.5)

    # Sidebar: q_i bars + numerics
    ax_side.set_xlim(0, 1)
    ax_side.set_ylim(0, 1)
    ax_side.axis('off')
    # q_i bars (horizontal): one row per drone
    q_max = max(0.1, float(qi_history.max()) * 1.1)
    text_artists = []
    bar_patches = []
    for i, d in enumerate(arena['drones']):
        y_top = 0.95 - i * 0.22
        ax_side.text(0.0, y_top, f'{d.name}', color=colors[i],
                     fontsize=12, fontweight='bold', transform=ax_side.transAxes)
        bar = mpatches.Rectangle((0.20, y_top - 0.06), 0.0, 0.05,
                                   facecolor=colors[i], alpha=0.7,
                                   transform=ax_side.transAxes)
        ax_side.add_patch(bar)
        bar_patches.append(bar)
        t_qi = ax_side.text(0.85, y_top - 0.04, '', color=colors[i],
                            fontsize=9, transform=ax_side.transAxes, ha='right')
        t_rk = ax_side.text(0.0, y_top - 0.10, '', color='black',
                            fontsize=9, transform=ax_side.transAxes)
        t_rs = ax_side.text(0.50, y_top - 0.10, '', color='gray',
                            fontsize=9, transform=ax_side.transAxes)
        text_artists.append((t_qi, t_rk, t_rs))
    # Headers
    ax_side.text(0.20, 0.99, f'q_i  (max ~{q_max:.1f})', fontsize=9,
                 transform=ax_side.transAxes, color='dimgray')
    ax_side.text(0.0, 0.05, 'r_i^k = drone leash    '
                            'r_i^* = KKT optimum',
                 fontsize=8, transform=ax_side.transAxes, color='dimgray')

    # Animation update
    frame_to_k = np.linspace(0, K_max - 1, args.frames).astype(int)

    def update(frame_idx):
        k = int(frame_to_k[frame_idx])
        t = k * dt
        # Update field (same heatmap for both panels)
        T_grid = field.evaluate(X, Y, t)
        im_fed.set_data(T_grid)
        im_base.set_data(T_grid)
        # Update FedDCSA wedges
        for i, w in enumerate(wedge_fed):
            w.set_radius(float(rk_history[k, i]))
        # Update baseline wedges
        for i, w in enumerate(wedge_base):
            w.set_radius(float(rk_base_history[k, i]))
        # Update sidebar (showing FedDCSA values + baseline r in parens)
        for i in range(N):
            qi_v = float(qi_history[k, i])
            bar_patches[i].set_width(0.62 * (qi_v / q_max))
            t_qi, t_rk, t_rs = text_artists[i]
            t_qi.set_text(f'{qi_v:.2f}')
            t_rk.set_text(f'r^k={rk_history[k, i]:.2f} (b:{rk_base_history[k, i]:.2f})')
            t_rs.set_text(f'r*={rstar_history[k, i]:.2f}')
        # Update titles, highlight baseline if currently violating
        title_fed.set_text(f'FedDCSA — t = {t:.1f}s  '
                           f'Σc_i r² = {constraint_history[k]:.2f}')
        base_violating = constraint_base_history[k] > arena['B']
        viol_str = '  ⚠ VIOLATION' if base_violating else ''
        title_base.set_text(f'Baseline — t = {t:.1f}s  '
                            f'Σc_i r² = {constraint_base_history[k]:.2f}{viol_str}')
        title_base.set_color('darkred' if base_violating else 'tab:red')
        # Move time cursor on violation plot
        time_cursor.set_xdata([k, k])
        return [im_fed, im_base] + wedge_fed + wedge_base + bar_patches + \
               [a for tx in text_artists for a in tx] + \
               [title_fed, title_base, time_cursor]

    anim = animation.FuncAnimation(
        fig, update, frames=args.frames, interval=1000 / args.fps,
        blit=False,  # blit gets weird with text + patches mixing
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    print(f"Rendering {args.frames} frames → {args.out} ...")
    writer = animation.PillowWriter(fps=args.fps)
    anim.save(args.out, writer=writer, dpi=90)
    print(f"Saved: {args.out}")


if __name__ == '__main__':
    main()
