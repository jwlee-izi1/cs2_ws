"""Figure for Result 1 — static-q convergence to KKT.

3-panel figure:
  (a) Per-drone radial allocation r_i^k(k), with KKT (constrained-optimum) lines.
  (b) Joint load Σc_i r_i^2(k) relative to budget B, infeasible region shaded.
  (c) Top-down arena snapshot at converged state (k = K): sector wedges,
      radial-allocation arcs, drone markers.

Usage:
  PYTHONPATH=src/thermal_mapping:src/fed_dcsa python3 scripts/figure_result1.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

_WS = Path(__file__).resolve().parents[1]
for sub in ("src/fed_dcsa", "src/thermal_mapping"):
    p = _WS / sub
    if p.is_dir() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

from fed_dcsa.radial_coverage_algorithm import CoverageDrone, RadialCoverageFedDCSA


def main():
    # --- Run FedDCSA with static q ---
    drones = [
        CoverageDrone(name="cf1", q=10.0, c=1.0, r_star=1.85, r_max=1.9, r=0.0),
        CoverageDrone(name="cf2", q=1.5,  c=1.0, r_star=1.85, r_max=1.9, r=0.0),
        CoverageDrone(name="cf3", q=1.0,  c=1.0, r_star=1.85, r_max=1.9, r=0.0),
        CoverageDrone(name="cf4", q=0.5,  c=1.0, r_star=1.85, r_max=1.9, r=0.0),
    ]
    B = 8.0
    K = 100
    opt = RadialCoverageFedDCSA(
        drones=drones, B=B, T=5, c1=0.015, c2=0.18, noise_bound=0.015, seed=0,
    )

    r_history = np.zeros((K, 4))
    constraint_history = np.zeros(K)
    gate_history = np.zeros(K)

    for k in range(K):
        result = opt.run_round(k)
        r_history[k] = result.radii
        constraint_history[k] = sum(d.c * d.r ** 2 for d in opt.drones)
        gate_history[k] = result.gate_state

    # Analytic KKT reference (validated to ~5 cm in the leash_only_demo)
    kkt = np.array([1.78, 1.45, 1.31, 1.01])

    # --- Figure layout ---
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.3),
                             gridspec_kw={"width_ratios": [1.1, 1.1, 1.0]})

    colors = ["#d62728", "#1f77b4", "#2ca02c", "#9467bd"]
    drone_names = ["cf1", "cf2", "cf3", "cf4"]
    q_values = [10.0, 1.5, 1.0, 0.5]

    # ===== Panel (a): leash trajectories =====
    ax_a = axes[0]
    for i, (name, color, qv) in enumerate(zip(drone_names, colors, q_values)):
        ax_a.plot(np.arange(K), r_history[:, i], color=color, linewidth=2.2,
                  label=f"{name} (q={qv})")
        ax_a.axhline(y=kkt[i], color=color, linestyle="--", linewidth=1.2, alpha=0.6)
        # Label KKT (constrained-optimum) value at the right edge
        ax_a.text(K + 1.5, kkt[i], f"KKT={kkt[i]:.2f}", color=color,
                  fontsize=9, va="center", ha="left")
    ax_a.axhline(y=1.9, color="gray", linestyle=":", linewidth=1.2, alpha=0.7)
    ax_a.text(K + 1.5, 1.9, "r_max", color="gray", fontsize=9, va="center", ha="left")
    ax_a.set_xlabel("Round k", fontsize=12)
    ax_a.set_ylabel(r"Radial allocation $r_i^k$ (m)", fontsize=12)
    ax_a.set_title("(a) Per-drone radial allocation convergence", fontsize=13, pad=8)
    ax_a.legend(loc="lower right", fontsize=10, framealpha=0.9)
    ax_a.grid(True, alpha=0.3)
    ax_a.set_xlim(0, K + 14)
    ax_a.set_ylim(0, 2.05)

    # ===== Panel (b): joint constraint =====
    ax_b = axes[1]
    ax_b.plot(np.arange(K), constraint_history, color="black", linewidth=2.2,
              label=r"$\sum_i c_i (r_i^k)^2$")
    y_max = max(constraint_history.max() * 1.05, B + 1.5)
    ax_b.axhline(y=B, color="red", linestyle="-", linewidth=1.8, label=f"B = {B}")
    ax_b.fill_between(np.arange(K), B, y_max, alpha=0.12, color="red")
    ax_b.text(K - 5, B + (y_max - B) * 0.5, "infeasible region", color="red",
              fontsize=10, ha="right", va="center", style="italic", alpha=0.7)
    ax_b.set_xlabel("Round k", fontsize=12)
    ax_b.set_ylabel(r"Joint load $\sum_i c_i (r_i^k)^2$", fontsize=12)
    ax_b.set_title("(b) Joint load relative to budget B", fontsize=13, pad=8)
    ax_b.legend(loc="lower right", fontsize=10, framealpha=0.9)
    ax_b.grid(True, alpha=0.3)
    ax_b.set_xlim(0, K)
    ax_b.set_ylim(0, y_max)
    ax_b.annotate(f"max = {constraint_history.max():.3f}",
                  xy=(np.argmax(constraint_history), constraint_history.max()),
                  xytext=(K * 0.45, constraint_history.max() + 0.4),
                  arrowprops=dict(arrowstyle="->", color="black"),
                  fontsize=10, ha="center")

    # ===== Panel (c): arena snapshot at convergence =====
    ax_c = axes[2]
    ax_c.set_aspect("equal")
    sector_phi_mids_deg = [0, 90, 180, 270]
    sector_phi_half_deg = 45.0

    # Reference circles (drawn first so wedges go on top)
    theta = np.linspace(0, 2 * np.pi, 200)
    ax_c.plot(1.9 * np.cos(theta), 1.9 * np.sin(theta), "k--",
              linewidth=1.5, alpha=0.7, label="r_max=1.9")
    ax_c.plot(1.85 * np.cos(theta), 1.85 * np.sin(theta), "k:",
              linewidth=1.0, alpha=0.5, label="r* =1.85")

    for i, (name, color, _) in enumerate(zip(drone_names, colors, q_values)):
        phi_mid_deg = sector_phi_mids_deg[i]
        phi_mid_rad = np.radians(phi_mid_deg)
        # Sector wedge (light fill)
        wedge = patches.Wedge((0, 0), 1.9,
                              phi_mid_deg - sector_phi_half_deg,
                              phi_mid_deg + sector_phi_half_deg,
                              facecolor=color, edgecolor="none", alpha=0.12)
        ax_c.add_patch(wedge)
        # Converged leash arc
        r_final = r_history[-1, i]
        arc_theta = np.linspace(
            np.radians(phi_mid_deg - sector_phi_half_deg),
            np.radians(phi_mid_deg + sector_phi_half_deg),
            60,
        )
        ax_c.plot(r_final * np.cos(arc_theta), r_final * np.sin(arc_theta),
                  color=color, linewidth=2.5)
        # Drone marker
        drone_x = r_final * np.cos(phi_mid_rad)
        drone_y = r_final * np.sin(phi_mid_rad)
        ax_c.plot(drone_x, drone_y, "o", color=color,
                  markersize=11, markeredgecolor="black", markeredgewidth=1.0)
        # Label
        label_offset = 0.20
        lx = (r_final + label_offset) * np.cos(phi_mid_rad)
        ly = (r_final + label_offset) * np.sin(phi_mid_rad)
        # Shift labels slightly so they don't run off the figure
        if phi_mid_deg == 0:
            ha, va = "left", "center"
        elif phi_mid_deg == 90:
            ha, va = "center", "bottom"
        elif phi_mid_deg == 180:
            ha, va = "right", "center"
        else:
            ha, va = "center", "top"
        ax_c.annotate(f"{name}\nr={r_final:.2f} m", xy=(lx, ly),
                      ha=ha, va=va, fontsize=10, fontweight="bold",
                      color=color)

    ax_c.set_xlim(-2.2, 2.2)
    ax_c.set_ylim(-2.2, 2.2)
    ax_c.set_xlabel("x (m)", fontsize=12)
    ax_c.set_ylabel("y (m)", fontsize=12)
    ax_c.set_title(f"(c) Converged arena state (k = {K})", fontsize=13, pad=8)
    ax_c.legend(loc="lower left", fontsize=9, framealpha=0.9)
    ax_c.grid(True, alpha=0.3)
    # Origin marker
    ax_c.plot(0, 0, "+", color="black", markersize=12, markeredgewidth=2)

    # Annotation: joint constraint at convergence
    final_constraint = constraint_history[-1]
    ax_c.text(0.02, 0.98,
              f"$\\sum c_i r_i^2 = {final_constraint:.3f} \\leq B = {B}$",
              transform=ax_c.transAxes, fontsize=10, va="top", ha="left",
              bbox=dict(boxstyle="round,pad=0.4", facecolor="white",
                        edgecolor="gray", alpha=0.9))

    # Suptitle
    fig.suptitle(
        "Result 1 — Static-q convergence  "
        "(q = [10, 1.5, 1.0, 0.5],  B = 8.0,  K = 100 rounds)",
        fontsize=13, fontweight="bold", y=0.99,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    out_dir = _WS / "exp1" / "paper_figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_png = out_dir / "figure_result1.png"
    out_pdf = out_dir / "figure_result1.pdf"
    fig.savefig(out_png, dpi=160, bbox_inches="tight")
    fig.savefig(out_pdf, bbox_inches="tight")
    plt.close(fig)

    # Print summary
    print(f"=== Result 1 figure summary ===")
    print(f"  Final leashes:        {[f'{r:.3f}' for r in r_history[-1]]}")
    print(f"  KKT reference:        {[f'{r:.2f}' for r in kkt]}")
    err = np.abs(r_history[-1] - kkt)
    print(f"  |r_final - r*|:       {[f'{e:.3f}' for e in err]}")
    print(f"  Max |error|:          {err.max():.3f} m")
    print(f"  Max constraint Σ:     {constraint_history.max():.3f} (B = {B})")
    print(f"  Rounds with violation: {int((constraint_history > B).sum())}/{K}")
    print(f"  Saved → {out_png}")
    print(f"  Saved → {out_pdf}")


if __name__ == "__main__":
    main()
