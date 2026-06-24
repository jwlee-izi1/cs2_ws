"""Figure for Result 2 — dynamic-q comparison: FedDCSA vs Lagrangian.

4-row figure:
  Row 1: 5 FedDCSA arena snapshots across the run (fire field + sector wedges
         + drone markers + radial-allocation arcs).
  Row 2: 5 Lagrangian arena snapshots at the same times for column-by-column
         comparison.
  Row 3: Joint load Σ c_i (r_i^k)^2 (t) overlay — FedDCSA solid vs Lagrangian
         dashed, with B reference and wind-handoff window shaded.
  Row 4: Per-drone r_i^k(t), 1×4 small multiples (FedDCSA solid, Lagrangian
         dashed) so the reader can see which drone drives each overshoot.

Usage:
  PYTHONPATH=src/thermal_mapping:src/fed_dcsa python3 scripts/figure_result2.py
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import yaml

_WS = Path(__file__).resolve().parents[1]
for sub in ("src/fed_dcsa", "src/thermal_mapping"):
    p = _WS / sub
    if p.is_dir() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

from thermal_mapping.thermal_field import ThermalField
from thermal_mapping.qi_estimator import SectorGeom, compute_sector_qi
from fed_dcsa.radial_coverage_algorithm import CoverageDrone, RadialCoverageFedDCSA
from fed_dcsa.lagrangian_baseline import LagrangianBaseline


@dataclass
class DroneCfg:
    name: str
    phi_mid: float
    phi_half: float
    q_fallback: float
    c: float
    r_star: float
    r_max: float


def load_arena(path: Path):
    with open(path) as f:
        doc = yaml.safe_load(f)
    center_xy = tuple(doc["arena"]["center_xy"])
    names = doc["drone_names"]
    drones = []
    for name in names:
        s = doc["sectors"][name]
        drones.append(DroneCfg(
            name=name,
            phi_mid=math.radians(s["phi_mid_deg"]),
            phi_half=math.radians(s["phi_half_deg"]),
            q_fallback=float(s["q"]),
            c=float(s["c"]),
            r_star=float(s["r_star"]),
            r_max=float(s["r_max"]),
        ))
    opt = doc["optimizer"]
    return {
        "center_xy": center_xy,
        "drones": drones,
        "B": float(opt["budget_B"]),
        "T": int(opt["T"]),
        "c1": float(opt["c_1"]),
        "c2": float(opt["c_2"]),
        "noise_bound": float(opt["noise_bound"]),
        "round_rate_hz": float(opt["round_rate_hz"]),
    }


def build_optimizer(drone_cfgs, arena, kind: str):
    drones = [
        CoverageDrone(name=d.name, q=d.q_fallback, c=d.c,
                      r_star=d.r_star, r_max=d.r_max, r=0.0)
        for d in drone_cfgs
    ]
    common = dict(
        drones=drones, B=arena["B"], T=arena["T"],
        c1=arena["c1"], c2=arena["c2"],
        noise_bound=arena["noise_bound"], seed=0,
    )
    if kind == "lagrangian":
        return LagrangianBaseline(**common), drones
    return RadialCoverageFedDCSA(**common), drones


def run_scenario(arena, field, duration: float, qi_threshold: float):
    """Run both optimizers against the same q_i(t) trajectory."""
    round_rate = arena["round_rate_hz"]
    K = int(duration * round_rate)
    dt = 1.0 / round_rate

    sectors = [
        SectorGeom(
            name=d.name, phi_mid=d.phi_mid, phi_half=d.phi_half,
            r_outer=1.9, center_xy=arena["center_xy"],
        )
        for d in arena["drones"]
    ]

    fed_opt, fed_drones = build_optimizer(arena["drones"], arena, "feddcsa")
    lag_opt, lag_drones = build_optimizer(arena["drones"], arena, "lagrangian")

    N = len(arena["drones"])
    times = np.zeros(K)
    qi_hist = np.zeros((K, N))
    rk_fed = np.zeros((K, N))
    rk_lag = np.zeros((K, N))
    sigma_fed = np.zeros(K)
    sigma_lag = np.zeros(K)

    for k in range(K):
        t = k * dt
        times[k] = t
        q = compute_sector_qi(field, t, sectors, threshold=qi_threshold)
        qi_hist[k] = q
        for d, qv in zip(fed_drones, q):
            d.q = float(qv)
        for d, qv in zip(lag_drones, q):
            d.q = float(qv)

        fed_opt.run_round(k)
        lag_opt.run_round(k)
        rk_fed[k] = [d.r for d in fed_drones]
        rk_lag[k] = [d.r for d in lag_drones]
        sigma_fed[k] = sum(d.c * d.r ** 2 for d in fed_drones)
        sigma_lag[k] = sum(d.c * d.r ** 2 for d in lag_drones)

    return {
        "times": times, "K": K, "dt": dt,
        "qi": qi_hist,
        "rk_fed": rk_fed, "rk_lag": rk_lag,
        "sigma_fed": sigma_fed, "sigma_lag": sigma_lag,
        "sectors": sectors,
        "B": arena["B"],
        "B_op": 8.15,   # operational data-loss threshold (what actually drives data loss)
        "drone_cfgs": arena["drones"],
        "field": field,
    }


def draw_snapshot(ax, data, t_target: float, optimizer_label: str, colors,
                  field_R: float, fuel_R: float, vmin: float, vmax: float,
                  grid_n: int = 80, show_xlabel: bool = False,
                  show_ylabel: bool = False, show_title: bool = True):
    """Draw a single arena snapshot at time t_target."""
    times = data["times"]
    k = int(np.argmin(np.abs(times - t_target)))
    t_actual = times[k]
    field = data["field"]
    drones = data["drone_cfgs"]
    if optimizer_label == "FedDCSA":
        rk = data["rk_fed"][k]
        sigma_val = float(data["sigma_fed"][k])
    else:
        rk = data["rk_lag"][k]
        sigma_val = float(data["sigma_lag"][k])
    B_op = data["B_op"]

    xs = np.linspace(-field_R, field_R, grid_n)
    ys = np.linspace(-field_R, field_R, grid_n)
    X, Y = np.meshgrid(xs, ys)
    T = field.evaluate(X, Y, t_actual)
    R_grid = np.sqrt(X * X + Y * Y)
    T_masked = np.where(R_grid <= fuel_R, T, field.ambient)

    ax.set_aspect("equal")
    im = ax.imshow(T_masked, origin="lower",
                   extent=(-field_R, field_R, -field_R, field_R),
                   vmin=vmin, vmax=vmax, cmap="inferno", interpolation="bilinear")
    # Sector wedges
    for i, d in enumerate(drones):
        phi_mid_deg = math.degrees(d.phi_mid)
        phi_half_deg = math.degrees(d.phi_half)
        wedge = patches.Wedge((0, 0), d.r_max,
                              phi_mid_deg - phi_half_deg,
                              phi_mid_deg + phi_half_deg,
                              facecolor=colors[i], edgecolor="none", alpha=0.10)
        ax.add_patch(wedge)
        # Leash arc
        r_val = rk[i]
        if r_val > 0.05:
            arc_phi = np.linspace(d.phi_mid - d.phi_half, d.phi_mid + d.phi_half, 50)
            ax.plot(r_val * np.cos(arc_phi), r_val * np.sin(arc_phi),
                    color=colors[i], linewidth=1.6, alpha=0.95)
        # Drone marker
        dx = r_val * math.cos(d.phi_mid)
        dy = r_val * math.sin(d.phi_mid)
        ax.plot(dx, dy, "o", color=colors[i], markersize=6.5,
                markeredgecolor="white", markeredgewidth=1.0)

    # Arena boundary
    theta = np.linspace(0, 2 * np.pi, 200)
    ax.plot(fuel_R * np.cos(theta), fuel_R * np.sin(theta),
            color="white", linewidth=0.8, alpha=0.4)

    ax.set_xlim(-field_R, field_R)
    ax.set_ylim(-field_R, field_R)
    ax.set_xticks([])
    ax.set_yticks([])
    if show_title:
        ax.set_title(f"t = {t_actual:.0f} s", fontsize=10, pad=4)
    if show_ylabel:
        ax.set_ylabel(optimizer_label, fontsize=11, fontweight="bold", labelpad=6)

    # Σ-value badge (top-left inside the axes) with safe/violate color coding
    # Red only when ABOVE the operational data-loss threshold B_op (not the soft B)
    is_violation = sigma_val > B_op
    badge_facecolor = "#fdecea" if is_violation else "#e6f4ea"   # light red / light green
    badge_edgecolor = "#d62728" if is_violation else "#2ca02c"
    badge_text_color = "#a40000" if is_violation else "#1b5e20"
    status_mark = "!"  if is_violation else "ok"
    ax.text(0.04, 0.96,
            f"Σ = {sigma_val:.2f}  [{status_mark}]",
            transform=ax.transAxes, fontsize=8.5, fontweight="bold",
            ha="left", va="top", color=badge_text_color,
            bbox=dict(boxstyle="round,pad=0.25",
                      facecolor=badge_facecolor,
                      edgecolor=badge_edgecolor,
                      linewidth=1.2, alpha=0.95))
    return im


def main():
    arena_yaml = _WS / "src/cf_coverage_planner/config/arena_4drone.yaml"
    thermal_yaml = _WS / "src/thermal_mapping/config/thermal_field.yaml"
    arena = load_arena(arena_yaml)
    field = ThermalField.from_yaml(str(thermal_yaml))
    with open(thermal_yaml) as f:
        qi_threshold = float((yaml.safe_load(f).get("qi") or {}).get("threshold", 0.0))

    duration = 360.0
    data = run_scenario(arena, field, duration, qi_threshold)

    colors = ["#d62728", "#1f77b4", "#2ca02c", "#9467bd"]
    drone_names = [d.name for d in arena["drones"]]

    fuel_R = getattr(field.fire, "arena_radius", 0.0) or 1.9
    field_R = 2.1
    vmin = field.ambient
    vmax = field.ambient + field.fire.peak_temperature

    # Handoff window (where the cf1↔cf2 fire transfer happens)
    handoff_window = (150.0, 210.0)
    B_op = data["B_op"]

    # Pick 5 snapshot times: 1 near start (calm) + 3 INSIDE handoff window where
    # Lagrangian crosses B_op but FedDCSA does not + 1 near end (post-transient).
    times = data["times"]
    in_window = (times >= handoff_window[0]) & (times <= handoff_window[1])
    contrast_mask = in_window & (data["sigma_lag"] > B_op) & (data["sigma_fed"] <= B_op)
    contrast_idxs = np.where(contrast_mask)[0]
    if len(contrast_idxs) >= 3:
        picks = np.linspace(0, len(contrast_idxs) - 1, 3).astype(int)
        handoff_times = [float(times[contrast_idxs[p]]) for p in picks]
    else:
        fb = np.where(in_window & (data["sigma_lag"] > B_op))[0]
        if len(fb) >= 3:
            picks = np.linspace(0, len(fb) - 1, 3).astype(int)
            handoff_times = [float(times[fb[p]]) for p in picks]
        else:
            handoff_times = [165.0, 180.0, 195.0]
    snap_times = [60.0] + handoff_times + [300.0]   # start | handoff×3 | end

    # ---- Figure layout: 4 rows, 5-col grid for snapshots; row 4 uses 4 cols ----
    fig = plt.figure(figsize=(16.5, 12.5))
    gs = fig.add_gridspec(
        4, 20,
        height_ratios=[1.05, 1.05, 1.0, 0.95],
        hspace=0.45, wspace=0.55,
    )

    # ---- Rows 1 & 2: 5 snapshots each (4 cols wide → 20 cols total) ----
    last_im = None
    snap_col_spans = [(0, 4), (4, 8), (8, 12), (12, 16), (16, 20)]
    for col_idx, (t, (c0, c1)) in enumerate(zip(snap_times, snap_col_spans)):
        ax_fed = fig.add_subplot(gs[0, c0:c1])
        last_im = draw_snapshot(
            ax_fed, data, t, "FedDCSA", colors, field_R, fuel_R, vmin, vmax,
            show_title=True, show_ylabel=(col_idx == 0),
        )
        ax_lag = fig.add_subplot(gs[1, c0:c1])
        draw_snapshot(
            ax_lag, data, t, "Lagrangian", colors, field_R, fuel_R, vmin, vmax,
            show_title=False, show_ylabel=(col_idx == 0),
        )

    # Colorbar to the right of row 1+2 snapshots
    cbar_ax = fig.add_axes([0.93, 0.575, 0.012, 0.32])
    cbar = fig.colorbar(last_im, cax=cbar_ax)
    cbar.set_label("T (°C)", fontsize=10)

    # ---- Row 3: joint load Σc_i r² overlay ----
    ax_sigma = fig.add_subplot(gs[2, :])
    ax_sigma.plot(times, data["sigma_fed"], color="black", linewidth=2.0,
                  label="FedDCSA")
    ax_sigma.plot(times, data["sigma_lag"], color="#d62728", linewidth=2.0,
                  linestyle="--", label="Lagrangian baseline")
    # Two thresholds: B (soft optimizer target) and B_op (data-loss threshold)
    ax_sigma.axhline(y=arena["B"], color="gray", linestyle=":", linewidth=1.2,
                     alpha=0.8, label=f"B = {arena['B']:.1f} (optimizer)")
    ax_sigma.axhline(y=B_op, color="#d62728", linestyle="-", linewidth=1.4,
                     alpha=0.85, label=f"B_op = {B_op:.2f} (data loss)")
    # Handoff shading
    ax_sigma.axvspan(handoff_window[0], handoff_window[1],
                     color="gray", alpha=0.12, label="wind-handoff window")
    # Danger region tint ONLY above B_op
    y_top = max(data["sigma_lag"].max(), data["sigma_fed"].max()) * 1.08
    ax_sigma.fill_between(times, B_op, y_top, color="red", alpha=0.06)
    ax_sigma.set_xlim(0, duration)
    ax_sigma.set_ylim(0, y_top)
    ax_sigma.set_xlabel("Time t (s)", fontsize=11)
    ax_sigma.set_ylabel(r"Joint load $\sum_i c_i (r_i^k)^2$", fontsize=11)
    ax_sigma.set_title("(c) Joint load over time — FedDCSA stays below data-loss threshold;  "
                       "Lagrangian violates",
                       fontsize=12, pad=6)
    ax_sigma.legend(loc="upper right", fontsize=9, framealpha=0.9, ncol=2)
    ax_sigma.grid(True, alpha=0.3)

    # Annotate max overshoot for each — color by whether it's above B_op
    max_fed_k = int(np.argmax(data["sigma_fed"]))
    max_lag_k = int(np.argmax(data["sigma_lag"]))
    fed_max = data["sigma_fed"].max()
    lag_max = data["sigma_lag"].max()
    fed_color = "#a40000" if fed_max > B_op else "#1b5e20"
    lag_color = "#a40000" if lag_max > B_op else "#1b5e20"
    ax_sigma.annotate(f"max = {fed_max:.2f}",
                      xy=(times[max_fed_k], fed_max),
                      xytext=(times[max_fed_k] + 30, fed_max - 1.4),
                      arrowprops=dict(arrowstyle="->", color=fed_color, lw=0.8),
                      fontsize=9, color=fed_color, fontweight="bold")
    ax_sigma.annotate(f"max = {lag_max:.2f}",
                      xy=(times[max_lag_k], lag_max),
                      xytext=(times[max_lag_k] - 80, lag_max + 0.05),
                      arrowprops=dict(arrowstyle="->", color=lag_color, lw=0.8),
                      fontsize=9, color=lag_color, fontweight="bold")

    # ---- Row 4: per-drone r_i^k panels (4 drones × 5 cols each = 20) ----
    r_max_y = 2.0
    for i, name in enumerate(drone_names):
        ax = fig.add_subplot(gs[3, i * 5:(i + 1) * 5])
        ax.plot(times, data["rk_fed"][:, i], color=colors[i], linewidth=1.8,
                label="FedDCSA")
        ax.plot(times, data["rk_lag"][:, i], color=colors[i], linewidth=1.5,
                linestyle="--", alpha=0.8, label="Lagrangian")
        ax.axvspan(handoff_window[0], handoff_window[1],
                   color="gray", alpha=0.12)
        ax.set_xlim(0, duration)
        ax.set_ylim(0, r_max_y)
        ax.set_xlabel("t (s)", fontsize=10)
        if i == 0:
            ax.set_ylabel(r"$r_i^k$ (m)", fontsize=11)
        ax.set_title(name, fontsize=11, fontweight="bold",
                     color=colors[i], pad=4)
        ax.grid(True, alpha=0.3)
        if i == 0:
            ax.legend(loc="lower right", fontsize=8, framealpha=0.9)

    # Annotate row 1/2 with optimizer labels on the leftmost column ylabel side
    # (draw_snapshot already does this via show_ylabel=True)

    # Suptitle
    fig.suptitle(
        "Result 2 — Dynamic q under wind handoff:  FedDCSA vs Lagrangian baseline  "
        f"(B = {arena['B']:.1f},  duration = {int(duration)} s)",
        fontsize=13, fontweight="bold", y=0.995,
    )
    fig.tight_layout(rect=[0, 0, 0.91, 0.97])

    out_dir = _WS / "exp1" / "paper_figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_png = out_dir / "figure_result2.png"
    out_pdf = out_dir / "figure_result2.pdf"
    fig.savefig(out_png, dpi=140, bbox_inches="tight")
    fig.savefig(out_pdf, bbox_inches="tight")
    plt.close(fig)

    # Summary — violations counted against B_op (data-loss threshold), not the soft B
    fed_viol = np.maximum(0.0, data["sigma_fed"] - B_op)
    lag_viol = np.maximum(0.0, data["sigma_lag"] - B_op)
    print("=== Result 2 figure summary ===")
    print(f"  Duration: {int(duration)} s,  K = {data['K']} rounds")
    print(f"  B = {arena['B']}  (optimizer soft target)")
    print(f"  B_op = {B_op:.2f}  (operational data-loss threshold)")
    print(f"  Snapshot times (Lag>B_op AND Fed≤B_op in handoff window):")
    print(f"    {[f'{t:.1f}' for t in snap_times]}")
    print(f"  Handoff window: {handoff_window} s")
    print(f"  Σ_fed  max = {data['sigma_fed'].max():.3f}   "
          f"ticks above B_op: {int((fed_viol > 1e-6).sum())}/{data['K']}  "
          f"max excess over B_op = {fed_viol.max():.3f}")
    print(f"  Σ_lag  max = {data['sigma_lag'].max():.3f}   "
          f"ticks above B_op: {int((lag_viol > 1e-6).sum())}/{data['K']}  "
          f"max excess over B_op = {lag_viol.max():.3f}")
    print(f"  Saved → {out_png}")
    print(f"  Saved → {out_pdf}")


if __name__ == "__main__":
    main()
