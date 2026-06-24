"""Result-2-style figure, FedDCSA ONLY: deployed per-iterate r^k vs windowed-average.

Adapted from scripts/figure_result2.py. Instead of FedDCSA-vs-Lagrangian, this
contrasts the two FedDCSA OUTPUTS against the SAME moving fire (the "moving target"):
  - the deployed ITERATE r_i^k  (reactive — tracks the fire)
  - the WINDOWED-AVERAGE (EMA of the iterate; the theorem-certified object — lags)

Rows:
  1: ITERATE arena snapshots (drones at r_i^k) — tracking the moving fire (★ = fire centroid = target).
  2: WINDOWED-AVG snapshots (drones at the EMA leash) — lagging the handoff.
  3: Joint load Σ c_i r_i^2 (t): iterate vs windowed-avg + B + B_op.
  4: Per-drone r_i(t): iterate (solid) vs windowed-avg (dashed).

Usage:
  PYTHONPATH=src/thermal_mapping:src/fed_dcsa python3 scripts/figure_result2_iterate_vs_avg.py
"""
from __future__ import annotations
import math, sys
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

EMA_ALPHA = 0.10   # windowed average ~ EMA, effective window ~1/alpha ≈ 10 rounds (~20 s lag)


@dataclass
class DroneCfg:
    name: str; phi_mid: float; phi_half: float
    q_fallback: float; c: float; r_star: float; r_max: float


def load_arena(path):
    with open(path) as f: doc = yaml.safe_load(f)
    drones = []
    for name in doc["drone_names"]:
        s = doc["sectors"][name]
        drones.append(DroneCfg(name, math.radians(s["phi_mid_deg"]), math.radians(s["phi_half_deg"]),
                               float(s["q"]), float(s["c"]), float(s["r_star"]), float(s["r_max"])))
    opt = doc["optimizer"]
    return {"center_xy": tuple(doc["arena"]["center_xy"]), "drones": drones,
            "B": float(opt["budget_B"]), "T": int(opt["T"]), "c1": float(opt["c_1"]),
            "c2": float(opt["c_2"]), "noise_bound": float(opt["noise_bound"]),
            "round_rate_hz": float(opt["round_rate_hz"])}


def run_scenario(arena, field, duration, qi_threshold):
    rr = arena["round_rate_hz"]; K = int(duration * rr); dt = 1.0 / rr
    sectors = [SectorGeom(name=d.name, phi_mid=d.phi_mid, phi_half=d.phi_half,
                          r_outer=1.9, center_xy=arena["center_xy"]) for d in arena["drones"]]
    drones = [CoverageDrone(name=d.name, q=d.q_fallback, c=d.c, r_star=d.r_star,
                            r_max=d.r_max, r=0.0) for d in arena["drones"]]
    opt = RadialCoverageFedDCSA(drones=drones, B=arena["B"], T=arena["T"], c1=arena["c1"],
                                c2=arena["c2"], noise_bound=arena["noise_bound"], seed=0)
    N = len(drones); cs = np.array([d.c for d in drones])
    times = np.zeros(K); rk_it = np.zeros((K, N)); rk_av = np.zeros((K, N))
    sig_it = np.zeros(K); sig_av = np.zeros(K); ema = np.zeros(N)
    for k in range(K):
        t = k * dt; times[k] = t
        q = compute_sector_qi(field, t, sectors, threshold=qi_threshold)
        for d, qv in zip(drones, q): d.q = float(qv)
        opt.run_round(k)
        r = np.array([d.r for d in drones])
        ema = EMA_ALPHA * r + (1.0 - EMA_ALPHA) * ema
        rk_it[k] = r; rk_av[k] = ema
        sig_it[k] = float((cs * r ** 2).sum()); sig_av[k] = float((cs * ema ** 2).sum())
    return {"times": times, "K": K, "rk_it": rk_it, "rk_av": rk_av, "sig_it": sig_it,
            "sig_av": sig_av, "B": arena["B"], "B_op": 8.15, "drone_cfgs": arena["drones"],
            "field": field}


def fire_centroid(field, t, R, n=60):
    xs = np.linspace(-R, R, n); X, Y = np.meshgrid(xs, xs)
    T = field.evaluate(X, Y, t) - field.ambient
    T = np.clip(T, 0, None)
    if T.sum() < 1e-9: return None
    return (float((X * T).sum() / T.sum()), float((Y * T).sum() / T.sum()))


def draw_snapshot(ax, data, t_target, mode, colors, field_R, fuel_R, vmin, vmax,
                  grid_n=80, show_ylabel=False, show_title=True, ylabel=""):
    times = data["times"]; k = int(np.argmin(np.abs(times - t_target))); t = times[k]
    field = data["field"]; drones = data["drone_cfgs"]
    rk = data["rk_it"][k] if mode == "iterate" else data["rk_av"][k]
    sigma_val = float((data["sig_it"] if mode == "iterate" else data["sig_av"])[k])
    B_op = data["B_op"]
    xs = np.linspace(-field_R, field_R, grid_n); X, Y = np.meshgrid(xs, xs)
    Tg = field.evaluate(X, Y, t); Rg = np.sqrt(X * X + Y * Y)
    Tm = np.where(Rg <= fuel_R, Tg, field.ambient)
    ax.set_aspect("equal")
    im = ax.imshow(Tm, origin="lower", extent=(-field_R, field_R, -field_R, field_R),
                   vmin=vmin, vmax=vmax, cmap="inferno", interpolation="bilinear")
    for i, d in enumerate(drones):
        wedge = patches.Wedge((0, 0), d.r_max, math.degrees(d.phi_mid - d.phi_half),
                              math.degrees(d.phi_mid + d.phi_half),
                              facecolor=colors[i], edgecolor="none", alpha=0.10)
        ax.add_patch(wedge)
        rv = rk[i]
        if rv > 0.05:
            ap = np.linspace(d.phi_mid - d.phi_half, d.phi_mid + d.phi_half, 50)
            ax.plot(rv * np.cos(ap), rv * np.sin(ap), color=colors[i], lw=1.6, alpha=0.95)
        ax.plot(rv * math.cos(d.phi_mid), rv * math.sin(d.phi_mid), "o", color=colors[i],
                ms=6.5, markeredgecolor="white", markeredgewidth=1.0)
    # ★ fire centroid = the MOVING TARGET
    cen = fire_centroid(field, t, fuel_R)
    if cen is not None:
        ax.plot(cen[0], cen[1], "*", color="cyan", ms=15, markeredgecolor="black",
                markeredgewidth=0.8, zorder=5)
    th = np.linspace(0, 2 * np.pi, 200)
    ax.plot(fuel_R * np.cos(th), fuel_R * np.sin(th), color="white", lw=0.8, alpha=0.4)
    ax.set_xlim(-field_R, field_R); ax.set_ylim(-field_R, field_R)
    ax.set_xticks([]); ax.set_yticks([])
    if show_title: ax.set_title(f"t = {t:.0f} s", fontsize=10, pad=4)
    if show_ylabel: ax.set_ylabel(ylabel, fontsize=10.5, fontweight="bold", labelpad=6)
    viol = sigma_val > B_op
    ax.text(0.04, 0.96, f"Σ = {sigma_val:.2f}  [{'!' if viol else 'ok'}]",
            transform=ax.transAxes, fontsize=8.5, fontweight="bold", ha="left", va="top",
            color=("#a40000" if viol else "#1b5e20"),
            bbox=dict(boxstyle="round,pad=0.25", facecolor=("#fdecea" if viol else "#e6f4ea"),
                      edgecolor=("#d62728" if viol else "#2ca02c"), linewidth=1.2, alpha=0.95))
    return im


def main():
    arena = load_arena(_WS / "src/cf_coverage_planner/config/arena_4drone.yaml")
    thermal_yaml = _WS / "src/thermal_mapping/config/thermal_field.yaml"
    field = ThermalField.from_yaml(str(thermal_yaml))
    with open(thermal_yaml) as f:
        qi_threshold = float((yaml.safe_load(f).get("qi") or {}).get("threshold", 0.0))
    duration = 360.0
    data = run_scenario(arena, field, duration, qi_threshold)
    colors = ["#d62728", "#1f77b4", "#2ca02c", "#9467bd"]
    names = [d.name for d in arena["drones"]]
    fuel_R = getattr(field.fire, "arena_radius", 0.0) or 1.9
    field_R = 2.1; vmin = field.ambient; vmax = field.ambient + field.fire.peak_temperature
    handoff = (150.0, 210.0); B_op = data["B_op"]; times = data["times"]
    # RISE-focused snapshots (40-114 s — where the windowed-avg lag is most visible:
    # iterate climbs fast, average trails ~20 s behind) + one in the handoff (174).
    snap_times = [40.0, 50.0, 80.0, 114.0, 174.0]   # 5 columns

    fig = plt.figure(figsize=(16.5, 12.5))
    gs = fig.add_gridspec(4, 20, height_ratios=[1.05, 1.05, 1.0, 0.95], hspace=0.45, wspace=0.55)
    spans = [(0, 4), (4, 8), (8, 12), (12, 16), (16, 20)]   # 5 snapshot columns
    last_im = None
    for ci, (t, (a0, a1)) in enumerate(zip(snap_times, spans)):
        ax1 = fig.add_subplot(gs[0, a0:a1])
        last_im = draw_snapshot(ax1, data, t, "iterate", colors, field_R, fuel_R, vmin, vmax,
                                show_title=True, show_ylabel=(ci == 0), ylabel="deployed iterate $r^k$")
        ax2 = fig.add_subplot(gs[1, a0:a1])
        draw_snapshot(ax2, data, t, "avg", colors, field_R, fuel_R, vmin, vmax,
                      show_title=False, show_ylabel=(ci == 0), ylabel="windowed average")
    cbar_ax = fig.add_axes([0.93, 0.575, 0.012, 0.32])
    fig.colorbar(last_im, cax=cbar_ax).set_label("T (°C)", fontsize=10)

    # Row 3: Sigma(t)
    ax = fig.add_subplot(gs[2, :])
    ax.plot(times, data["sig_it"], color="#1f77b4", lw=2.0, label="Σ deployed iterate $r^k$ (reactive)")
    ax.plot(times, data["sig_av"], color="#2ca02c", lw=2.4, label="Σ windowed-average (certified, lags)")
    ax.axhline(arena["B"], color="gray", ls=":", lw=1.2, alpha=0.8, label=f"B = {arena['B']:.1f} (optimizer)")
    ax.axhline(B_op, color="#d62728", ls="-", lw=1.4, alpha=0.85, label=f"B_op = {B_op:.2f} (data loss)")
    ax.axvspan(*handoff, color="gray", alpha=0.12, label="wind-handoff window")
    ytop = max(data["sig_it"].max(), data["sig_av"].max()) * 1.10
    ax.fill_between(times, B_op, ytop, color="red", alpha=0.06)
    ax.set_xlim(0, duration); ax.set_ylim(0, ytop)
    ax.set_xlabel("Time t (s)", fontsize=11); ax.set_ylabel(r"Joint load $\sum_i c_i (r_i)^2$", fontsize=11)
    ax.set_title("(c) Joint load — deployed iterate tracks fast (small bounded excursion); windowed-average lags but stays clean",
                 fontsize=11.5, pad=6)
    ax.legend(loc="upper right", fontsize=8.5, framealpha=0.9, ncol=2); ax.grid(True, alpha=0.3)
    it_max, av_max = data["sig_it"].max(), data["sig_av"].max()
    ax.annotate(f"iterate max = {it_max:.2f}", xy=(times[int(np.argmax(data['sig_it']))], it_max),
                xytext=(times[int(np.argmax(data['sig_it']))] + 20, it_max + 0.15), fontsize=9,
                color=("#a40000" if it_max > B_op else "#1b5e20"), fontweight="bold")
    ax.annotate(f"avg max = {av_max:.2f}", xy=(times[int(np.argmax(data['sig_av']))], av_max),
                xytext=(times[int(np.argmax(data['sig_av']))] - 90, av_max - 1.0), fontsize=9,
                color=("#a40000" if av_max > B_op else "#1b5e20"), fontweight="bold")

    # Row 4: per-drone
    for i, name in enumerate(names):
        ax = fig.add_subplot(gs[3, i * 5:(i + 1) * 5])
        ax.plot(times, data["rk_it"][:, i], color=colors[i], lw=1.8, label="iterate $r^k$")
        ax.plot(times, data["rk_av"][:, i], color=colors[i], lw=1.5, ls="--", alpha=0.85, label="windowed-avg")
        ax.axvspan(*handoff, color="gray", alpha=0.12)
        ax.set_xlim(0, duration); ax.set_ylim(0, 2.0); ax.set_xlabel("t (s)", fontsize=10)
        if i == 0: ax.set_ylabel(r"$r_i$ (m)", fontsize=11)
        ax.set_title(name, fontsize=11, fontweight="bold", color=colors[i], pad=4)
        ax.grid(True, alpha=0.3)
        if i == 0: ax.legend(loc="lower right", fontsize=8, framealpha=0.9)

    fig.suptitle("FedDCSA — deployed iterate vs windowed-average (same dynamic-q handoff):  "
                 "iterate tracks the moving fire (★); windowed-average lags but stays below the data-loss threshold",
                 fontsize=12.5, fontweight="bold", y=0.995)
    fig.tight_layout(rect=[0, 0, 0.91, 0.97])
    out_dir = _WS / "exp1" / "iterate_vs_avg"; out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / "figure_result2_iterate_vs_avg.png"
    fig.savefig(out, dpi=140, bbox_inches="tight"); plt.close(fig)
    it_v = int((np.maximum(0, data["sig_it"] - B_op) > 1e-6).sum())
    av_v = int((np.maximum(0, data["sig_av"] - B_op) > 1e-6).sum())
    print(f"=== FedDCSA iterate vs windowed-avg (EMA α={EMA_ALPHA}) ===")
    print(f"  Σ iterate max={data['sig_it'].max():.3f}  ticks>B_op: {it_v}/{data['K']}")
    print(f"  Σ wndavg  max={data['sig_av'].max():.3f}  ticks>B_op: {av_v}/{data['K']}")
    print(f"  saved → {out}")


if __name__ == "__main__":
    main()
