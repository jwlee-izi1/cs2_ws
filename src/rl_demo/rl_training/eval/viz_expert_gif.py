"""Render a 2D GIF of a deterministic expert (or trained policy) running
in the pure-Python coverage env. NO Gazebo, NO ROS. Pure matplotlib.

For Phase 0 of iter12 — eyeball candidate baselines (sweep / polar rose)
before committing to BC + PPO fine-tune.

Usage:
    PYTHONPATH=src/rl_demo:src:src/thermal_mapping:src/fed_dcsa \
        python3 src/rl_demo/rl_training/eval/viz_expert_gif.py \
            --expert sweep_full --optimizer feddcsa --out /tmp/viz_sweep_full_fed.gif
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Callable

import numpy as np

_THIS = Path(__file__).resolve()
_RL_DEMO = _THIS.parents[2]
_WS_SRC = _THIS.parents[3]
for sub in (_RL_DEMO, _WS_SRC, _WS_SRC / "thermal_mapping", _WS_SRC / "fed_dcsa"):
    if str(sub) not in sys.path:
        sys.path.insert(0, str(sub))

import matplotlib
matplotlib.use("Agg")
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Wedge, Arc

from rl_training.env import CoverageEnv


# ----------------------------------------------------------------------
# Expert factory — returns a (theta, r_frac) given current env state.
# ----------------------------------------------------------------------
SECTOR_PHASE = {0.0: 0.0,
                math.pi / 2: 1.25,
                math.pi: 2.5,
                3 * math.pi / 2: 3.75}


def _phase_from_phi_mid(phi_mid_rad: float) -> float:
    """Map drone's phi_mid to its sweep phase offset (4-drone arena)."""
    # Find nearest key (modulo 2π) — phi_mid comes from yaml in degrees → radians.
    p = phi_mid_rad % (2 * math.pi)
    closest = min(SECTOR_PHASE.keys(), key=lambda k: abs(((p - k) + math.pi) % (2 * math.pi) - math.pi))
    return SECTOR_PHASE[closest]


def make_bc_expert(bc_checkpoint_path: str) -> Callable:
    """Load a BC checkpoint and return a callable that takes the env's
    observation dict directly (NOT (t, phi_mid)) and produces an action."""
    import torch
    sys.path.insert(0, str(_RL_DEMO))
    from rl_training.training.train_bc import BCPolicy, make_obs_space

    obs_space = make_obs_space()
    policy = BCPolicy(obs_space).to("cpu")
    ckpt = torch.load(bc_checkpoint_path, map_location="cpu", weights_only=False)
    policy.load_state_dict(ckpt["bc_policy_state_dict"])
    policy.eval()
    print(f"   loaded BC checkpoint: val_loss={ckpt.get('final_val_loss', '?'):.5g}, "
          f"RMSE(θ)={ckpt.get('final_rmse_theta', '?'):.4f}, "
          f"RMSE(r_frac)={ckpt.get('final_rmse_rfrac', '?'):.4f}")

    @torch.no_grad()
    def bc_expert_from_obs(obs_dict):
        t = torch.from_numpy(np.asarray(obs_dict["thermal"], dtype=np.float32)).unsqueeze(0)
        a = torch.from_numpy(np.asarray(obs_dict["age"], dtype=np.float32)).unsqueeze(0)
        v = torch.from_numpy(np.asarray(obs_dict["vec"], dtype=np.float32)).unsqueeze(0)
        out = policy({"thermal": t, "age": a, "vec": v})
        return out.squeeze(0).cpu().numpy().astype(np.float32)
    return bc_expert_from_obs


def make_hybrid_expert(ppo_checkpoint_path: str, r_frac_lock: float = 0.99) -> Callable:
    """Hybrid: load a SB3 PPO checkpoint, use policy.predict for θ, but
    HARDCODE r_frac. Preserves the policy's learned θ behavior while
    locking r at a known-good edge ratio for the demo math."""
    from stable_baselines3 import PPO
    model = PPO.load(ppo_checkpoint_path, device="cpu")
    print(f"   loaded HYBRID (PPO θ + r_frac={r_frac_lock} hardcode): n_updates={model.num_timesteps}")

    def hybrid_callable(obs_dict):
        batched = {k: np.asarray(v, dtype=np.float32)[None, ...] for k, v in obs_dict.items()}
        action, _ = model.predict(batched, deterministic=True)
        # Keep PPO's θ; override r_frac.
        theta = float(action[0][0])
        return np.array([theta, r_frac_lock], dtype=np.float32)
    return hybrid_callable


def make_ppo_expert(ppo_checkpoint_path: str) -> Callable:
    """Load a SB3 PPO checkpoint and return an obs_dict -> action callable.
    Deterministic (no exploration noise)."""
    from stable_baselines3 import PPO
    model = PPO.load(ppo_checkpoint_path, device="cpu")
    print(f"   loaded PPO checkpoint: n_updates={model.num_timesteps}")

    def ppo_callable(obs_dict):
        # SB3 expects a batched dict; we have a single obs.
        batched = {k: np.asarray(v, dtype=np.float32)[None, ...] for k, v in obs_dict.items()}
        action, _ = model.predict(batched, deterministic=True)
        return np.asarray(action[0], dtype=np.float32)
    return ppo_callable


def make_expert(name: str) -> Callable[[float, float], np.ndarray]:
    """name → expert_fn(t_s, phi_mid_rad) -> np.array([theta, r_frac])."""

    if name == "sweep_current":  # exact copy of rl_planner_node.py:344-355
        AMP, PER = 0.55, 5.0
        def expert(t, phi_mid):
            ph = _phase_from_phi_mid(phi_mid)
            return np.array([AMP * math.sin(2 * math.pi * (t + ph) / PER),
                             0.99], dtype=np.float32)
        return expert

    if name == "sweep_full":  # full ±phi_half, slower period
        AMP, PER = 1.0, 8.0
        def expert(t, phi_mid):
            ph = _phase_from_phi_mid(phi_mid)
            return np.array([AMP * math.sin(2 * math.pi * (t + ph) / PER),
                             0.99], dtype=np.float32)
        return expert

    if name == "rose_tight":  # r oscillates tightly around 0.95 ± 0.04
        AMP_T, PER_T = 1.0, 8.0
        AMP_R, PER_R, R0 = 0.04, 3.0, 0.95
        def expert(t, phi_mid):
            ph = _phase_from_phi_mid(phi_mid)
            theta = AMP_T * math.sin(2 * math.pi * (t + ph) / PER_T)
            r_frac = R0 + AMP_R * math.sin(2 * math.pi * (t + ph) / PER_R)
            return np.array([theta, r_frac], dtype=np.float32)
        return expert

    if name == "rose_wide":  # r oscillates wide: 0.71-0.99
        AMP_T, PER_T = 1.0, 8.0
        AMP_R, PER_R, R0 = 0.14, 3.0, 0.85
        def expert(t, phi_mid):
            ph = _phase_from_phi_mid(phi_mid)
            theta = AMP_T * math.sin(2 * math.pi * (t + ph) / PER_T)
            r_frac = R0 + AMP_R * math.sin(2 * math.pi * (t + ph) / PER_R)
            return np.array([theta, r_frac], dtype=np.float32)
        return expert

    raise ValueError(f"unknown expert: {name!r}")


# ----------------------------------------------------------------------
# Roll out one episode capturing per-tick state for later replay.
# ----------------------------------------------------------------------
def rollout(expert_fn, optimizer_type: str, seed: int, episode_s: float = 200.0,
            arena_yaml: str | None = None, obs_callable=None):
    """If obs_callable is provided, it takes obs_dict → action (used for BC/PPO
    policy validation). Otherwise expert_fn(t, phi_mid) → action is used."""
    kwargs = dict(
        episode_seconds=episode_s,
        tick_hz=5.0,
        action_slack=1.03,
        optimizer_type=optimizer_type,
        dynamics_model="cascaded_pid",
        randomize_dynamics=False,
        randomize_sectors=False,
        randomize_seed_range=(42, 43),
        seed=seed,
    )
    if arena_yaml is not None:
        kwargs["arena_yaml"] = arena_yaml
    env = CoverageEnv(**kwargs)

    # All shadow drones also follow the expert (or the BC obs_callable).
    if obs_callable is not None:
        env.shadow_policy_fn = obs_callable
    else:
        def shadow_policy(obs_dict):
            phi_mid = float(obs_dict["vec"][5])
            return expert_fn(env._t, phi_mid)
        env.shadow_policy_fn = shadow_policy

    obs, info = env.reset(seed=seed)
    sectors = env._cfg["sectors"]
    n_drones = len(env._drones)

    # Pre-compute fixed per-slot geometry.
    slot_phi_mid = [math.radians(sectors[env._sector_assignment[s]]["phi_mid_deg"])
                    for s in range(n_drones)]
    slot_phi_half = [math.radians(sectors[env._sector_assignment[s]]["phi_half_deg"])
                     for s in range(n_drones)]
    slot_c = [sectors[env._sector_assignment[s]]["c"] for s in range(n_drones)]
    slot_r_max = sectors[env._sector_assignment[0]]["r_max"]
    drone_names = [sectors[env._sector_assignment[s]]["name"] for s in range(n_drones)]

    ticks = {
        "t": [],
        "drone_xy": [],         # (N, 4, 2)
        "leashes": [],          # (N, 4)
        "theta_cmd": [],        # (N, 4)
        "r_frac_cmd": [],       # (N, 4)
        "sigma_drone": [],      # (N,) — Σ c·(x²+y²)
        "sigma_leash": [],      # (N,) — Σ c·r_leash²
        "k_round": [],          # (N,)
    }

    done = False
    while not done:
        # Compute focal action.
        if obs_callable is not None:
            focal_obs = env._build_obs()  # already the focal drone's obs
            action = obs_callable(focal_obs)
            # For the per-drone θ/r_frac trace we'd need to roll out 4 separate
            # BC inferences. For simplicity, record the focal action for ALL
            # drone slots as a placeholder when using BC. (The viz still shows
            # actual drone positions, leashes, Σ traces; only the θ inset is
            # approximate when obs_callable is used.)
            thetas_now = [float(action[0])] * n_drones
            rfracs_now = [float(action[1])] * n_drones
        else:
            phi_mid_focal = slot_phi_mid[env._my_drone_idx]
            action = expert_fn(env._t, phi_mid_focal)
            thetas_now = []
            rfracs_now = []
            for s in range(n_drones):
                a = expert_fn(env._t, slot_phi_mid[s])
                thetas_now.append(float(a[0]))
                rfracs_now.append(float(a[1]))

        obs, _r, term, trunc, info = env.step(action)
        done = term or trunc

        xys = np.array([env._drones[s].position[:2].copy() for s in range(n_drones)], dtype=np.float32)
        leashes = np.array(env._opt.leashes, dtype=np.float32)
        sig_drone = float(sum(slot_c[s] * (xys[s, 0] ** 2 + xys[s, 1] ** 2) for s in range(n_drones)))
        sig_leash = float(sum(slot_c[s] * leashes[s] ** 2 for s in range(n_drones)))
        ticks["t"].append(env._t)
        ticks["drone_xy"].append(xys)
        ticks["leashes"].append(leashes)
        ticks["theta_cmd"].append(thetas_now)
        ticks["r_frac_cmd"].append(rfracs_now)
        ticks["sigma_drone"].append(sig_drone)
        ticks["sigma_leash"].append(sig_leash)
        ticks["k_round"].append(env._opt.round_idx)

    out = {
        "t": np.array(ticks["t"], dtype=np.float32),
        "drone_xy": np.stack(ticks["drone_xy"], axis=0).astype(np.float32),
        "leashes": np.stack(ticks["leashes"], axis=0).astype(np.float32),
        "theta_cmd": np.array(ticks["theta_cmd"], dtype=np.float32),
        "r_frac_cmd": np.array(ticks["r_frac_cmd"], dtype=np.float32),
        "sigma_drone": np.array(ticks["sigma_drone"], dtype=np.float32),
        "sigma_leash": np.array(ticks["sigma_leash"], dtype=np.float32),
        "k_round": np.array(ticks["k_round"], dtype=np.int32),
        "phi_mid": np.array(slot_phi_mid, dtype=np.float32),
        "phi_half": np.array(slot_phi_half, dtype=np.float32),
        "c": np.array(slot_c, dtype=np.float32),
        "r_max": float(slot_r_max),
        "drone_names": np.array(drone_names),
    }

    # Capture a final-state thermal field for the GIF background (truth map).
    if env._field is not None:
        # Render the truth field on a 100×100 grid covering [-2.5, 2.5]².
        gx, gy = np.meshgrid(np.linspace(-2.5, 2.5, 100),
                             np.linspace(-2.5, 2.5, 100), indexing="xy")
        # Sample fire at episode mid-point (t/2) for a representative background.
        # This is just for visualization — not used for any policy decision.
        T_mid = env._field.evaluate(gx, gy, env._t / 2.0)
        out["fire_xy_extent"] = np.array([-2.5, 2.5, -2.5, 2.5], dtype=np.float32)
        out["fire_T_mid"] = T_mid.astype(np.float32)

    return out


# ----------------------------------------------------------------------
# GIF rendering
# ----------------------------------------------------------------------
DRONE_COLORS = ["#ffeb3b", "#f44336", "#2196f3", "#4caf50"]  # cf1..cf4


def render_gif(data: dict, out_gif: str, title_prefix: str, fps: int = 10, every: int = 5):
    """Render data dict as an animated GIF saved to out_gif.

    every: render every Nth tick (data has 1000 ticks at 5Hz=200s; with every=5 → 200 frames).
    fps: GIF playback speed.
    """
    t = data["t"]
    drone_xy = data["drone_xy"]      # (N, 4, 2)
    leashes = data["leashes"]
    theta = data["theta_cmd"]
    r_frac = data["r_frac_cmd"]
    sigma_d = data["sigma_drone"]
    sigma_l = data["sigma_leash"]
    k_round = data["k_round"]
    phi_mid = data["phi_mid"]
    phi_half = data["phi_half"]
    drone_names = data["drone_names"]
    n_drones = drone_xy.shape[1]

    keep_idx = np.arange(0, len(t), every)

    fig = plt.figure(figsize=(10, 8), facecolor="#202024")
    gs = fig.add_gridspec(3, 1, height_ratios=[5, 1.2, 0.7], hspace=0.18)
    ax = fig.add_subplot(gs[0])
    ax_sig = fig.add_subplot(gs[1])
    ax_th = fig.add_subplot(gs[2])

    # Arena setup
    ax.set_facecolor("#202024")
    ax.set_xlim(-2.5, 2.5)
    ax.set_ylim(-2.5, 2.5)
    ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])

    # Fire field background
    if "fire_T_mid" in data:
        ax.imshow(data["fire_T_mid"], cmap="turbo", vmin=22, vmax=80, alpha=0.35,
                  extent=data["fire_xy_extent"], origin="lower")

    # Sector wedge outlines
    for s in range(n_drones):
        wedge = Wedge((0, 0), data["r_max"],
                      math.degrees(phi_mid[s] - phi_half[s]),
                      math.degrees(phi_mid[s] + phi_half[s]),
                      width=data["r_max"] - 0.1,
                      facecolor="none", edgecolor=DRONE_COLORS[s],
                      linewidth=0.8, alpha=0.3)
        ax.add_patch(wedge)

    # Leash arc artists (updated per frame)
    leash_arcs = []
    drone_dots = []
    drone_labels = []
    trail_lines = []
    for s in range(n_drones):
        arc = Arc((0, 0), 1, 1,
                  theta1=math.degrees(phi_mid[s] - phi_half[s]),
                  theta2=math.degrees(phi_mid[s] + phi_half[s]),
                  color=DRONE_COLORS[s], linewidth=2.0, alpha=0.85)
        ax.add_patch(arc)
        leash_arcs.append(arc)
        d, = ax.plot([], [], "o", markersize=12, markerfacecolor=DRONE_COLORS[s],
                     markeredgecolor="white", markeredgewidth=1.2, zorder=5)
        drone_dots.append(d)
        lbl = ax.text(0, 0, drone_names[s], fontsize=8, color="white",
                      weight="bold", zorder=6)
        drone_labels.append(lbl)
        # Trail of last ~6s
        trail, = ax.plot([], [], "-", color=DRONE_COLORS[s], linewidth=1.0,
                         alpha=0.5, zorder=4)
        trail_lines.append(trail)

    # Sigma plot
    ax_sig.set_facecolor("#202024")
    ax_sig.set_xlim(0, t[-1])
    ax_sig.set_ylim(0, 12)
    ax_sig.axhline(8.15, color="#ff4444", linestyle="-", linewidth=1.0, label="B_op=8.15 (drop threshold)")
    ax_sig.axhline(8.0, color="#ffcc00", linestyle="--", linewidth=0.8, label="B=8.0 (opt goal)")
    ax_sig.set_xticks([]); ax_sig.tick_params(colors="white")
    for spine in ax_sig.spines.values():
        spine.set_color("white")
    ax_sig.set_ylabel("Σ c·r²", color="white", fontsize=9)
    line_sd, = ax_sig.plot([], [], "-", color="white", linewidth=1.6, label="Σ_drone")
    line_sl, = ax_sig.plot([], [], "-", color="#ffd54f", linewidth=1.2, label="Σ_leash")
    ax_sig.legend(loc="upper left", fontsize=7, framealpha=0.3, labelcolor="white")
    cursor = ax_sig.axvline(0, color="white", linewidth=0.6, alpha=0.6)

    # Theta plot
    ax_th.set_facecolor("#202024")
    ax_th.set_xlim(0, t[-1])
    ax_th.set_ylim(-1.05, 1.05)
    ax_th.axhline(0, color="#666", linewidth=0.5)
    ax_th.set_xlabel("t (s)", color="white", fontsize=9)
    ax_th.set_ylabel("θ", color="white", fontsize=9)
    ax_th.tick_params(colors="white", labelsize=8)
    for spine in ax_th.spines.values():
        spine.set_color("white")
    theta_lines = []
    for s in range(n_drones):
        tl, = ax_th.plot([], [], "-", color=DRONE_COLORS[s], linewidth=1.0, alpha=0.7)
        theta_lines.append(tl)

    title_text = ax.set_title("", color="white", fontsize=11)

    TRAIL_TICKS = int(6.0 * 5.0)  # 6s at 5Hz

    def update(frame_i):
        idx = keep_idx[frame_i]
        for s in range(n_drones):
            leash_arcs[s].width = 2 * leashes[idx, s]
            leash_arcs[s].height = 2 * leashes[idx, s]
            drone_dots[s].set_data([drone_xy[idx, s, 0]], [drone_xy[idx, s, 1]])
            drone_labels[s].set_position((drone_xy[idx, s, 0] + 0.1, drone_xy[idx, s, 1] + 0.1))
            # Trail
            j0 = max(0, idx - TRAIL_TICKS)
            trail_lines[s].set_data(drone_xy[j0:idx + 1, s, 0],
                                    drone_xy[j0:idx + 1, s, 1])
        # Sigma traces (incremental)
        line_sd.set_data(t[:idx + 1], sigma_d[:idx + 1])
        line_sl.set_data(t[:idx + 1], sigma_l[:idx + 1])
        cursor.set_xdata([t[idx], t[idx]])
        # Theta traces
        for s in range(n_drones):
            theta_lines[s].set_data(t[:idx + 1], theta[:idx + 1, s])
        title_text.set_text(
            f"{title_prefix}  t={t[idx]:5.1f}s  k={k_round[idx]}  "
            f"Σ_drone={sigma_d[idx]:.2f}  Σ_leash={sigma_l[idx]:.2f}  "
            f"{'VIOLATION' if sigma_d[idx] > 8.15 else 'safe'}")
        return drone_dots + leash_arcs + drone_labels + trail_lines + theta_lines + [line_sd, line_sl, cursor, title_text]

    anim = FuncAnimation(fig, update, frames=len(keep_idx), interval=1000 // fps, blit=False)
    writer = PillowWriter(fps=fps)
    anim.save(out_gif, writer=writer, dpi=80)
    plt.close(fig)


# ----------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--expert", choices=["sweep_current", "sweep_full", "rose_tight", "rose_wide", "bc", "ppo", "hybrid"], required=True,
                   help="hand-coded expert | 'bc' (--bc-checkpoint) | 'ppo' (--ppo-checkpoint) | 'hybrid' (--hybrid-checkpoint + --r-frac-lock)")
    p.add_argument("--bc-checkpoint", default=None,
                   help="path to BC pretrained checkpoint (required when --expert bc)")
    p.add_argument("--ppo-checkpoint", default=None,
                   help="path to SB3 PPO checkpoint (required when --expert ppo)")
    p.add_argument("--hybrid-checkpoint", default=None,
                   help="path to SB3 PPO checkpoint for HYBRID mode (θ from PPO, r_frac hardcoded)")
    p.add_argument("--r-frac-lock", type=float, default=0.99,
                   help="r_frac value to lock in hybrid mode (default 0.99 matches sweep_full)")
    p.add_argument("--optimizer", choices=["feddcsa", "lagrangian"], required=True)
    p.add_argument("--out", required=True, help="path to save GIF")
    p.add_argument("--data", default=None, help="optional path to save NPZ of per-tick data")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--episode-s", type=float, default=200.0)
    p.add_argument("--fps", type=int, default=10)
    p.add_argument("--every", type=int, default=5)
    p.add_argument("--arena-yaml", default=None, help="override arena_4drone.yaml path (e.g. for K=180 override)")
    args = p.parse_args()

    print(f"== rollout: expert={args.expert} optimizer={args.optimizer} seed={args.seed}"
          f"{' arena=' + args.arena_yaml if args.arena_yaml else ''} ==")
    if args.expert == "bc":
        if not args.bc_checkpoint:
            raise ValueError("--bc-checkpoint required when --expert bc")
        obs_callable = make_bc_expert(args.bc_checkpoint)
        data = rollout(None, args.optimizer, seed=args.seed, episode_s=args.episode_s,
                       arena_yaml=args.arena_yaml, obs_callable=obs_callable)
    elif args.expert == "ppo":
        if not args.ppo_checkpoint:
            raise ValueError("--ppo-checkpoint required when --expert ppo")
        obs_callable = make_ppo_expert(args.ppo_checkpoint)
        data = rollout(None, args.optimizer, seed=args.seed, episode_s=args.episode_s,
                       arena_yaml=args.arena_yaml, obs_callable=obs_callable)
    elif args.expert == "hybrid":
        if not args.hybrid_checkpoint:
            raise ValueError("--hybrid-checkpoint required when --expert hybrid")
        obs_callable = make_hybrid_expert(args.hybrid_checkpoint, r_frac_lock=args.r_frac_lock)
        data = rollout(None, args.optimizer, seed=args.seed, episode_s=args.episode_s,
                       arena_yaml=args.arena_yaml, obs_callable=obs_callable)
    else:
        expert_fn = make_expert(args.expert)
        data = rollout(expert_fn, args.optimizer, seed=args.seed, episode_s=args.episode_s,
                       arena_yaml=args.arena_yaml)
    print(f"   ticks captured: {len(data['t'])}")
    print(f"   Σ_drone max: {data['sigma_drone'].max():.2f}  (B_op=8.15)")
    print(f"   Σ_leash max: {data['sigma_leash'].max():.2f}  (B=8.0)")
    print(f"   frames over B_op: {int((data['sigma_drone'] > 8.15).sum())} / {len(data['t'])}")
    print(f"   max drone r: {np.linalg.norm(data['drone_xy'], axis=-1).max():.3f}m (r_max={data['r_max']})")
    # θ histogram per drone (entropy proxy)
    print(f"   θ range per drone (min, max):")
    for s in range(data['drone_xy'].shape[1]):
        print(f"     {data['drone_names'][s]}: ({data['theta_cmd'][:, s].min():+.2f}, {data['theta_cmd'][:, s].max():+.2f})")

    if args.data:
        np.savez_compressed(args.data, **data)
        print(f"   data → {args.data}")

    print(f"   rendering GIF → {args.out} ...")
    title = f"{args.expert} | {args.optimizer}"
    render_gif(data, args.out, title, fps=args.fps, every=args.every)
    print(f"   saved {args.out}")


if __name__ == "__main__":
    main()
