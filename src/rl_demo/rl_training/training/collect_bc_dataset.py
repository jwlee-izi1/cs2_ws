"""Generate a Behavior-Cloning dataset by running the sweep_full expert
in the pure-Python coverage env across many seeds and sector assignments.

Each tick produces one (obs, action) training pair where action is the
deterministic sweep_full expert. Saves an NPZ that train_bc.py consumes.

Usage:
    PYTHONPATH=src/rl_demo:src:src/thermal_mapping:src/fed_dcsa \
        python3 src/rl_demo/rl_training/training/collect_bc_dataset.py \
            --n-episodes 100 --out exp1/rl_training/bc_data/bc_sweep_full.npz
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

_THIS = Path(__file__).resolve()
_RL_DEMO = _THIS.parents[2]
_WS_SRC = _THIS.parents[3]
for sub in (_RL_DEMO, _WS_SRC, _WS_SRC / "thermal_mapping", _WS_SRC / "fed_dcsa"):
    if str(sub) not in sys.path:
        sys.path.insert(0, str(sub))

from rl_training.env import CoverageEnv


# ----------------------------------------------------------------------
# sweep_full expert (Phase 0 winner).
#   theta = sin(2π(t + PHASE) / SWEEP_PERIOD)   amplitude = 1.0 (full ±phi_half)
#   r_fraction = 0.99
# PHASE depends on the drone's sector (we infer it from phi_mid in the obs vec).
# ----------------------------------------------------------------------
SWEEP_AMP = 1.0
SWEEP_PERIOD = 8.0
SECTOR_PHASE = {0.0: 0.0,
                math.pi / 2: 1.25,
                math.pi: 2.5,
                3 * math.pi / 2: 3.75}


def _phase_from_phi_mid(phi_mid_rad: float) -> float:
    p = phi_mid_rad % (2 * math.pi)
    closest = min(
        SECTOR_PHASE.keys(),
        key=lambda k: abs(((p - k) + math.pi) % (2 * math.pi) - math.pi),
    )
    return SECTOR_PHASE[closest]


def expert_action(t_s: float, phi_mid_rad: float) -> np.ndarray:
    ph = _phase_from_phi_mid(phi_mid_rad)
    theta = SWEEP_AMP * math.sin(2 * math.pi * (t_s + ph) / SWEEP_PERIOD)
    r_frac = 0.99
    return np.array([theta, r_frac], dtype=np.float32)


# ----------------------------------------------------------------------
# Dataset collection loop.
# ----------------------------------------------------------------------
def collect(n_episodes: int, optimizer_type: str, arena_yaml: str | None = None,
            episode_seconds: float = 360.0, verbose: bool = True) -> dict:
    """Collect (obs, action) pairs.

    Returns a dict with stacked numpy arrays ready for np.savez_compressed.
    """
    obs_thermal = []
    obs_age = []
    obs_vec = []
    actions = []
    ep_id = []
    drone_slot_id = []
    t_in_ep = []

    for ep in range(n_episodes):
        kwargs = dict(
            episode_seconds=episode_seconds,
            tick_hz=5.0,
            action_slack=1.03,
            optimizer_type=optimizer_type,
            dynamics_model="cascaded_pid",
            randomize_dynamics=True,        # randomize PID gains for sim2real robustness
            randomize_sectors=True,         # each episode shuffles which sector this slot gets
            randomize_seed_range=(0, 10_000_000),
            seed=ep,
        )
        if arena_yaml is not None:
            kwargs["arena_yaml"] = arena_yaml
        env = CoverageEnv(**kwargs)

        # Shadow drones also follow the same sweep_full expert (so the env's other
        # 3 drones aren't stuck on the heuristic 0.85-centerline — that would
        # train BC on observations from an off-distribution multi-drone scene).
        def shadow_policy(o):
            phi_mid_shadow = float(o["vec"][5])
            return expert_action(env._t, phi_mid_shadow)
        env.shadow_policy_fn = shadow_policy

        obs, info = env.reset(seed=ep)
        done = False
        # phi_mid of the focal (controlled) drone for this episode.
        phi_mid_focal = float(obs["vec"][5])

        while not done:
            a = expert_action(env._t, phi_mid_focal)
            # Record THIS tick's pair (state before stepping):
            obs_thermal.append(obs["thermal"].copy())
            obs_age.append(obs["age"].copy())
            obs_vec.append(obs["vec"].copy())
            actions.append(a.copy())
            ep_id.append(ep)
            drone_slot_id.append(int(env._my_drone_idx))
            t_in_ep.append(float(env._t))

            obs, _r, term, trunc, _info = env.step(a)
            done = term or trunc

        if verbose and (ep + 1) % 10 == 0:
            print(f"  episode {ep+1}/{n_episodes} — running total {len(actions)} samples")

    data = {
        "thermal": np.stack(obs_thermal, axis=0).astype(np.float32),
        "age": np.stack(obs_age, axis=0).astype(np.float32),
        "vec": np.stack(obs_vec, axis=0).astype(np.float32),
        "action": np.stack(actions, axis=0).astype(np.float32),
        "ep_id": np.array(ep_id, dtype=np.int32),
        "drone_slot_id": np.array(drone_slot_id, dtype=np.int32),
        "t_in_ep": np.array(t_in_ep, dtype=np.float32),
        "expert_name": np.array(["sweep_full"]),
        "optimizer_type": np.array([optimizer_type]),
    }
    return data


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--n-episodes", type=int, default=100,
                   help="number of episodes (each = 360s × 5Hz = 1800 transitions)")
    p.add_argument("--optimizer", choices=["feddcsa", "lagrangian"], default="feddcsa",
                   help="which optimizer to run during BC data collection. Use feddcsa "
                        "since that's the actual deployment mode; the policy learns "
                        "what to do under normal (in-budget) leashes.")
    p.add_argument("--arena-yaml", default=None,
                   help="override arena yaml (e.g. K=180 override)")
    p.add_argument("--episode-s", type=float, default=360.0)
    p.add_argument("--out", default="exp1/rl_training/bc_data/bc_sweep_full.npz")
    args = p.parse_args()

    print(f"== collect_bc_dataset: n_ep={args.n_episodes} optimizer={args.optimizer} ==")
    data = collect(
        n_episodes=args.n_episodes,
        optimizer_type=args.optimizer,
        arena_yaml=args.arena_yaml,
        episode_seconds=args.episode_s,
    )
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out_path, **data)
    print(f"\n  N samples: {len(data['action'])}")
    print(f"  obs.thermal shape: {data['thermal'].shape}")
    print(f"  obs.age shape:     {data['age'].shape}")
    print(f"  obs.vec shape:     {data['vec'].shape}")
    print(f"  action shape:      {data['action'].shape}")
    print(f"  saved → {out_path}")


if __name__ == "__main__":
    main()
