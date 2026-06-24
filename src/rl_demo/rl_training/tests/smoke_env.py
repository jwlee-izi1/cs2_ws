"""Phase 1 smoke test: env builds, resets, steps with shape sanity, and a
short rollout that lets the fire spread enough to see non-zero leash + reward.

Run from the workspace root:
    PYTHONPATH=src/rl_demo:src python3 src/rl_demo/rl_training/tests/smoke_env.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_THIS = Path(__file__).resolve()
_RL_DEMO = _THIS.parents[2]
_WS_SRC = _THIS.parents[3]
for p in (str(_RL_DEMO), str(_WS_SRC)):
    if p not in sys.path:
        sys.path.insert(0, p)

from rl_training.env import CoverageEnv


def main():
    env = CoverageEnv()
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")
    obs, info = env.reset(seed=42)
    print(f"\nReset OK. Sector = {info['sector_name']}")
    for key, value in obs.items():
        print(f"  obs[{key}] shape={value.shape} dtype={value.dtype} "
              f"min={float(value.min()):+.3f} max={float(value.max()):+.3f}")

    # Longer rollout — 1500 ticks = 300 s of sim time, well into fire dev.
    n_steps = 1500
    total_reward = 0.0
    progress_ticks = (1, 100, 250, 500, 900, 1500)
    coverage_sum = 0.0
    edge_sum = 0.0
    smooth_sum = 0.0
    leash_sum = 0.0

    # Use a "ride the edge" policy: theta = sweep slowly, r_fraction = 0.99.
    for i in range(n_steps):
        theta = np.sin(i * 0.05) * 0.9
        r_frac = 0.99
        action = np.array([theta, r_frac], dtype=np.float32)
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        parts = info.get("reward_parts", {})
        coverage_sum += parts.get("coverage", 0.0)
        edge_sum += parts.get("edge", 0.0)
        smooth_sum += parts.get("smooth", 0.0)
        leash_sum += parts.get("leash", 0.0)
        if (i + 1) in progress_ticks:
            print(f"  step={i+1:4d} t={info['t']:6.2f}s "
                  f"r_drone={info['r_drone']:.3f} leash={info['r_leash']:.3f} "
                  f"reward={reward:+7.4f}")

    print(f"\n{n_steps} steps total_reward={total_reward:+.2f}")
    print(f"  cum coverage: {coverage_sum:+.2f}")
    print(f"  cum edge    : {edge_sum:+.2f}")
    print(f"  cum smooth  : {smooth_sum:+.2f}")
    print(f"  cum leash   : {leash_sum:+.2f}")
    if any(np.isnan(v) for v in (total_reward, coverage_sum, edge_sum)):
        print("PHASE 1 SMOKE: FAIL — NaN detected")
        return
    print("PHASE 1 SMOKE: PASS")


if __name__ == "__main__":
    main()
