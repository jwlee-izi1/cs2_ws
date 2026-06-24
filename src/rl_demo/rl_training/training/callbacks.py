"""SB3 callbacks for the coverage RL training.

- ThroughputLoggerCallback: records env steps/sec to TensorBoard.
- RolloutPlotCallback: every N updates, runs the deterministic policy in a
  fresh env and saves a matplotlib summary (r_drone, leash, reward parts).
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback


class ThroughputLoggerCallback(BaseCallback):

    def __init__(self, verbose: int = 0):
        super().__init__(verbose)
        self._wall_start: float | None = None
        self._steps_start: int = 0

    def _on_training_start(self) -> None:
        self._wall_start = time.perf_counter()
        self._steps_start = self.num_timesteps

    def _on_step(self) -> bool:
        # Log throughput every 5000 env steps.
        if self.num_timesteps - self._steps_start >= 5_000:
            elapsed = time.perf_counter() - self._wall_start
            steps = self.num_timesteps - self._steps_start
            rate = steps / max(elapsed, 1e-6)
            self.logger.record("throughput/env_steps_per_sec", rate)
            self._wall_start = time.perf_counter()
            self._steps_start = self.num_timesteps
        return True


class RolloutPlotCallback(BaseCallback):

    def __init__(
        self,
        make_env_fn,
        out_dir: str | Path,
        plot_every_steps: int = 25_000,
        rollout_steps: int = 600,
        verbose: int = 0,
    ):
        super().__init__(verbose)
        self.make_env_fn = make_env_fn
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.plot_every = int(plot_every_steps)
        self.rollout_steps = int(rollout_steps)
        self._next_plot_at = self.plot_every

    def _on_step(self) -> bool:
        if self.num_timesteps < self._next_plot_at:
            return True
        self._next_plot_at = self.num_timesteps + self.plot_every

        try:
            self._save_plot(tag=f"step{self.num_timesteps}")
        except Exception as exc:
            if self.verbose:
                print(f"[RolloutPlot] failed: {exc}")
        return True

    def _save_plot(self, tag: str):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        env = self.make_env_fn()
        obs, info = env.reset(seed=0)
        rewards, r_drones, leashes = [], [], []
        coverages, edges, smooths, leash_pens = [], [], [], []
        for _ in range(self.rollout_steps):
            # SB3 expects batched input; our env returns a single-instance dict.
            obs_batched = {k: np.expand_dims(v, 0) for k, v in obs.items()}
            action, _ = self.model.predict(obs_batched, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action[0])
            rewards.append(reward)
            r_drones.append(info["r_drone"])
            leashes.append(info["r_leash"])
            parts = info.get("reward_parts", {})
            coverages.append(parts.get("coverage", 0.0))
            edges.append(parts.get("edge", 0.0))
            smooths.append(parts.get("smooth", 0.0))
            leash_pens.append(parts.get("leash", 0.0))
            if terminated or truncated:
                break

        t = np.arange(len(rewards)) * 0.2
        fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        axes[0].plot(t, r_drones, label="r_drone")
        axes[0].plot(t, leashes, label="opt_leash")
        axes[0].set_ylabel("radius (m)")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title(f"Rollout @ {tag}")
        axes[1].plot(t, coverages, label="coverage")
        axes[1].plot(t, edges, label="edge")
        axes[1].plot(t, smooths, label="smooth")
        axes[1].plot(t, leash_pens, label="leash_pen")
        axes[1].set_xlabel("sim time (s)")
        axes[1].set_ylabel("reward parts / tick")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        fig.tight_layout()
        plot_path = self.out_dir / f"rollout_{tag}.png"
        fig.savefig(plot_path, dpi=100)
        plt.close(fig)
        if self.verbose:
            print(f"[RolloutPlot] saved {plot_path}")
