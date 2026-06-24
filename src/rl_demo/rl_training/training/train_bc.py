"""Behavior-cloning trainer for iter12.

Loads the BC dataset (obs, action) pairs produced by collect_bc_dataset.py
and fits a CNN+MLP to map obs → action. Uses the SAME architecture as the
PPO feature extractor (CoverageFeatureExtractor in feature_extractor.py)
plus a small 2-layer MLP head matching SB3 ActorCriticPolicy's MlpExtractor.

Output: a state_dict checkpoint that train_ppo.py can warm-start from.

Usage:
    PYTHONPATH=src/rl_demo:src:src/thermal_mapping:src/fed_dcsa \\
        python3 src/rl_demo/rl_training/training/train_bc.py \\
            --data exp1/rl_training/bc_data/bc_sweep_full.npz \\
            --out exp1/rl_training/checkpoints/bc_pretrained_iter12.pt
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset, random_split

_THIS = Path(__file__).resolve()
_RL_DEMO = _THIS.parents[2]
_WS_SRC = _THIS.parents[3]
for sub in (_RL_DEMO, _WS_SRC, _WS_SRC / "thermal_mapping", _WS_SRC / "fed_dcsa"):
    if str(sub) not in sys.path:
        sys.path.insert(0, str(sub))

# Build a stand-in observation_space so the feature extractor inits.
import gymnasium as gym
from rl_training.training.feature_extractor import CoverageFeatureExtractor


def make_obs_space(thermal_shape=(16, 16), age_shape=(32, 32), vec_dim=28):
    return gym.spaces.Dict({
        "thermal": gym.spaces.Box(-50.0, 200.0, shape=thermal_shape, dtype=np.float32),
        "age": gym.spaces.Box(0.0, 360.0, shape=age_shape, dtype=np.float32),
        "vec": gym.spaces.Box(-10.0, 1000.0, shape=(vec_dim,), dtype=np.float32),
    })


class BCPolicy(nn.Module):
    """CoverageFeatureExtractor + 2-layer MLP head matching SB3's default
    MlpExtractor (net_arch=[64, 64] by default for SB3 PPO).
    Outputs 2 floats = (theta_logit, r_frac_logit) before tanh/scaling.
    For BC we use plain (theta, r_fraction) targets and the policy outputs
    them directly via the same architecture; no tanh squash for BC (the
    expert actions are already in [-1, 1.03] domain). PPO will add its own
    Gaussian action head later — we only transfer the feature extractor
    weights + the trunk MLP weights, NOT the action distribution head.
    """

    def __init__(self, obs_space):
        super().__init__()
        self.features = CoverageFeatureExtractor(obs_space)
        # MlpExtractor trunk: must match train_ppo.py's net_arch (= [128, 128])
        # AND activation (= ReLU per ppo_config.yaml → policy_kwargs). Tanh
        # was wrong — kept iter11's BC weights from transferring cleanly.
        # Now: Linear(features_dim, 128) → ReLU → Linear(128, 128) → ReLU
        self.trunk = nn.Sequential(
            nn.Linear(self.features.features_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
        )
        # Action mean head: matches SB3 PPO action_net = Linear(128, 2)
        self.action_head = nn.Linear(128, 2)

    def forward(self, obs: dict) -> torch.Tensor:
        f = self.features(obs)
        h = self.trunk(f)
        return self.action_head(h)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data", default="exp1/rl_training/bc_data/bc_sweep_full.npz")
    p.add_argument("--out", default="exp1/rl_training/checkpoints/bc_pretrained_iter12.pt")
    p.add_argument("--epochs", type=int, default=10)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--val-frac", type=float, default=0.1)
    p.add_argument("--device", default="cpu")
    args = p.parse_args()

    print(f"== train_bc: data={args.data} epochs={args.epochs} device={args.device} ==")
    npz = np.load(args.data, allow_pickle=True)
    thermal = torch.from_numpy(npz["thermal"]).float()  # (N, 16, 16)
    age = torch.from_numpy(npz["age"]).float()           # (N, 32, 32)
    vec = torch.from_numpy(npz["vec"]).float()           # (N, 28)
    action = torch.from_numpy(npz["action"]).float()     # (N, 2)
    N = thermal.shape[0]
    print(f"  N samples: {N}")
    print(f"  action stats: theta∈[{action[:,0].min():.3f}, {action[:,0].max():.3f}]"
          f"  r_frac∈[{action[:,1].min():.3f}, {action[:,1].max():.3f}]")

    # Build dataset; split train/val.
    ds = TensorDataset(thermal, age, vec, action)
    n_val = int(N * args.val_frac)
    n_train = N - n_val
    train_ds, val_ds = random_split(ds, [n_train, n_val],
                                    generator=torch.Generator().manual_seed(42))
    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True, num_workers=0)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False, num_workers=0)

    # Build policy.
    obs_space = make_obs_space(thermal.shape[1:], age.shape[1:], vec.shape[1])
    policy = BCPolicy(obs_space).to(args.device)
    n_params = sum(p.numel() for p in policy.parameters())
    print(f"  policy params: {n_params:,}")

    opt = torch.optim.Adam(policy.parameters(), lr=args.lr)
    loss_fn = nn.MSELoss()

    for epoch in range(args.epochs):
        t0 = time.time()
        policy.train()
        train_loss = 0.0
        train_n = 0
        for batch in train_loader:
            t_, a_, v_, y = [b.to(args.device) for b in batch]
            obs = {"thermal": t_, "age": a_, "vec": v_}
            pred = policy(obs)
            loss = loss_fn(pred, y)
            opt.zero_grad()
            loss.backward()
            opt.step()
            train_loss += loss.item() * t_.shape[0]
            train_n += t_.shape[0]
        train_loss /= train_n

        policy.eval()
        val_loss = 0.0
        val_n = 0
        per_dim_err = torch.zeros(2)
        with torch.no_grad():
            for batch in val_loader:
                t_, a_, v_, y = [b.to(args.device) for b in batch]
                obs = {"thermal": t_, "age": a_, "vec": v_}
                pred = policy(obs)
                val_loss += loss_fn(pred, y).item() * t_.shape[0]
                val_n += t_.shape[0]
                per_dim_err += ((pred - y) ** 2).sum(dim=0).cpu()
        val_loss /= val_n
        rmse_theta = math.sqrt(per_dim_err[0].item() / val_n)
        rmse_rfrac = math.sqrt(per_dim_err[1].item() / val_n)

        dt = time.time() - t0
        print(f"  epoch {epoch+1}/{args.epochs}  "
              f"train_loss={train_loss:.5f}  val_loss={val_loss:.5f}  "
              f"rmse(theta)={rmse_theta:.4f}  rmse(r_frac)={rmse_rfrac:.4f}  "
              f"({dt:.1f}s)")

    # Save state_dict (full BCPolicy) — the train_ppo script picks the relevant
    # sub-keys when warm-starting an SB3 ActorCriticPolicy.
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save({
        "bc_policy_state_dict": policy.state_dict(),
        "n_params": n_params,
        "obs_space_repr": str(obs_space),
        "action_stats": {
            "theta_min": float(action[:, 0].min()),
            "theta_max": float(action[:, 0].max()),
            "r_frac_min": float(action[:, 1].min()),
            "r_frac_max": float(action[:, 1].max()),
        },
        "final_val_loss": val_loss,
        "final_rmse_theta": rmse_theta,
        "final_rmse_rfrac": rmse_rfrac,
        "trained_on_dataset": args.data,
    }, out_path)
    print(f"\n  saved → {out_path}")
    print(f"  final val RMSE: theta={rmse_theta:.4f}, r_frac={rmse_rfrac:.4f}")
    print(f"  (sweep_full expert outputs theta∈[-1,1], r_frac=0.99 const;")
    print(f"   target val_loss ~ 0.001 if BC has learned the pattern.)")


if __name__ == "__main__":
    main()
