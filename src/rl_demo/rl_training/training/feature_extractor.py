"""Custom CNN+MLP feature extractor for the coverage RL policy.

The env's observation is a Dict:
    thermal: (16, 16)  — current sensor reading
    age:     (32, 32)  — sector-wide staleness crop
    vec:     (28,)     — own leash, pose, velocity, sector geom, past actions, time

Architecture (matches plan §Q3):
    [thermal 16×16] ─► CNN ─► flatten ─► feat_thermal (64)
    [age 32×32]     ─► CNN ─► flatten ─► feat_age     (64)
    [vec 28]        ─► MLP ─► feat_vec  (32)
    concat(64 + 64 + 32) = 160 → output features for PPO actor + critic heads

SB3 then puts its own (actor, critic) heads on top via the policy_kwargs.
"""

from __future__ import annotations

import gymnasium as gym
import torch
import torch.nn as nn
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


class CoverageFeatureExtractor(BaseFeaturesExtractor):

    def __init__(
        self,
        observation_space: gym.spaces.Dict,
        thermal_feat: int = 64,
        age_feat: int = 64,
        vec_feat: int = 32,
    ):
        # We must call super().__init__ with the TOTAL output dim.
        total_features = thermal_feat + age_feat + vec_feat
        super().__init__(observation_space, features_dim=total_features)

        # Thermal sensor branch: 16×16 → small CNN → MLP.
        self.thermal_cnn = nn.Sequential(
            nn.Conv2d(1, 16, kernel_size=3, padding=1),    # → 16×16×16
            nn.ReLU(),
            nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1),   # → 8×8×32
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(8 * 8 * 32, thermal_feat),
            nn.ReLU(),
        )

        # Age map branch: 32×32 → deeper CNN → MLP.
        self.age_cnn = nn.Sequential(
            nn.Conv2d(1, 16, kernel_size=3, padding=1),   # → 32×32×16
            nn.ReLU(),
            nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1),   # → 16×16×32
            nn.ReLU(),
            nn.Conv2d(32, 32, kernel_size=3, stride=2, padding=1),   # → 8×8×32
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(8 * 8 * 32, age_feat),
            nn.ReLU(),
        )

        # Vector branch: 28 scalars → 2-layer MLP.
        vec_dim = observation_space["vec"].shape[0]
        self.vec_mlp = nn.Sequential(
            nn.Linear(vec_dim, 64),
            nn.ReLU(),
            nn.Linear(64, vec_feat),
            nn.ReLU(),
        )

    def forward(self, observations: dict[str, torch.Tensor]) -> torch.Tensor:
        # Each input arrives as a batched tensor from SB3.
        thermal = observations["thermal"]   # (B, 16, 16)
        age = observations["age"]           # (B, 32, 32)
        vec = observations["vec"]           # (B, 28)

        # Normalize spatial inputs (centered & scaled), add channel dim.
        # thermal is in °C (typically 20-100°C); normalize relative to ambient.
        thermal_n = (thermal - 22.0) / 60.0     # → roughly [0, 1.3]
        age_n = age / 60.0                       # max_age_explore=60 → [0, 6]
        thermal_n = thermal_n.unsqueeze(1)      # (B, 1, 16, 16)
        age_n = age_n.unsqueeze(1)              # (B, 1, 32, 32)

        f_thermal = self.thermal_cnn(thermal_n)
        f_age = self.age_cnn(age_n)
        f_vec = self.vec_mlp(vec)

        return torch.cat([f_thermal, f_age, f_vec], dim=1)
