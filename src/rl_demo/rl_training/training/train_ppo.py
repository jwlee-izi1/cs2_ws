"""Main PPO training script for the coverage RL policy.

Usage:
    PYTHONPATH=src/rl_demo:src:src/thermal_mapping:src/fed_dcsa \\
        python3 src/rl_demo/rl_training/training/train_ppo.py \\
            --config src/rl_demo/rl_training/configs/ppo_config.yaml \\
            [--total-steps 100000] [--n-envs 8] [--dry-run]

`--dry-run` builds everything (env, network, PPO) but does NOT call .learn().
Useful for Phase 3 plumbing verification (BREAKPOINT 3).
"""

from __future__ import annotations

import argparse
import os
import sys
from functools import partial
from pathlib import Path

import yaml

_THIS = Path(__file__).resolve()
_RL_DEMO = _THIS.parents[2]
_WS_SRC = _THIS.parents[3]
for sub in (_RL_DEMO, _WS_SRC, _WS_SRC / "thermal_mapping", _WS_SRC / "fed_dcsa"):
    if str(sub) not in sys.path:
        sys.path.insert(0, str(sub))

import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor, VecNormalize

from rl_training.env import CoverageEnv
from rl_training.env.rewards import RewardWeights
from rl_training.training.callbacks import (
    RolloutPlotCallback,
    ThroughputLoggerCallback,
)
from rl_training.training.feature_extractor import CoverageFeatureExtractor


def linear_schedule(initial_value: float):
    """Linear LR schedule: progress_remaining (1 at start -> 0 at end) maps to
    progress_remaining * initial_value, i.e. the LR anneals linearly to 0.
    Standard PPO practice (Schulman et al. 2017 Table 5 anneals to 0; SB3 RL Zoo
    `lin_` prefix; SB3 docs `linear_schedule`). iter16: kills the late-training drift
    that made iter15's reward curve decline after its peak."""
    initial_value = float(initial_value)

    def _f(progress_remaining: float) -> float:
        return float(progress_remaining) * initial_value

    return _f


def _resolve_yaml_path(path_str: str) -> str:
    """Treat paths starting with `src/` as relative to the cs2_ws root."""
    p = Path(path_str)
    if p.is_absolute() and p.exists():
        return str(p)
    ws_root = _WS_SRC.parent   # cs2_ws/
    candidate = ws_root / p
    if candidate.exists():
        return str(candidate)
    # Fall back to the default in the env module.
    return path_str


def _build_env_factory(env_cfg: dict, reward_weights_cfg: dict, base_seed: int):
    weights = RewardWeights(**reward_weights_cfg)
    arena_yaml = _resolve_yaml_path(env_cfg["arena_yaml"])
    thermal_yaml = _resolve_yaml_path(env_cfg["thermal_yaml"])

    def _make(rank: int):
        env = CoverageEnv(
            arena_yaml=arena_yaml,
            thermal_yaml=thermal_yaml,
            episode_seconds=env_cfg["episode_seconds"],
            tick_hz=env_cfg["tick_hz"],
            action_slack=env_cfg["action_slack"],
            past_action_history=env_cfg["past_action_history"],
            age_crop_size=env_cfg["age_crop_size"],
            reward_candidate=env_cfg["reward_candidate"],
            reward_weights=weights,
            randomize_sectors=env_cfg["randomize_sectors"],
            max_velocity_override=env_cfg.get("max_velocity", 1.0),
            r_fraction_min=env_cfg.get("r_fraction_min", 0.0),
            dynamics_model=env_cfg.get("dynamics_model", "cascaded_pid"),
            randomize_dynamics=env_cfg.get("randomize_dynamics", True),
            seed=base_seed + rank,
        )
        env.reset(seed=base_seed + rank)
        return env

    return _make


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default=str(_RL_DEMO / "rl_training" / "configs" / "ppo_config.yaml"))
    parser.add_argument("--total-steps", type=int, default=None,
                        help="Override total_timesteps in config.")
    parser.add_argument("--n-envs", type=int, default=None,
                        help="Override n_envs in config.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Build everything but skip .learn(); for Phase 3 verification.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", type=str, default="auto", choices=["auto", "cpu", "cuda"])
    parser.add_argument("--ckpt-suffix", type=str, default="",
                        help="Suffix appended to checkpoint filename (e.g. 'opt1' → ppo_coverage_opt1_final.zip)")
    parser.add_argument("--init-from", type=str, default="",
                        help="iter12: path to BC-pretrained checkpoint produced by train_bc.py. "
                             "Loads the feature extractor + MLP trunk weights into the PPO policy "
                             "before training, so PPO starts from sweep_full-like behavior.")
    parser.add_argument("--reward-overrides", type=str, default="",
                        help="Comma-separated key=value overrides for reward_weights, e.g. 'w_anchor=30,edge_target=0.99'")
    args = parser.parse_args()

    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)

    n_envs = int(args.n_envs if args.n_envs is not None else cfg["ppo"]["n_envs"])
    total_steps = int(args.total_steps if args.total_steps is not None else cfg["ppo"]["total_timesteps"])

    # Apply reward weight overrides from CLI. Keys prefixed "env." apply to env config.
    if args.reward_overrides:
        for kv in args.reward_overrides.split(","):
            kv = kv.strip()
            if "=" not in kv:
                continue
            k, v = kv.split("=", 1)
            k = k.strip()
            v = v.strip()
            try:
                v_cast = float(v)
            except ValueError:
                v_cast = v
            if k.startswith("env."):
                env_key = k[len("env."):]
                cfg["env"][env_key] = v_cast
                print(f"[override] env[{env_key}] = {v_cast}")
            else:
                cfg["reward_weights"][k] = v_cast
                print(f"[override] reward_weights[{k}] = {v_cast}")

    # Build vec env.
    make_one = _build_env_factory(cfg["env"], cfg["reward_weights"], base_seed=args.seed)
    # SB3 expects make_vec_env(env_fn) where env_fn() -> env.
    env_fns = [partial(make_one, rank=i) for i in range(n_envs)]
    if n_envs > 1:
        vec_env = SubprocVecEnv(env_fns)
    else:
        # Use DummyVecEnv for n_envs=1 to keep stack traces readable.
        from stable_baselines3.common.vec_env import DummyVecEnv
        vec_env = DummyVecEnv(env_fns)

    # iter15: VecMonitor on the RAW vec_env so SB3 logs rollout/ep_rew_mean (the
    # TRUE, un-normalized episode reward) → the reward-vs-steps training curve for
    # the paper appendix. This was MISSING → no ep_rew_mean was ever logged.
    # Wrapped BEFORE VecNormalize so it records the true reward, not the scaled one.
    vec_env = VecMonitor(vec_env)

    # Auto-normalize rewards (CRITICAL — raw reward magnitudes are in the
    # thousands, which destabilizes PPO's value loss / KL divergence).
    # Don't normalize obs: our CNN feature extractor already does per-input
    # scaling, and the deployment ROS node won't have access to the
    # normalize stats unless we ship them.
    vec_env = VecNormalize(
        vec_env, norm_obs=False, norm_reward=True,
        clip_reward=10.0, gamma=cfg["ppo"]["gamma"],
    )

    # Set up the policy with our custom feature extractor.
    policy_kwargs = dict(
        features_extractor_class=CoverageFeatureExtractor,
        features_extractor_kwargs=dict(
            thermal_feat=cfg["network"].get("cnn_out_features", 64),
            age_feat=cfg["network"].get("cnn_out_features", 64),
            vec_feat=32,
        ),
        net_arch=dict(
            pi=cfg["network"].get("mlp_hidden", [128, 128]),
            vf=cfg["network"].get("mlp_hidden", [128, 128]),
        ),
        activation_fn=torch.nn.ReLU,
    )

    # Build PPO.
    ppo_cfg = cfg["ppo"]
    tb_dir = str(_WS_SRC.parent / cfg["logging"]["tensorboard_dir"])
    # iter16: linear LR decay (3e-4 -> 0) when lr_schedule=linear (standard; kills the
    # late-training drift). Falls back to a constant LR otherwise.
    if str(ppo_cfg.get("lr_schedule", "constant")).lower() == "linear":
        learning_rate = linear_schedule(ppo_cfg["learning_rate"])
        print(f"  LR schedule: LINEAR anneal {ppo_cfg['learning_rate']} -> 0")
    else:
        learning_rate = float(ppo_cfg["learning_rate"])
    model = PPO(
        policy="MultiInputPolicy",
        env=vec_env,
        learning_rate=learning_rate,
        n_steps=ppo_cfg["n_steps"],
        batch_size=ppo_cfg["batch_size"],
        n_epochs=ppo_cfg["n_epochs"],
        gamma=ppo_cfg["gamma"],
        gae_lambda=ppo_cfg["gae_lambda"],
        clip_range=ppo_cfg["clip_range"],
        ent_coef=ppo_cfg["ent_coef"],
        vf_coef=ppo_cfg["vf_coef"],
        max_grad_norm=ppo_cfg["max_grad_norm"],
        policy_kwargs=policy_kwargs,
        tensorboard_log=tb_dir,
        device=args.device,
        verbose=1,
        seed=args.seed,
    )

    # iter12: BC warm-start. Load the feature extractor + MLP trunk + action
    # head weights from a BC-pretrained checkpoint so PPO starts in the
    # neighborhood of the sweep_full expert. We DO NOT load the value head
    # (BC doesn't train one) — PPO will learn it from scratch, which is fine
    # because the value function adapts fast under good action distributions.
    if args.init_from:
        from pathlib import Path as _P
        bc_path = _P(args.init_from)
        if not bc_path.is_absolute():
            bc_path = _WS_SRC.parent / bc_path
        print(f"\n[iter12 warm-start] loading {bc_path}")
        # Detect file type: BC checkpoint (torch dict with 'bc_policy_state_dict')
        # OR SB3 PPO zip (PPO.load format). Handle both.
        is_ppo_zip = str(bc_path).endswith(".zip")
        if is_ppo_zip:
            from stable_baselines3 import PPO as _PPO
            print(f"  detected SB3 PPO zip — resuming from existing PPO checkpoint")
            src_model = _PPO.load(str(bc_path), device=model.device)
            src_sd = src_model.policy.state_dict()
            ppo_sd = model.policy.state_dict()
            loaded, skipped = [], []
            for k, v in src_sd.items():
                if k in ppo_sd and ppo_sd[k].shape == v.shape:
                    ppo_sd[k] = v
                    loaded.append(k)
                else:
                    skipped.append(k)
            model.policy.load_state_dict(ppo_sd)
            print(f"  loaded {len(loaded)} PPO tensors, skipped {len(skipped)}")
            if skipped:
                for s in skipped[:5]:
                    print(f"    SKIP: {s}")
            del src_model
            # Skip the BC translation block below — we're done.
            bc_ckpt = None
            bc_sd = {}
        else:
            bc_ckpt = torch.load(str(bc_path), map_location=model.device, weights_only=False)
            bc_sd = bc_ckpt["bc_policy_state_dict"]
        # SB3 ActorCriticPolicy state_dict keys we want to map:
        #   features_extractor.thermal_cnn.*, age_cnn.*, vec_mlp.*  ← from BC features.*
        #   mlp_extractor.policy_net.*                              ← from BC trunk
        #   action_net.weight, action_net.bias                      ← from BC action_head
        ppo_sd = model.policy.state_dict()
        loaded, skipped = [], []
        for bc_k, bc_v in bc_sd.items():
            # BC key → PPO key translation.
            if bc_k.startswith("features."):
                ppo_k = "features_extractor." + bc_k[len("features."):]
            elif bc_k.startswith("trunk."):
                ppo_k = "mlp_extractor.policy_net." + bc_k[len("trunk."):]
            elif bc_k.startswith("action_head."):
                ppo_k = "action_net." + bc_k[len("action_head."):]
            else:
                skipped.append(bc_k); continue
            if ppo_k in ppo_sd and ppo_sd[ppo_k].shape == bc_v.shape:
                ppo_sd[ppo_k] = bc_v
                loaded.append(f"{bc_k} → {ppo_k}")
            else:
                skipped.append(f"{bc_k} → {ppo_k} (no match or shape mismatch)")
        model.policy.load_state_dict(ppo_sd)
        print(f"  loaded {len(loaded)} tensors, skipped {len(skipped)}")
        for s in skipped:
            print(f"    SKIP: {s}")

    # Summary printout for Phase 3 dry-run check.
    print("=" * 70)
    print(f"PPO config — total_steps={total_steps}, n_envs={n_envs}")
    print(f"  n_steps_per_env={ppo_cfg['n_steps']}  →  rollout_buffer={n_envs * ppo_cfg['n_steps']} tuples")
    print(f"  batch_size={ppo_cfg['batch_size']}  n_epochs={ppo_cfg['n_epochs']}")
    print(f"  learning_rate={ppo_cfg['learning_rate']}  γ={ppo_cfg['gamma']}  λ={ppo_cfg['gae_lambda']}")
    print(f"  device={model.device}  tensorboard_log={tb_dir}")
    print("=" * 70)
    print(f"Policy network: {model.policy}")
    print("=" * 70)
    # Probe the env once to confirm shapes.
    obs = vec_env.reset()
    for k, v in obs.items():
        print(f"  obs[{k}] shape={v.shape} dtype={v.dtype}")
    print("=" * 70)
    n_params = sum(p.numel() for p in model.policy.parameters() if p.requires_grad)
    print(f"Trainable params: {n_params:,}")
    print("=" * 70)

    if args.dry_run:
        print("\n[--dry-run] Skipping model.learn(). Phase 3 plumbing OK.")
        vec_env.close()
        return

    # Callbacks.
    ckpt_dir = _WS_SRC.parent / cfg["logging"]["checkpoint_dir"]
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    plot_dir = _WS_SRC.parent / "exp1" / "rl_training" / "rollout_plots"
    suffix = f"_{args.ckpt_suffix}" if args.ckpt_suffix else ""
    ckpt_cb = CheckpointCallback(
        save_freq=max(1, int(cfg["logging"]["checkpoint_freq"]) // n_envs),
        save_path=str(ckpt_dir),
        name_prefix=f"ppo_coverage{suffix}",   # iter16: suffix so intermediates are identifiable
    )
    throughput_cb = ThroughputLoggerCallback()
    plot_cb = RolloutPlotCallback(
        make_env_fn=lambda: make_one(rank=999),
        out_dir=plot_dir,
        plot_every_steps=25_000,
        rollout_steps=600,
        verbose=1,
    )

    print(f"\nStarting training: {total_steps:,} total env steps...")
    model.learn(
        total_timesteps=total_steps,
        callback=[ckpt_cb, throughput_cb, plot_cb],
        progress_bar=False,
    )

    final_ckpt = ckpt_dir / f"ppo_coverage{suffix}_final.zip"
    model.save(str(final_ckpt))
    # Also save VecNormalize stats so the eval/deploy side can reproduce
    # the training-time reward normalization if needed.
    vecnorm_path = ckpt_dir / f"vecnormalize{suffix}.pkl"
    vec_env.save(str(vecnorm_path))
    print(f"\nSaved final model to {final_ckpt}")
    print(f"Saved VecNormalize stats to {vecnorm_path}")
    vec_env.close()


if __name__ == "__main__":
    main()
