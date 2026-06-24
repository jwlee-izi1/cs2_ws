"""Evaluate trained policy vs random baseline.

Runs N episodes of each, records behavioral metrics, prints a comparison
table. If the trained policy isn't measurably better than random on the
intended-behavior metrics, the reward shape needs more work.

Usage:
    PYTHONPATH=src/rl_demo:src:src/thermal_mapping:src/fed_dcsa \\
        python3 src/rl_demo/rl_training/eval/eval_policy.py \\
            [--checkpoint exp1/rl_training/checkpoints/ppo_coverage_final.zip] \\
            [--episodes 3]
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

from stable_baselines3 import PPO

from rl_training.env import CoverageEnv


def _run_episode(env, policy_fn, label=""):
    """Run one full episode; return per-tick metrics PLUS area / hot-cell coverage."""
    obs, info = env.reset()
    r_drones, leashes_opt, leashes_eff, actions, rewards = [], [], [], [], []
    coverages, edges, smooths = [], [], []
    # Per-tick joint constraint Σ c_i r_drone² across all 4 drones (key demo metric).
    # Both the optimizer's leash sum AND the drones' actual position sum.
    joint_sum_drone_per_tick = []   # Σ c_i r_drone²
    joint_sum_leash_per_tick = []   # Σ c_i (r_i^k)²
    ts_per_tick = []
    # Track which cells in the sector were observed at any point, and which
    # cells reached "hot" status (T > ambient + 30) at any point.
    sector_cells_observed = set()
    hot_cells_ever = set()
    hot_cells_observed = set()
    # Per-tick "energy density in FOV" — fraction of FOV cells that are hot.
    fov_hot_fraction_per_tick = []

    # Tracking-accuracy machinery (the metric you asked for):
    # drone_belief = cell_idx -> (observed_temp, last_obs_time).
    # Updated only on ticks where Σ_drone <= B_op (binary packet loss).
    # At each metric-check tick, we compare drone's belief vs ground truth
    # at the CURRENT time, using a staleness window — old observations are
    # treated as "unknown" (ambient) since they're outdated.
    B_op_tracking = 8.15
    staleness_window_s = 30.0  # observations older than this are stale → ambient
    drone_belief = {}              # WITH packet loss: only kept ticks update this
    drone_belief_ideal = {}        # WITHOUT packet loss: ALL ticks update this
    tracking_accuracy_samples = []   # per metric-check tick
    dT_error_samples = []            # per metric-check tick
    # DELTA-FROM-IDEAL: cells drone "would have known are hot" but doesn't due to loss.
    # FedDCSA → 0 (no loss → drone_belief == drone_belief_ideal).
    # Lagrangian → spikes during 60s violation window.
    damage_cells_per_tick = []
    damage_window_samples = []   # per tick (within 60s window only)
    # Fire spread metric: per-sector wavefront radius (drone belief vs truth).
    # |truth_r - belief_r| at each tick, averaged over sectors.
    wavefront_radius_error_samples = []  # mean error in meters per tick
    wavefront_radius_truth_samples = []  # actual truth wavefront radius for context
    wavefront_radius_belief_samples = []  # drone belief wavefront radius
    iou_samples = []                     # IoU of truth-hot vs belief-hot 2D maps
    # F1 score for "actual fire point" detection — stricter threshold (T_amb+60°C)
    # means we only count cells in main burn phase (not just warmed-up cells).
    FIRE_THRESHOLD_C = 78.0              # delta-T above ambient (T>100°C — near peak only)
    f1_samples = []                      # per metric-check tick: F1 score
    precision_samples = []               # per tick: precision
    recall_samples = []                  # per tick: recall
    while True:
        action = policy_fn(obs)
        obs, reward, terminated, truncated, info = env.step(action)
        r_drones.append(info["r_drone"])
        leashes_opt.append(info["r_leash"])
        leashes_eff.append(info["r_leash_effective"])
        actions.append(action.copy())
        rewards.append(reward)
        parts = info.get("reward_parts", {})
        coverages.append(parts.get("coverage", 0.0))
        edges.append(parts.get("edge", 0.0))
        smooths.append(parts.get("smooth", 0.0))

        # Track which sector cells the drone's sensor footprint just covered.
        my_idx = env._my_drone_idx
        drone = env._drones[my_idx]
        sensor = env._sensors[my_idx]
        xs = drone.position[0] + sensor._gx_h
        ys = drone.position[1] + sensor._gy_h
        cell_x = (xs / env._map.resolution).astype(np.int32)
        cell_y = (ys / env._map.resolution).astype(np.int32)
        for cx, cy in zip(cell_x.flatten(), cell_y.flatten()):
            sector_cells_observed.add((int(cx), int(cy)))

        # Energy density: fraction of FOV cells that are hot RIGHT NOW.
        truth_fov = env._field.evaluate(xs, ys, env._t)
        hot_in_fov = (truth_fov > 22.0 + 30.0).sum()
        fov_hot_fraction_per_tick.append(float(hot_in_fov) / truth_fov.size)

        # JOINT CONSTRAINT TRACES: Σ c_i r² across all 4 drones (the demo metric).
        # Both physical (drone position) and optimizer (leash) versions.
        drones_all = env._drones
        leashes_all = env._opt.leashes
        cs = [env._cfg["sectors"][env._sector_assignment[i]]["c"]
              for i in range(len(drones_all))]
        joint_drone = sum(c * (d.position[0] ** 2 + d.position[1] ** 2)
                          for c, d in zip(cs, drones_all))
        joint_leash = sum(c * (lk ** 2) for c, lk in zip(cs, leashes_all))
        joint_sum_drone_per_tick.append(float(joint_drone))
        joint_sum_leash_per_tick.append(float(joint_leash))
        ts_per_tick.append(float(env._t))

        # TRACKING-ACCURACY UPDATE.
        # IDEAL belief: always updated (no loss).
        # ACTUAL belief: only updated if tick NOT lost (Σ_drone <= B_op).
        tick_kept = (joint_drone <= B_op_tracking)
        for slot_idx, drone_k in enumerate(env._drones):
            sensor_k = env._sensors[slot_idx]
            xs_k = drone_k.position[0] + sensor_k._gx_h
            ys_k = drone_k.position[1] + sensor_k._gy_h
            truth_fov_k = env._field.evaluate(xs_k, ys_k, env._t)
            for ii in range(truth_fov_k.shape[0]):
                for jj in range(truth_fov_k.shape[1]):
                    cx = int(xs_k[ii, jj] / env._map.resolution)
                    cy = int(ys_k[ii, jj] / env._map.resolution)
                    # Ideal belief: always update
                    drone_belief_ideal[(cx, cy)] = (float(truth_fov_k[ii, jj]), float(env._t))
                    if tick_kept:
                        drone_belief[(cx, cy)] = (float(truth_fov_k[ii, jj]), float(env._t))

        # Compute per-tick tracking accuracy + dT error (every 5 ticks ≈ 1s)
        # SYSTEM-LEVEL: sample over the whole arena disc (all 4 sectors), not
        # just our drone's sector. This is the joint-system tracking accuracy.
        if len(rewards) % 5 == 0:
            r_max_chk = 1.9
            rs = np.linspace(0.1, r_max_chk, 40, dtype=np.float32)
            ps = np.linspace(-math.pi, math.pi, 60, dtype=np.float32)
            rr_a, pp_a = np.meshgrid(rs, ps, indexing="ij")
            xs_a = rr_a * np.cos(pp_a)
            ys_a = rr_a * np.sin(pp_a)
            truth_t = env._field.evaluate(xs_a, ys_a, env._t)
            T_amb = 22.0
            truth_hot_mask = truth_t > (T_amb + 30.0)
            # Build per-cell belief at THIS time (ambient if no fresh obs).
            cx_a = (xs_a / env._map.resolution).astype(np.int32)
            cy_a = (ys_a / env._map.resolution).astype(np.int32)
            belief_t = np.full(truth_t.shape, T_amb, dtype=np.float32)
            for ii in range(truth_t.shape[0]):
                for jj in range(truth_t.shape[1]):
                    cell = (int(cx_a[ii, jj]), int(cy_a[ii, jj]))
                    if cell in drone_belief:
                        temp, t_obs = drone_belief[cell]
                        if env._t - t_obs <= staleness_window_s:
                            belief_t[ii, jj] = temp
            # rebuild shape for new grid (40 r × 60 phi)
            truth_hot_mask = truth_t > (T_amb + 30.0)
            belief_hot_mask = belief_t > (T_amb + 30.0)
            n_truth_hot = int(truth_hot_mask.sum())
            if n_truth_hot > 0:
                n_inter = int((truth_hot_mask & belief_hot_mask).sum())
                tracking_accuracy_samples.append(float(n_inter) / float(n_truth_hot))
                dT_err = float(np.abs(truth_t[truth_hot_mask] - belief_t[truth_hot_mask]).mean())
                dT_error_samples.append(dT_err)

            # FIRE SPREAD METRIC — per-sector wavefront radius (truth vs belief).
            # Use the SAME polar grid already sampled. For each of the 4 sectors,
            # find max r where truth/belief is hot. Average |error| over sectors.
            sector_errors = []
            sector_truth_rs = []
            sector_belief_rs = []
            for s_idx in range(len(env._cfg["sectors"])):
                s_cfg = env._cfg["sectors"][s_idx]
                s_phi_mid = math.radians(s_cfg["phi_mid_deg"])
                s_phi_half = math.radians(s_cfg["phi_half_deg"])
                # rr_a is the r grid, pp_a the phi grid
                phi_diff = np.mod(pp_a - s_phi_mid + np.pi, 2 * np.pi) - np.pi
                in_sector_mask = np.abs(phi_diff) <= s_phi_half
                truth_in_sec_hot = truth_hot_mask & in_sector_mask
                belief_in_sec_hot = belief_hot_mask & in_sector_mask
                truth_r_max = float(rr_a[truth_in_sec_hot].max()) if truth_in_sec_hot.any() else 0.0
                belief_r_max = float(rr_a[belief_in_sec_hot].max()) if belief_in_sec_hot.any() else 0.0
                sector_errors.append(abs(truth_r_max - belief_r_max))
                sector_truth_rs.append(truth_r_max)
                sector_belief_rs.append(belief_r_max)
            wavefront_radius_error_samples.append(float(np.mean(sector_errors)))
            wavefront_radius_truth_samples.append(float(np.mean(sector_truth_rs)))
            wavefront_radius_belief_samples.append(float(np.mean(sector_belief_rs)))

            # IoU on 2D hot-cell sets (full arena, not just one sector).
            intersection = (truth_hot_mask & belief_hot_mask).sum()
            union = (truth_hot_mask | belief_hot_mask).sum()
            iou = float(intersection) / float(union) if union > 0 else 0.0
            iou_samples.append(iou)

            # F1 score with STRICTER "fire point" threshold (T_amb + 60°C).
            # Captures whether drone correctly identifies actual fire points,
            # not just warm cells.
            truth_fire_mask = truth_t > (T_amb + FIRE_THRESHOLD_C)
            belief_fire_mask = belief_t > (T_amb + FIRE_THRESHOLD_C)
            n_inter_fire = int((truth_fire_mask & belief_fire_mask).sum())
            n_belief_fire = int(belief_fire_mask.sum())
            n_truth_fire = int(truth_fire_mask.sum())
            precision_t = (n_inter_fire / n_belief_fire) if n_belief_fire > 0 else 1.0
            recall_t = (n_inter_fire / n_truth_fire) if n_truth_fire > 0 else None
            if recall_t is not None:
                # F1 only meaningful when there's actual fire to detect
                if precision_t + recall_t > 0:
                    f1 = 2.0 * precision_t * recall_t / (precision_t + recall_t)
                else:
                    f1 = 0.0
                f1_samples.append(f1)
                precision_samples.append(precision_t)
                recall_samples.append(recall_t)

            # DAMAGE METRIC: cells the drone would have known were hot
            # (ideal) but doesn't (actual) due to packet loss.
            belief_ideal_t = np.full(truth_t.shape, T_amb, dtype=np.float32)
            for ii in range(truth_t.shape[0]):
                for jj in range(truth_t.shape[1]):
                    cell = (int(cx_a[ii, jj]), int(cy_a[ii, jj]))
                    if cell in drone_belief_ideal:
                        temp_i, t_obs_i = drone_belief_ideal[cell]
                        if env._t - t_obs_i <= staleness_window_s:
                            belief_ideal_t[ii, jj] = temp_i
            belief_ideal_hot = belief_ideal_t > (T_amb + FIRE_THRESHOLD_C)
            actual_hot = belief_fire_mask
            damage_mask = belief_ideal_hot & ~actual_hot
            damage = int(damage_mask.sum())
            damage_cells_per_tick.append(damage)
            if 150.0 <= env._t <= 210.0:
                damage_window_samples.append(damage)

        # Also track which cells are currently hot (using truth field).
        # Sample a coarse grid of the whole sector to find hot cells.
        if len(rewards) % 5 == 0:  # every 5 ticks (~1s) to save compute
            sector_cfg = env._cfg["sectors"][env._sector_assignment[my_idx]]
            phi_mid = math.radians(sector_cfg["phi_mid_deg"])
            phi_half = math.radians(sector_cfg["phi_half_deg"])
            r_max = sector_cfg["r_max"]
            r_samples = np.linspace(0.1, r_max, 40)
            phi_samples = np.linspace(phi_mid - phi_half, phi_mid + phi_half, 40)
            rr, pp = np.meshgrid(r_samples, phi_samples, indexing="ij")
            xs_chk = rr * np.cos(pp)
            ys_chk = rr * np.sin(pp)
            temps_chk = env._field.evaluate(xs_chk, ys_chk, env._t)
            hot_mask = temps_chk > 22.0 + 30.0  # T_ambient = 22, threshold = 30°C
            for cx, cy in zip(
                (xs_chk[hot_mask] / env._map.resolution).astype(np.int32).flatten(),
                (ys_chk[hot_mask] / env._map.resolution).astype(np.int32).flatten(),
            ):
                hot_cells_ever.add((int(cx), int(cy)))

        if terminated or truncated:
            break

    # Intersect hot_cells_ever with sector_cells_observed.
    hot_cells_observed = hot_cells_ever & sector_cells_observed

    # Effective hot coverage under data loss: BINARY model — any tick where
    # Σ_drone > B_op loses ALL drone observations in that tick (wireless channel
    # over capacity → packets dropped). Conservative, matches typical hard-cutoff
    # interpretation of channel capacity.
    B_op = 8.15
    n_ticks = len(joint_sum_drone_per_tick)
    keep_flags = [(sd <= B_op) for sd in joint_sum_drone_per_tick]
    n_kept = sum(keep_flags)
    transmit_rate = n_kept / max(n_ticks, 1)
    # Effective hot coverage: cells we'd actually receive at base = cells observed
    # in NON-violating ticks, intersected with ever-hot set.
    # Approximation: each tick contributes uniformly to sector_cells_observed,
    # so effective coverage scales linearly with transmit_rate.
    effective_hot_obs_rate = len(hot_cells_observed) / max(len(hot_cells_ever), 1) * transmit_rate

    return dict(
        r_drone=np.array(r_drones),
        leash_opt=np.array(leashes_opt),
        leash_eff=np.array(leashes_eff),
        action=np.array(actions),
        reward=np.array(rewards),
        coverage=np.array(coverages),
        edge=np.array(edges),
        smooth=np.array(smooths),
        n_cells_observed=len(sector_cells_observed),
        n_hot_cells_ever=len(hot_cells_ever),
        n_hot_cells_observed=len(hot_cells_observed),
        hot_observation_rate=(
            len(hot_cells_observed) / max(len(hot_cells_ever), 1)
        ),
        mean_fov_hot_density=float(np.mean(fov_hot_fraction_per_tick)) if fov_hot_fraction_per_tick else 0.0,
        max_fov_hot_density=float(np.max(fov_hot_fraction_per_tick)) if fov_hot_fraction_per_tick else 0.0,
        joint_sum_drone=np.array(joint_sum_drone_per_tick),
        joint_sum_leash=np.array(joint_sum_leash_per_tick),
        ts=np.array(ts_per_tick),
        transmit_rate=float(transmit_rate),
        effective_hot_obs_rate=float(effective_hot_obs_rate),
        mean_tracking_accuracy=float(np.mean(tracking_accuracy_samples)) if tracking_accuracy_samples else 0.0,
        mean_dT_error=float(np.mean(dT_error_samples)) if dT_error_samples else 0.0,
        mean_wavefront_radius_error=float(np.mean(wavefront_radius_error_samples)) if wavefront_radius_error_samples else 0.0,
        mean_iou_hot=float(np.mean(iou_samples)) if iou_samples else 0.0,
        mean_fire_f1=float(np.mean(f1_samples)) if f1_samples else 0.0,
        mean_fire_precision=float(np.mean(precision_samples)) if precision_samples else 0.0,
        mean_fire_recall=float(np.mean(recall_samples)) if recall_samples else 0.0,
        mean_damage_cells=float(np.mean(damage_cells_per_tick)) if damage_cells_per_tick else 0.0,
        mean_damage_in_window=float(np.mean(damage_window_samples)) if damage_window_samples else 0.0,
        max_damage_cells=float(np.max(damage_cells_per_tick)) if damage_cells_per_tick else 0.0,
    )


def _summarize(per_episode_metrics, label):
    """Compute a single summary row across episodes."""
    edge_util_per_ep = []
    cum_reward_per_ep = []
    cum_coverage_per_ep = []
    cum_edge_per_ep = []
    cum_smooth_per_ep = []
    r_frac_mean_per_ep = []
    r_frac_std_per_ep = []
    theta_std_per_ep = []
    n_cells_per_ep = []
    n_hot_ever_per_ep = []
    n_hot_obs_per_ep = []
    hot_obs_rate_per_ep = []
    mean_fov_hot_density_per_ep = []
    max_fov_hot_density_per_ep = []
    transmit_rate_per_ep = []
    effective_hot_obs_per_ep = []
    mean_tracking_acc_per_ep = []
    mean_dT_error_per_ep = []
    mean_wavefront_radius_error_per_ep = []
    mean_iou_per_ep = []
    mean_fire_f1_per_ep = []
    mean_fire_precision_per_ep = []
    mean_fire_recall_per_ep = []
    mean_damage_per_ep = []
    mean_damage_window_per_ep = []
    max_damage_per_ep = []
    for ep in per_episode_metrics:
        eff = np.maximum(ep["leash_eff"], 1e-6)
        edge_util = (ep["r_drone"] / eff).mean()
        edge_util_per_ep.append(edge_util)
        cum_reward_per_ep.append(ep["reward"].sum())
        cum_coverage_per_ep.append(ep["coverage"].sum())
        cum_edge_per_ep.append(ep["edge"].sum())
        cum_smooth_per_ep.append(ep["smooth"].sum())
        r_frac_mean_per_ep.append(ep["action"][:, 1].mean())
        r_frac_std_per_ep.append(ep["action"][:, 1].std())
        theta_std_per_ep.append(ep["action"][:, 0].std())
        n_cells_per_ep.append(ep.get("n_cells_observed", 0))
        n_hot_ever_per_ep.append(ep.get("n_hot_cells_ever", 0))
        n_hot_obs_per_ep.append(ep.get("n_hot_cells_observed", 0))
        hot_obs_rate_per_ep.append(ep.get("hot_observation_rate", 0.0))
        mean_fov_hot_density_per_ep.append(ep.get("mean_fov_hot_density", 0.0))
        max_fov_hot_density_per_ep.append(ep.get("max_fov_hot_density", 0.0))
        transmit_rate_per_ep.append(ep.get("transmit_rate", 1.0))
        effective_hot_obs_per_ep.append(ep.get("effective_hot_obs_rate", 0.0))
        mean_tracking_acc_per_ep.append(ep.get("mean_tracking_accuracy", 0.0))
        mean_dT_error_per_ep.append(ep.get("mean_dT_error", 0.0))
        mean_wavefront_radius_error_per_ep.append(ep.get("mean_wavefront_radius_error", 0.0))
        mean_iou_per_ep.append(ep.get("mean_iou_hot", 0.0))
        mean_fire_f1_per_ep.append(ep.get("mean_fire_f1", 0.0))
        mean_fire_precision_per_ep.append(ep.get("mean_fire_precision", 0.0))
        mean_fire_recall_per_ep.append(ep.get("mean_fire_recall", 0.0))
        mean_damage_per_ep.append(ep.get("mean_damage_cells", 0.0))
        mean_damage_window_per_ep.append(ep.get("mean_damage_in_window", 0.0))
        max_damage_per_ep.append(ep.get("max_damage_cells", 0.0))
    return dict(
        label=label,
        n_ep=len(per_episode_metrics),
        mean_edge_util=np.mean(edge_util_per_ep),
        mean_cum_reward=np.mean(cum_reward_per_ep),
        mean_cum_coverage=np.mean(cum_coverage_per_ep),
        mean_cum_edge=np.mean(cum_edge_per_ep),
        mean_cum_smooth=np.mean(cum_smooth_per_ep),
        mean_r_frac=np.mean(r_frac_mean_per_ep),
        mean_r_frac_std=np.mean(r_frac_std_per_ep),
        mean_theta_std=np.mean(theta_std_per_ep),
        mean_n_cells_obs=np.mean(n_cells_per_ep),
        mean_n_hot_cells_ever=np.mean(n_hot_ever_per_ep),
        mean_n_hot_cells_obs=np.mean(n_hot_obs_per_ep),
        mean_hot_obs_rate=np.mean(hot_obs_rate_per_ep),
        mean_fov_hot_density=np.mean(mean_fov_hot_density_per_ep),
        max_fov_hot_density=np.mean(max_fov_hot_density_per_ep),
        mean_transmit_rate=np.mean(transmit_rate_per_ep),
        mean_effective_hot_obs_rate=np.mean(effective_hot_obs_per_ep),
        mean_tracking_accuracy=np.mean(mean_tracking_acc_per_ep),
        mean_dT_error=np.mean(mean_dT_error_per_ep),
        mean_wavefront_radius_error=np.mean(mean_wavefront_radius_error_per_ep),
        mean_iou_hot=np.mean(mean_iou_per_ep),
        mean_fire_f1=np.mean(mean_fire_f1_per_ep),
        mean_fire_precision=np.mean(mean_fire_precision_per_ep),
        mean_fire_recall=np.mean(mean_fire_recall_per_ep),
        mean_damage_cells=np.mean(mean_damage_per_ep),
        mean_damage_in_window=np.mean(mean_damage_window_per_ep),
        max_damage_cells=np.mean(max_damage_per_ep),
    )


def _print_table(summaries):
    cols = [
        ("Metric", "label"),
        ("n_ep", "n_ep"),
        ("mean edge util (r/leash)", "mean_edge_util"),
        ("mean cum reward", "mean_cum_reward"),
        ("mean cum coverage", "mean_cum_coverage"),
        ("mean cum edge", "mean_cum_edge"),
        ("mean cum smooth (penalty)", "mean_cum_smooth"),
        ("mean r_fraction", "mean_r_frac"),
        ("r_fraction std (action variability)", "mean_r_frac_std"),
        ("theta std (action variability)", "mean_theta_std"),
        ("mean FOV hot density (NEW)", "mean_fov_hot_density"),
        ("max  FOV hot density (NEW)", "max_fov_hot_density"),
        ("% hot cells observed (raw)", "mean_hot_obs_rate"),
        ("transmit rate (1-loss)", "mean_transmit_rate"),
        ("% hot cells EFFECTIVE (post-loss)", "mean_effective_hot_obs_rate"),
        ("TRACKING accuracy (T>amb+30, raw)", "mean_tracking_accuracy"),
        ("mean dT error (deg C, hot cells)", "mean_dT_error"),
        ("FIRE-POINT F1 (T>amb+60)", "mean_fire_f1"),
        ("  precision (no false positives)", "mean_fire_precision"),
        ("  recall (catch all fire)", "mean_fire_recall"),
        ("wavefront radius error (m)", "mean_wavefront_radius_error"),
        ("IoU on hot region", "mean_iou_hot"),
        ("DAMAGE cells (whole episode)", "mean_damage_cells"),
        ("DAMAGE cells (60s window only)", "mean_damage_in_window"),
        ("DAMAGE peak (max per tick)", "max_damage_cells"),
    ]
    for header, key in cols:
        if key == "label":
            row = "  ".join(f"{s['label']:>14}" for s in summaries)
            print(f"  {header:<38} | {row}")
            print("  " + "-" * (40 + 14 * len(summaries) + 3))
        else:
            vals = []
            for s in summaries:
                v = s[key]
                if isinstance(v, int):
                    vals.append(f"{v:>14d}")
                else:
                    vals.append(f"{v:>14.3f}")
            row = "  ".join(vals)
            print(f"  {header:<38} | {row}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=str,
                        default=str(_WS_SRC.parent / "exp1" / "rl_training" / "checkpoints" / "ppo_coverage_final.zip"))
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--episode-seconds", type=float, default=360.0)
    parser.add_argument("--seed", type=int, default=12345)
    parser.add_argument("--optimizer", type=str, default="feddcsa",
                        choices=["feddcsa", "lagrangian"],
                        help="Which optimizer drives the leashes in the env")
    parser.add_argument("--all-drones-trained", action="store_true",
                        help="Deployment-style eval: all 4 drones run the trained policy "
                             "(not just 1 trained + 3 shadow heuristic). "
                             "Slower per episode but gives REAL Σ_drone numbers.")
    args = parser.parse_args()

    print(f"Loading checkpoint: {args.checkpoint}")
    model = PPO.load(args.checkpoint, device="cpu")

    def trained_policy(obs):
        obs_batched = {k: np.expand_dims(v, 0) for k, v in obs.items()}
        action, _ = model.predict(obs_batched, deterministic=True)
        return action[0]

    rng = np.random.default_rng(args.seed)
    def random_policy(obs):
        # Sample in same action space: theta uniform [-1,1], r_frac uniform [0,1.03]
        return np.array([rng.uniform(-1.0, 1.0), rng.uniform(0.0, 1.03)], dtype=np.float32)

    def hold_centerline_policy(obs):
        # Hand-crafted "ride centerline at 0.99" — what we want the policy to learn
        return np.array([0.0, 0.99], dtype=np.float32)

    trained_eps = []
    random_eps = []
    handcrafted_eps = []

    for i in range(args.episodes):
        seed_i = args.seed + i
        env = CoverageEnv(episode_seconds=args.episode_seconds, randomize_sectors=False,
                          optimizer_type=args.optimizer)
        env.reset(seed=seed_i)
        print(f"\nEpisode {i+1}/{args.episodes} (seed={seed_i}, optimizer={args.optimizer})")

        env_t = CoverageEnv(episode_seconds=args.episode_seconds, randomize_sectors=False,
                            optimizer_type=args.optimizer)
        env_t.reset(seed=seed_i)
        if args.all_drones_trained:
            env_t.shadow_policy_fn = trained_policy
        trained_eps.append(_run_episode(env_t, trained_policy, "trained"))
        print(f"  trained:    mean r_drone={trained_eps[-1]['r_drone'].mean():.3f}, total reward={trained_eps[-1]['reward'].sum():+.1f}")

        env_r = CoverageEnv(episode_seconds=args.episode_seconds, randomize_sectors=False,
                            optimizer_type=args.optimizer)
        env_r.reset(seed=seed_i)
        random_eps.append(_run_episode(env_r, random_policy, "random"))
        print(f"  random:     mean r_drone={random_eps[-1]['r_drone'].mean():.3f}, total reward={random_eps[-1]['reward'].sum():+.1f}")

        env_h = CoverageEnv(episode_seconds=args.episode_seconds, randomize_sectors=False,
                            optimizer_type=args.optimizer)
        env_h.reset(seed=seed_i)
        handcrafted_eps.append(_run_episode(env_h, hold_centerline_policy, "hand-99"))
        print(f"  hand-0.99:  mean r_drone={handcrafted_eps[-1]['r_drone'].mean():.3f}, total reward={handcrafted_eps[-1]['reward'].sum():+.1f}")

    summaries = [
        _summarize(trained_eps, "trained"),
        _summarize(random_eps, "random"),
        _summarize(handcrafted_eps, "hand-0.99"),
    ]
    print("\n" + "=" * 75)
    print(f"SUMMARY (mean across episodes) — optimizer={args.optimizer}")
    print("=" * 75)
    _print_table(summaries)
    print("=" * 75)

    # ----------------------------------------------------------------
    # Joint constraint analysis — Σ c_i r² over the 60s wind-handoff window.
    # ----------------------------------------------------------------
    B_op = 8.15
    B = 8.0
    print(f"\nJOINT CONSTRAINT ANALYSIS — optimizer={args.optimizer}")
    print(f"  B (optimizer constraint) = {B}, B_op (data-loss threshold) = {B_op}")
    print(f"  Wind-handoff window: t ∈ [150s, 210s]")
    print(f"  {'Group':<15} {'mean Σ_drone':>12} {'max Σ_drone':>12} {'mean Σ_leash':>12} "
          f"{'max Σ_leash':>12} {'% t>B_op':>10}")
    print("  " + "-" * 80)
    for label, eps in [("trained", trained_eps), ("random", random_eps), ("hand-0.99", handcrafted_eps)]:
        drone_in_window = []
        leash_in_window = []
        for ep in eps:
            ts = ep.get("ts", np.array([]))
            sd = ep.get("joint_sum_drone", np.array([]))
            sl = ep.get("joint_sum_leash", np.array([]))
            if len(ts) == 0:
                continue
            mask = (ts >= 150.0) & (ts <= 210.0)
            drone_in_window.append(sd[mask])
            leash_in_window.append(sl[mask])
        if not drone_in_window:
            continue
        d_all = np.concatenate(drone_in_window)
        l_all = np.concatenate(leash_in_window)
        pct_over = float((d_all > B_op).sum() / max(len(d_all), 1)) * 100.0
        print(f"  {label:<15} {d_all.mean():>12.3f} {d_all.max():>12.3f} "
              f"{l_all.mean():>12.3f} {l_all.max():>12.3f} {pct_over:>9.1f}%")
    print("  " + "-" * 80)
    print(f"  Σ_drone = Σ c_i × r_drone²  (PHYSICAL violation, demo plot metric)")
    print(f"  Σ_leash = Σ c_i × (r_i^k)²  (OPTIMIZER violation, paper's headline claim)")
    print(f"  % t>B_op = fraction of ticks in window where Σ_drone > B_op = {B_op}")

    # Pass/fail vs the locked behavior priorities
    print("\nBehavior check vs intended:")
    t = summaries[0]
    r = summaries[1]
    h = summaries[2]

    def check(name, ok, detail):
        tag = "✓" if ok else "✗"
        print(f"  [{tag}] {name}: {detail}")

    check("Trained beats random on cum reward",
          t["mean_cum_reward"] > r["mean_cum_reward"],
          f"trained={t['mean_cum_reward']:.1f}  random={r['mean_cum_reward']:.1f}")
    check("Trained heads outward (mean r > random's mean r)",
          t["mean_edge_util"] > r["mean_edge_util"],
          f"trained edge_util={t['mean_edge_util']:.3f}  random={r['mean_edge_util']:.3f}")
    check("Trained edge_util on path toward hand-crafted ~0.99",
          t["mean_edge_util"] > 0.4,
          f"trained edge_util={t['mean_edge_util']:.3f}  (target ~0.99, hand-crafted={h['mean_edge_util']:.3f})")
    check("Trained smooth penalty smaller than random's",
          t["mean_cum_smooth"] > r["mean_cum_smooth"],
          f"trained smooth={t['mean_cum_smooth']:.1f}  random={r['mean_cum_smooth']:.1f}  (less negative = smoother)")


if __name__ == "__main__":
    main()
