"""Phase 2 — env verification.

Unit tests per component + numerical match against the ROS node math + a
hand-crafted manual rollout. Prints a pass/fail summary at the end.

Run from the workspace root:
    PYTHONPATH=src/rl_demo:src python3 src/rl_demo/rl_training/tests/test_phase2_verify.py
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np

_THIS = Path(__file__).resolve()
_RL_DEMO = _THIS.parents[2]
_WS_SRC = _THIS.parents[3]
for p in (str(_RL_DEMO), str(_WS_SRC)):
    if p not in sys.path:
        sys.path.insert(0, p)

from thermal_mapping.thermal_field import ThermalField

from rl_training.env import CoverageEnv
from rl_training.env.dynamics import PointMassDrone
from rl_training.env.optimizer_wrap import OptimizerWrap, estimate_q_for_sector
from rl_training.env.rewards import RewardWeights, cell_value
from rl_training.env.sensor_model import ThermalSensor
from rl_training.env.thermal_map import ThermalMap


_results = []


def _record(name: str, ok: bool, detail: str = ""):
    status = "PASS" if ok else "FAIL"
    _results.append((name, ok, detail))
    print(f"  [{status}] {name}{(' — ' + detail) if detail else ''}")


# ----------------------------------------------------------------------
# 1. Dynamics — verifies my PointMassDrone class mathematically implements
#    SetpointStreamer's rate-limit formula (pos += clip(target-pos, ±v*dt)).
#    Does NOT compare to Gazebo's quadrotor physics — firmware PID dynamics
#    are unmodeled and treated as the residual sim2real gap, to be checked
#    only at the crazysim validation step (Phase 6).
# ----------------------------------------------------------------------
def test_dynamics():
    print("\n== Dynamics (vs SetpointStreamer rate-limit math) ==")
    # 1a: drone at origin moving toward (10m, 0) at 1.5 m/s for 1s should reach 1.5m.
    drone = PointMassDrone(initial_xy=(0.0, 0.0), max_velocity=1.5, streamer_rate_hz=20.0)
    drone.step(np.array([10.0, 0.0], dtype=np.float32), dt=1.0)
    travelled = float(drone.position[0])
    _record("1.5 m/s cap over 1s gives ~1.5m", abs(travelled - 1.5) < 1e-3,
            f"travelled = {travelled:.4f} m")

    # 1b: drone reaches target exactly when within step.
    drone.reset((0.0, 0.0))
    drone.step(np.array([0.05, 0.0], dtype=np.float32), dt=1.0)
    _record("Reaches close target exactly", abs(float(drone.position[0]) - 0.05) < 1e-6,
            f"pos = {drone.position[0]:.6f}")

    # 1c: velocity property reports correct outer-dt velocity.
    drone.reset((0.0, 0.0))
    drone.step(np.array([10.0, 0.0], dtype=np.float32), dt=0.2)
    expected_speed = 0.3 / 0.2   # moved 0.3m in 0.2s = 1.5 m/s
    actual_speed = float(np.linalg.norm(drone.velocity))
    _record("Velocity property = outer dt velocity", abs(actual_speed - expected_speed) < 1e-3,
            f"|v| = {actual_speed:.3f} m/s (expected ~{expected_speed:.3f})")

    # 1d: training-time cap of 1.0 m/s (the conservative hedge).
    drone = PointMassDrone(initial_xy=(0.0, 0.0), max_velocity=1.0, streamer_rate_hz=20.0)
    drone.step(np.array([10.0, 0.0], dtype=np.float32), dt=1.0)
    travelled = float(drone.position[0])
    _record("Training max_v=1.0 cap over 1s gives ~1.0m", abs(travelled - 1.0) < 1e-3,
            f"travelled = {travelled:.4f} m")


# ----------------------------------------------------------------------
# 2. Sensor: numerical match against thermal_sensor_node.py's math.
# ----------------------------------------------------------------------
def test_sensor():
    print("\n== Sensor (vs thermal_sensor_node.py math) ==")
    field = ThermalField.from_yaml(str(_WS_SRC / "thermal_mapping" / "config" / "thermal_field.yaml"))

    # Mirror the ROS node's exact computation locally (no noise).
    def ros_truth(px, py, pz, fov_deg, resolution, t):
        h = pz * math.tan(math.radians(fov_deg) / 2.0)
        u = (np.arange(resolution) + 0.5) / resolution * 2.0 - 1.0
        gx, gy = np.meshgrid(u, u, indexing="xy")
        xs = px + gx * h
        ys = py + gy * h
        return field.evaluate(xs, ys, t)

    pose = (0.5, 0.3)
    altitude = 0.6
    fov_deg = 35.0
    res = 16
    t = 120.0

    our_sensor = ThermalSensor(
        field=field, altitude=altitude, fov_deg=fov_deg, resolution=res,
        noise_sigma=0.0, latency_seconds=0.0, sensor_rate_hz=5.0,
    )
    our_sensor.sample(np.array(pose, dtype=np.float32), t)
    our_truth = our_sensor.read(t)
    ros_value = ros_truth(*pose, altitude, fov_deg, res, t)
    err = float(np.abs(our_truth - ros_value).max())
    _record("Sensor TRUTH matches ROS math (max |Δ| < 1e-5)", err < 1e-5,
            f"max_abs_err = {err:.2e}")

    # 2b: noise statistics over many samples ≈ σ=0.5°C.
    rng = np.random.default_rng(0)
    noisy_sensor = ThermalSensor(
        field=field, altitude=altitude, fov_deg=fov_deg, resolution=res,
        noise_sigma=0.5, latency_seconds=0.0, rng=rng,
    )
    diffs = []
    for _ in range(500):
        noisy_sensor.sample(np.array(pose, dtype=np.float32), t)
        sample = noisy_sensor.read(t)
        diffs.append(sample - ros_value)
    diffs = np.array(diffs)
    sigma_est = float(diffs.std())
    _record("Sensor noise σ ≈ 0.5°C (estimated within 0.05)", abs(sigma_est - 0.5) < 0.05,
            f"σ̂ = {sigma_est:.4f}")

    # 2c: footprint side = 2 * h * tan(fov/2) at h=0.6m, fov=35° → 0.378m.
    h_half = altitude * math.tan(math.radians(fov_deg) / 2.0)
    footprint_side = 2.0 * h_half
    _record("Footprint side ≈ 0.378m at h=0.6, fov=35°", abs(footprint_side - 0.378) < 0.005,
            f"footprint = {footprint_side:.4f} m")


# ----------------------------------------------------------------------
# 3. Thermal map: age tracker increments / resets correctly.
# ----------------------------------------------------------------------
def test_thermal_map():
    print("\n== Thermal map ==")
    tm = ThermalMap(length_x=5.0, length_y=5.0, resolution=0.01)
    # Pre-update: all cells unobserved.
    ages = tm.age_seconds(now=10.0)
    _record("Initial ages are all NaN", bool(np.all(np.isnan(ages))))

    # Update a dense 16×16 sensor footprint centered at (0, 0) — same shape
    # as the real ThermalSensor produces.
    res = 16
    h_half = 0.189   # 35° FOV at 0.6m → half-side 0.189m
    u = (np.arange(res) + 0.5) / res * 2.0 - 1.0
    gx, gy = np.meshgrid(u * h_half, u * h_half, indexing="xy")
    grid_xy = np.stack([gx, gy], axis=-1).astype(np.float32)
    values = np.full((res, res), 40.0, dtype=np.float32)
    tm.update_from_frame(np.array([0.0, 0.0]), grid_xy, values, t=5.0)
    # The sensor footprint covers (0, 0); age there should be 5.0 at now=10.0.
    row, col = tm.world_to_grid(0.0, 0.0)
    age_now = float(tm.age_seconds(now=10.0)[row, col])
    _record("Cell observed at t=5, age at t=10 ≈ 5s", abs(age_now - 5.0) < 0.01,
            f"age = {age_now:.4f}")

    # Sector crop: cf1 wedge (phi_mid=0°, phi_half=45°, r_max=1.9).
    crop = tm.sector_age_crop(
        now=10.0, phi_mid_rad=0.0, phi_half_rad=math.radians(45.0), r_max=1.9, crop_size=32,
    )
    _record("Sector crop shape (32, 32)", crop.shape == (32, 32))
    # Cells in the wedge with no observation → unobserved_age_cap=360 (default).
    _record("Unobserved wedge cells = 360 (cap)", float(crop.max()) == 360.0,
            f"max = {crop.max():.1f}")
    # Cells outside wedge → 0 (masked).
    _record("Out-of-wedge cells masked to 0", float(crop.min()) == 0.0,
            f"min = {crop.min():.1f}")


# ----------------------------------------------------------------------
# 4. Optimizer: converges near KKT under static q (sanity, not bitwise).
# ----------------------------------------------------------------------
def test_optimizer():
    print("\n== Optimizer ==")
    # Static q from arena_4drone.yaml: cf1=10, cf2=1.5, cf3=1, cf4=0.5.
    # KKT reference (validated to ~4cm in 2D): r ≈ [1.78, 1.45, 1.31, 1.01].
    drones_cfg = [
        {"name": "cf1", "q": 10.0, "c": 1.0, "r_star": 1.85, "r_max": 1.9},
        {"name": "cf2", "q": 1.5,  "c": 1.0, "r_star": 1.85, "r_max": 1.9},
        {"name": "cf3", "q": 1.0,  "c": 1.0, "r_star": 1.85, "r_max": 1.9},
        {"name": "cf4", "q": 0.5,  "c": 1.0, "r_star": 1.85, "r_max": 1.9},
    ]
    opt = OptimizerWrap(
        drones_cfg=drones_cfg, budget_B=8.0, round_rate_hz=0.5,
        T_inner=5, c1=0.015, c2=0.18, noise_bound=0.0, seed=0,
    )
    # 200 rounds at 0.5 Hz = 400s of sim time. Run extra rounds to let cf4
    # (low q=0.5, slow gradient) converge tighter than 6-7cm.
    for k in range(200):
        opt.maybe_step(t=k * 2.0)
    final = opt.leashes.tolist()
    kkt = np.array([1.78, 1.45, 1.31, 1.01])
    err = np.max(np.abs(np.array(final) - kkt))
    _record("After 200 rounds, r within 10cm of KKT", err < 0.10,
            f"r = {[f'{r:.3f}' for r in final]} vs KKT {kkt.tolist()}, max |Δ| = {err:.4f}")
    # Joint constraint: Σ c_i r² ≤ B+small.
    sum_cr2 = sum(d.c * d.r ** 2 for d in opt.opt.drones)
    _record("Σ c_i r² ≤ B = 8 (within 0.1)", sum_cr2 <= 8.1,
            f"Σ c_i r² = {sum_cr2:.4f}")


# ----------------------------------------------------------------------
# 5. Rewards: cell-value gives positive signal for hot+unexplored cells.
# ----------------------------------------------------------------------
def test_rewards():
    print("\n== Rewards ==")
    w = RewardWeights()
    # Hot, unexplored, stale cell → high value.
    temp = np.array([72.0])
    age = np.array([60.0])
    was_visited = np.array([False])
    v_unexplored_hot = float(cell_value(temp, age, was_visited, t_ambient=22.0, w=w, candidate="A").item())
    # Same cell, just visited (age=0) → zero value.
    v_just_visited = float(cell_value(temp, np.array([0.0]), np.array([True]),
                                       t_ambient=22.0, w=w, candidate="A").item())
    _record("Hot+unexplored has higher value than just-visited",
            v_unexplored_hot > v_just_visited and v_just_visited < 1.0,
            f"v_hot_unexp = {v_unexplored_hot:.2f}, v_visited = {v_just_visited:.4f}")

    # Pure cold cell, never visited → only the explore bonus (small but positive).
    temp_cold = np.array([22.0])
    v_cold_unvisited = float(cell_value(temp_cold, age, was_visited, t_ambient=22.0, w=w, candidate="A").item())
    _record("Cold+unvisited has small positive value (explore bonus)",
            0 < v_cold_unvisited < v_unexplored_hot,
            f"v_cold_unvisited = {v_cold_unvisited:.4f}")


# ----------------------------------------------------------------------
# 6. Hand-crafted rollouts: three scenarios each isolating a reward term.
# ----------------------------------------------------------------------
def _run_scenario(label: str, action_fn, n_steps=600, seed=11):
    env = CoverageEnv(episode_seconds=200.0, randomize_sectors=False)
    obs, info = env.reset(seed=seed)
    rewards, parts_list, r_drone_list, leash_list = [], [], [], []
    for i in range(n_steps):
        action = action_fn(i)
        obs, reward, terminated, truncated, info = env.step(action)
        rewards.append(reward)
        parts_list.append(info.get("reward_parts", {}))
        r_drone_list.append(info["r_drone"])
        leash_list.append(info["r_leash"])
    return rewards, parts_list, r_drone_list, leash_list


def _cum_part(parts_list, key):
    return float(sum(p.get(key, 0.0) for p in parts_list))


def test_manual_rollout():
    print("\n== Manual rollouts (3 scenarios isolating reward terms) ==")
    n_steps = 600   # 120s of sim — fire has spread plenty by t=100s.

    # Scenario A: drone near fire core → coverage term should dominate positively.
    rewards_A, parts_A, r_A, leash_A = _run_scenario(
        "A: fire core",
        action_fn=lambda i: np.array([0.0, 0.05], dtype=np.float32),
        n_steps=n_steps,
    )
    coverage_A = _cum_part(parts_A, "coverage")
    edge_A = _cum_part(parts_A, "edge")
    smooth_A = _cum_part(parts_A, "smooth")
    print(f"  Scenario A (drone near fire core, theta=0, r_frac=0.05):")
    print(f"    cum coverage={coverage_A:+.2f}  edge={edge_A:+.2f}  smooth={smooth_A:+.2f}")
    _record("[A] coverage term positive after fire spread", coverage_A > 0,
            f"cum coverage = {coverage_A:+.2f}")

    # Scenario B: drone sweeping at the edge (r_fraction = 0.99) → edge_pull dominates.
    rewards_B, parts_B, r_B, leash_B = _run_scenario(
        "B: edge sweep",
        action_fn=lambda i: np.array([np.sin(i * 0.05) * 0.9, 0.99], dtype=np.float32),
        n_steps=n_steps,
    )
    coverage_B = _cum_part(parts_B, "coverage")
    edge_B = _cum_part(parts_B, "edge")
    smooth_B = _cum_part(parts_B, "smooth")
    print(f"  Scenario B (drone sweeping at edge, theta=sin sweep, r_frac=0.99):")
    print(f"    cum coverage={coverage_B:+.2f}  edge={edge_B:+.2f}  smooth={smooth_B:+.2f}")
    _record("[B] edge_pull positive", edge_B > 0, f"cum edge = {edge_B:+.2f}")
    _record("[B] more edge_pull than scenario A (drone is at edge)", edge_B > edge_A,
            f"edge_B={edge_B:+.2f} vs edge_A={edge_A:+.2f}")

    # Scenario C: thrashing action → smoothness penalty dominates negatively.
    rng = np.random.default_rng(42)
    def thrash_action(i):
        # Alternating ±0.9 theta + alternating r_fraction ∈ [0.3, 0.9]
        return np.array([0.9 if i % 2 == 0 else -0.9,
                         0.3 if i % 2 == 0 else 0.9], dtype=np.float32)
    rewards_C, parts_C, r_C, leash_C = _run_scenario(
        "C: thrashing",
        action_fn=thrash_action,
        n_steps=n_steps,
    )
    smooth_C = _cum_part(parts_C, "smooth")
    print(f"  Scenario C (thrashing action):")
    print(f"    cum smooth={smooth_C:+.2f}")
    _record("[C] smoothness penalty fires (negative)", smooth_C < 0,
            f"cum smooth = {smooth_C:+.2f}")
    _record("[C] more negative smooth than scenario A (action is jittery)", smooth_C < smooth_A,
            f"smooth_C={smooth_C:+.2f} vs smooth_A={smooth_A:+.2f}")

    # Save matplotlib trace comparing all three scenarios (3 columns).
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        out_dir = _WS_SRC.parent / "exp1" / "rl_training"
        out_dir.mkdir(parents=True, exist_ok=True)
        scenarios = [
            ("A: fire core", r_A, leash_A, rewards_A, parts_A),
            ("B: edge sweep", r_B, leash_B, rewards_B, parts_B),
            ("C: thrashing", r_C, leash_C, rewards_C, parts_C),
        ]
        fig, axes = plt.subplots(3, len(scenarios), figsize=(14, 8), sharex=True)
        t_arr = np.arange(n_steps) * 0.2
        for col, (label, r_list, leash_list, reward_list, parts_list) in enumerate(scenarios):
            axes[0, col].plot(t_arr, r_list, label="r_drone")
            axes[0, col].plot(t_arr, leash_list, label="opt_leash")
            axes[0, col].set_title(label)
            axes[0, col].set_ylabel("radius (m)")
            axes[0, col].legend(loc="best", fontsize=8)
            axes[0, col].grid(True, alpha=0.3)
            axes[1, col].plot(t_arr, reward_list, color="tab:green")
            axes[1, col].set_ylabel("reward / tick")
            axes[1, col].grid(True, alpha=0.3)
            axes[2, col].plot(t_arr, [p.get("coverage", 0.0) for p in parts_list], label="coverage")
            axes[2, col].plot(t_arr, [p.get("edge", 0.0) for p in parts_list], label="edge")
            axes[2, col].plot(t_arr, [p.get("smooth", 0.0) for p in parts_list], label="smooth")
            axes[2, col].set_xlabel("sim time (s)")
            axes[2, col].set_ylabel("reward parts")
            axes[2, col].legend(loc="best", fontsize=8)
            axes[2, col].grid(True, alpha=0.3)
        fig.suptitle("Phase 2 manual rollouts — 3 scenarios isolating reward terms")
        fig.tight_layout()
        plot_path = out_dir / "phase2_manual_rollouts.png"
        fig.savefig(plot_path, dpi=110)
        plt.close(fig)
        _record("Saved rollout plot", plot_path.exists(), f"-> {plot_path}")
    except ImportError:
        _record("Matplotlib available", False, "skipped plot")


# ----------------------------------------------------------------------
# 7. Throughput — informational only, used to firm up Phase 4/server time estimate.
# ----------------------------------------------------------------------
def test_throughput():
    print("\n== Throughput (informational, no pass/fail) ==")
    env = CoverageEnv(episode_seconds=200.0, randomize_sectors=False)
    env.reset(seed=0)
    n_steps = 500
    actions = np.tile(np.array([0.0, 0.9], dtype=np.float32), (n_steps, 1))
    start = time.perf_counter()
    for i in range(n_steps):
        env.step(actions[i])
    elapsed = time.perf_counter() - start
    rate = n_steps / elapsed
    print(f"  Single-thread: {rate:.0f} steps/sec ({elapsed:.2f}s for {n_steps} steps)")
    # Extrapolated training time estimates assuming linear scaling across envs.
    print(f"  Extrapolated wall-clock for total PPO steps (assuming linear scale):")
    for n_envs in (8, 32, 64, 128):
        aggregate = rate * n_envs
        for total in (100_000, 2_000_000, 8_000_000):
            secs = total / aggregate
            t_str = f"{secs:.1f}s" if secs < 60 else (f"{secs/60:.1f} min" if secs < 3600 else f"{secs/3600:.2f} hr")
            print(f"    n_envs={n_envs:3d} × {total:>9,} steps → {t_str}")


def main():
    test_dynamics()
    test_sensor()
    test_thermal_map()
    test_optimizer()
    test_rewards()
    test_manual_rollout()
    test_throughput()

    print("\n" + "=" * 60)
    n_pass = sum(1 for _, ok, _ in _results if ok)
    n_fail = sum(1 for _, ok, _ in _results if not ok)
    print(f"PHASE 2 SUMMARY: {n_pass} PASS, {n_fail} FAIL  /  {len(_results)} total")
    if n_fail:
        print("\nFailed tests:")
        for name, ok, detail in _results:
            if not ok:
                print(f"  - {name}: {detail}")
    print("=" * 60)
    sys.exit(0 if n_fail == 0 else 1)


if __name__ == "__main__":
    main()
