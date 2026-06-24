"""Gymnasium env for the federated coverage RL policy.

One env = one DRONE's perspective over a 360s fire episode. The env owns
its own world state (fire field, optimizer running for all 4 drones, age
map). The other 3 drones follow a hand-coded "shadow" policy (currently:
hold at origin; later: swap in polar lawnmower) so their leashes evolve
realistically through the federated optimizer.

Action: sector-polar (θ ∈ [-1, +1], r_fraction ∈ [0, slack]).
Observation: dict with thermal (16×16), age (32×32), and a 28-vector of
scalars (own leash, pose, velocity, sector geometry, past 10 actions,
episode time).

This env is designed for single-agent shared-policy training: multiple
copies running in parallel (different drone-index/sector assignments)
share the same trained policy network.
"""

from __future__ import annotations

import math
import sys
from collections import deque
from pathlib import Path
from typing import Any, Optional

import gymnasium as gym
import numpy as np
import yaml

# Make sibling ROS Python packages importable for thermal_field and fed_dcsa.
# parents[3] is cs2_ws/src; adding cs2_ws/src/<pkg> exposes the inner Python module.
_CS2_WS_SRC = Path(__file__).resolve().parents[3]
for sub in ("thermal_mapping", "fed_dcsa", "cf_coverage_planner"):
    p = _CS2_WS_SRC / sub
    if p.is_dir() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

from thermal_mapping.thermal_field import ThermalField

from .dynamics import CascadedPidDrone, PidGains, PointMassDrone, randomize_gains
from .optimizer_wrap import OptimizerWrap, estimate_q_for_sector
from .rewards import (
    RewardContext,
    RewardWeights,
    cell_value,
    compute_reward,
)
from .sensor_model import ThermalSensor
from .thermal_map import ThermalMap


DEFAULT_ARENA_YAML = str(_CS2_WS_SRC / "cf_coverage_planner" / "config" / "arena_4drone.yaml")
DEFAULT_THERMAL_YAML = str(_CS2_WS_SRC / "thermal_mapping" / "config" / "thermal_field.yaml")


def _load_sector_cfg(arena_yaml: str) -> list[dict]:
    with open(arena_yaml, "r") as f:
        doc = yaml.safe_load(f)
    names = doc["drone_names"]
    sectors = doc["sectors"]
    cfg = []
    for name in names:
        s = sectors[name]
        cfg.append({
            "name": name,
            "phi_mid_deg": float(s["phi_mid_deg"]),
            "phi_half_deg": float(s["phi_half_deg"]),
            "q": float(s["q"]),
            "c": float(s["c"]),
            "r_star": float(s["r_star"]),
            "r_max": float(s["r_max"]),
        })
    optimizer_doc = doc["optimizer"]
    streamer_doc = doc["streamer"]
    planner_doc = doc["planner"]
    sensor_doc = doc["sensor"]
    arena_doc = doc["arena"]
    return {
        "sectors": cfg,
        "drone_names": names,
        "budget_B": float(optimizer_doc["budget_B"]),
        "round_rate_hz": float(optimizer_doc["round_rate_hz"]),
        "K": int(optimizer_doc["K"]),
        "T_inner": int(optimizer_doc["T"]),
        "c1": float(optimizer_doc["c_1"]),
        "c2": float(optimizer_doc["c_2"]),
        "noise_bound": float(optimizer_doc["noise_bound"]),
        "max_velocity": float(streamer_doc["max_setpoint_velocity"]),
        "setpoint_rate_hz": float(streamer_doc["setpoint_rate_hz"]),
        "planner_rate_hz": float(planner_doc["planner_rate_hz"]),
        "sensor_fov_deg": float(sensor_doc["fov_deg"]),
        "altitude": float(arena_doc["altitude"]),
    }


class CoverageEnv(gym.Env):
    """Single-drone slice of the 4-drone federated coverage scenario."""

    metadata = {"render_modes": []}

    def __init__(
        self,
        arena_yaml: str = DEFAULT_ARENA_YAML,
        thermal_yaml: str = DEFAULT_THERMAL_YAML,
        episode_seconds: float = 360.0,
        tick_hz: float = 5.0,
        action_slack: float = 1.0,   # iter13: 1.03 → 1.0 to close the [0.99, 1.03] "no-man's-land" exploit. With symmetric anchor at edge_target=0.99 and action upper-bound at 1.0, the only zero-penalty state is r/leash=0.99.
        past_action_history: int = 10,
        age_crop_size: int = 32,
        reward_candidate: str = "A",
        reward_weights: Optional[RewardWeights] = None,
        randomize_sectors: bool = True,
        randomize_seed_range: tuple[int, int] = (0, 10_000_000),
        min_leash: float = 0.4,
        max_velocity_override: float = 1.0,
        r_fraction_min: float = 0.0,
        optimizer_type: str = "feddcsa",   # "feddcsa" | "lagrangian"
        # NEW: which xy dynamics model to use.
        # "point_mass" — legacy, used to train iter1–9.
        # "cascaded_pid" — firmware-realistic; required for sim2real transfer
        #                  to the cf2-sitl + HW Crazyflie.
        dynamics_model: str = "cascaded_pid",
        # If True, sample fresh PidGains every reset within randomize_gains()'s
        # ±25%-of-firmware-default ranges. Sim2real-standard.
        randomize_dynamics: bool = True,
        seed: Optional[int] = None,
    ):
        super().__init__()
        self.arena_yaml = arena_yaml
        self.thermal_yaml = thermal_yaml
        self.episode_seconds = float(episode_seconds)
        self.tick_dt = 1.0 / float(tick_hz)
        self.action_slack = float(action_slack)
        self.past_n = int(past_action_history)
        self.age_crop_size = int(age_crop_size)
        self.randomize_sectors = bool(randomize_sectors)
        self.randomize_seed_range = randomize_seed_range
        self.min_leash = float(min_leash)
        self.optimizer_type = str(optimizer_type).lower()
        self.dynamics_model = str(dynamics_model).lower()
        self.randomize_dynamics = bool(randomize_dynamics)
        if self.dynamics_model not in ("point_mass", "cascaded_pid"):
            raise ValueError(
                f"dynamics_model must be 'point_mass' or 'cascaded_pid', "
                f"got {self.dynamics_model!r}")
        # Optional callable for shadow drones: shadow_policy_fn(obs_dict) -> action np.ndarray(2,)
        # If None (default), shadow drones use the hardcoded "0.85 leash centerline" heuristic.
        # Used for deployment-style eval (all 4 drones running the trained policy).
        self.shadow_policy_fn = None
        self._cfg = _load_sector_cfg(arena_yaml)
        # Override the YAML's max_velocity (1.5 m/s) with the training-time
        # conservative hedge (default 1.0 m/s). Deployment streamer remains
        # at 1.5; policy is trained to plan within a tighter envelope so it
        # never commands targets that push the streamer's actual cap.
        self._cfg["max_velocity"] = float(max_velocity_override)

        # Reward setup
        self.reward_ctx = RewardContext(
            candidate=reward_candidate,
            weights=reward_weights if reward_weights is not None else RewardWeights(),
            dt=self.tick_dt,
        )

        # Action: (theta in [-1, +1], r_fraction in [r_fraction_min, action_slack])
        # r_fraction_min lets us hard-constrain the drone to ride the edge ring
        # by clipping the action space itself (cleaner than reward-side shaping).
        self.r_fraction_min = float(r_fraction_min)
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, self.r_fraction_min], dtype=np.float32),
            high=np.array([1.0, self.action_slack], dtype=np.float32),
            shape=(2,),
            dtype=np.float32,
        )

        # Observation: dict with 'thermal' (16×16), 'age' (32×32), 'vec' (1308-256-1024=28 scalars)
        n_vec = 1 + 2 + 2 + 2 + 2 * self.past_n + 1   # leash, pose, vel, sector geom, past actions, episode time
        self.observation_space = gym.spaces.Dict({
            "thermal": gym.spaces.Box(-50.0, 200.0, shape=(16, 16), dtype=np.float32),
            "age": gym.spaces.Box(0.0, 360.0, shape=(self.age_crop_size, self.age_crop_size), dtype=np.float32),
            "vec": gym.spaces.Box(-10.0, 1000.0, shape=(n_vec,), dtype=np.float32),
        })

        # RNG seeded later in reset; rest of state initialized lazily.
        self._np_random: np.random.Generator = np.random.default_rng(seed)
        self._field: ThermalField | None = None
        self._sensor: ThermalSensor | None = None
        self._map: ThermalMap | None = None
        self._opt: OptimizerWrap | None = None
        self._drones: list = []   # CascadedPidDrone | PointMassDrone
        self._sector_assignment: list[int] = []   # for each drone slot, index into self._cfg['sectors']
        self._my_drone_idx: int = 0
        self._t: float = 0.0
        self._past_actions: deque = deque(maxlen=self.past_n)
        self._prev_action_raw: np.ndarray | None = None
        self._prev_position: np.ndarray | None = None

    # ------------------------------------------------------------------
    # Gym API
    # ------------------------------------------------------------------
    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict[str, Any]] = None,
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        super().reset(seed=seed)
        if seed is not None:
            self._np_random = np.random.default_rng(seed)

        # Per-episode randomization: fire seed + sector shuffle.
        thermal_seed = int(self._np_random.integers(self.randomize_seed_range[0], self.randomize_seed_range[1]))
        self._build_field(thermal_seed)
        self._build_optimizer(thermal_seed)
        self._build_runtime()

        # Pick which sector slot this env represents. With sector shuffle,
        # the policy sees a uniformly-random sector each episode.
        slot = int(self._np_random.integers(0, len(self._cfg["sectors"])))
        self._my_drone_idx = slot

        self._t = 0.0
        self._past_actions.clear()
        for _ in range(self.past_n):
            self._past_actions.append(np.zeros(2, dtype=np.float32))
        self._prev_action_raw = None
        self._prev_position = self._drones[slot].position.copy()
        # iter12: reset the θ-bias EMA so it doesn't leak across episodes.
        self.reward_ctx.theta_bias_ema = 0.0
        # iter17: reset the angular-coverage sweep bins (per-episode staleness of each
        # sector angular bin; collected/aged in _compute_reward → drives a wide arc sweep).
        self._angular_bin_age = np.zeros(int(self.reward_ctx.weights.sweep_n_bins), dtype=np.float64)

        obs = self._build_obs()
        info = self._build_info()
        return obs, info

    def step(self, action: np.ndarray) -> tuple[dict[str, np.ndarray], float, bool, bool, dict[str, Any]]:
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, self.action_space.low, self.action_space.high)

        # ------------------------------------------------------------------
        # 1. Advance the optimizer if it's time for a new round.
        # ------------------------------------------------------------------
        q_values = [self._estimate_q(i) for i in range(len(self._sector_assignment))]
        self._opt.maybe_step(self._t, q_values=q_values)
        leashes = self._opt.leashes

        # ------------------------------------------------------------------
        # 2. Translate THIS drone's action → world target. Move it.
        #    Other drones follow a shadow target (hold at last-leash * 0.7,
        #    sector centerline; placeholder until a richer behavior is set).
        # ------------------------------------------------------------------
        sector_assn = self._sector_assignment
        for slot_idx, drone in enumerate(self._drones):
            sector_cfg = self._cfg["sectors"][sector_assn[slot_idx]]
            phi_mid = math.radians(sector_cfg["phi_mid_deg"])
            phi_half = math.radians(sector_cfg["phi_half_deg"])
            r_max = sector_cfg["r_max"]
            leash = float(leashes[slot_idx])
            # Safety floor mirroring quadrant_figure8_node.py:202.
            effective_leash = max(leash, self.min_leash)
            if slot_idx == self._my_drone_idx:
                theta = float(action[0])
                r_frac = float(action[1])
            elif self.shadow_policy_fn is not None:
                # Deployment-style eval: each shadow drone uses the trained policy
                # with its OWN observation. shadow_policy_fn(obs) -> (θ, r_frac).
                shadow_obs = self._build_obs_for_drone(slot_idx)
                shadow_action = self.shadow_policy_fn(shadow_obs)
                shadow_action = np.clip(shadow_action, self.action_space.low, self.action_space.high)
                theta = float(shadow_action[0])
                r_frac = float(shadow_action[1])
            else:
                # Default heuristic: 0.85× effective_leash centerline.
                theta = 0.0
                r_frac = 0.85
            target_r = min(r_frac * effective_leash, r_max)
            target_phi = phi_mid + theta * phi_half
            target = np.array([target_r * math.cos(target_phi),
                               target_r * math.sin(target_phi)],
                              dtype=np.float32)
            target = self._overshoot_aware_filter(
                target, drone, phi_mid, phi_half, r_max)
            drone.step(target, self.tick_dt)

        self._t += self.tick_dt

        # ------------------------------------------------------------------
        # 3. SNAPSHOT pre-fusion ages for OUR drone's FOV (used by reward).
        # ------------------------------------------------------------------
        my_drone = self._drones[self._my_drone_idx]
        my_sensor = self._sensors[self._my_drone_idx]
        fov_xs = my_drone.position[0] + my_sensor._gx_h
        fov_ys = my_drone.position[1] + my_sensor._gy_h
        snapshot = self._snapshot_fov_ages(fov_xs, fov_ys)

        # ------------------------------------------------------------------
        # 4. Each drone (incl. shadows) samples its thermal sensor & writes
        #    into the fused map. This mirrors live multi-drone fusion.
        # ------------------------------------------------------------------
        for slot_idx, drone in enumerate(self._drones):
            sensor = self._sensors[slot_idx]
            sensor.sample(drone.position, self._t)
            xs = drone.position[0] + sensor._gx_h
            ys = drone.position[1] + sensor._gy_h
            grid_xy = np.stack([xs, ys], axis=-1)
            values = sensor.read(self._t)
            self._map.update_from_frame(drone.position, grid_xy, values, self._t)

        # ------------------------------------------------------------------
        # 5. Compute reward using PRE-fusion snapshot for the coverage term.
        # ------------------------------------------------------------------
        reward, reward_parts = self._compute_reward(action, snapshot=snapshot,
                                                    fov_xs=fov_xs, fov_ys=fov_ys)

        # Update history.
        self._past_actions.append(action.copy())
        self._prev_action_raw = action.copy()
        self._prev_position = self._drones[self._my_drone_idx].position.copy()

        # ------------------------------------------------------------------
        # 5. Build next obs, decide termination.
        # ------------------------------------------------------------------
        obs = self._build_obs()
        truncated = bool(self._t >= self.episode_seconds - 1e-6)
        terminated = False
        info = self._build_info()
        info["reward_parts"] = reward_parts
        return obs, float(reward), terminated, truncated, info

    # ------------------------------------------------------------------
    # Build helpers
    # ------------------------------------------------------------------
    def _build_field(self, thermal_seed: int):
        """Build a fresh ThermalField with the given seed.

        We override the YAML's randomize_seed by writing a tiny in-memory
        override since ThermalField.from_yaml reads it directly.
        """
        with open(self.thermal_yaml, "r") as f:
            doc = yaml.safe_load(f)
        if "fire" in doc and doc["fire"] is not None:
            doc["fire"]["randomize_seed"] = int(thermal_seed)
        # Build manually mirroring ThermalField.from_yaml.
        from thermal_mapping.thermal_field import BurnProfile, FireWavefront, HotSpot
        ambient = float(doc.get("ambient", 22.0))
        spots = [HotSpot(**s) for s in (doc.get("hot_spots") or [])]
        fire = None
        fire_doc = doc.get("fire")
        if fire_doc:
            ig = list(fire_doc["ignition_point"])
            burn = BurnProfile(**fire_doc["burn"])
            arena_center = fire_doc.get("arena_center", [0.0, 0.0])
            wind = fire_doc.get("wind") or {}
            suppression = fire_doc.get("suppression") or {}
            ignition_x = float(ig[0])
            ignition_y = float(ig[1])
            wind_direction_deg = float(wind.get("direction_deg", 0.0))
            suppression_center_deg = float(
                suppression.get("center_deg", (wind_direction_deg + 180.0) % 360.0)
            )
            randomize_seed = int(fire_doc.get("randomize_seed", 0))
            if randomize_seed > 0:
                rng = np.random.default_rng(randomize_seed)
                wind_delta = float(rng.uniform(0.0, 360.0))
                wind_direction_deg = (wind_direction_deg + wind_delta) % 360.0
                suppression_center_deg = (suppression_center_deg + wind_delta) % 360.0
                jitter_r = float(fire_doc.get("ignition_jitter_radius", 0.0))
                if jitter_r > 0.0:
                    r = jitter_r * float(np.sqrt(rng.uniform(0.0, 1.0)))
                    phi = float(rng.uniform(0.0, 2.0 * np.pi))
                    ignition_x += r * float(np.cos(phi))
                    ignition_y += r * float(np.sin(phi))
            fire = FireWavefront(
                ignition_x=ignition_x,
                ignition_y=ignition_y,
                ignition_time=float(fire_doc.get("ignition_time", 0.0)),
                v_front=float(fire_doc.get("v_front", 0.01)),
                peak_temperature=float(fire_doc.get("peak_temperature", 80.0)),
                burn=burn,
                random_seed=int(fire_doc.get("random_seed", 42)),
                arena_radius=float(fire_doc.get("arena_radius", 0.0)),
                arena_center_x=float(arena_center[0]),
                arena_center_y=float(arena_center[1]),
                wind_direction_deg=wind_direction_deg,
                wind_rotation_rate_deg_per_sec=float(wind.get("rotation_rate_deg_per_sec", 0.0)),
                anisotropy=float(wind.get("anisotropy", 0.0)),
                suppression_center_deg=suppression_center_deg,
                suppression_half_angle_deg=float(suppression.get("half_angle_deg", 0.0)),
                suppression_max_radius=float(suppression.get("max_radius", 0.0)),
                suppression_edge_taper_deg=float(suppression.get("edge_taper_deg", 5.0)),
            )
        qi_scale = float((doc.get("qi") or {}).get("scale", 1.0))
        self._field = ThermalField(ambient=ambient, hot_spots=spots, fire=fire, qi_scale=qi_scale)
        self._ambient = ambient
        self._qi_scale = qi_scale

    def _build_optimizer(self, seed: int):
        sectors_cfg = list(self._cfg["sectors"])
        if self.randomize_sectors:
            perm = list(range(len(sectors_cfg)))
            self._np_random.shuffle(perm)
            self._sector_assignment = perm
        else:
            self._sector_assignment = list(range(len(sectors_cfg)))
        drones_for_opt = [
            {
                "name": sectors_cfg[s]["name"],
                "q": sectors_cfg[s]["q"],
                "c": sectors_cfg[s]["c"],
                "r_star": sectors_cfg[s]["r_star"],
                "r_max": sectors_cfg[s]["r_max"],
                "r0": 0.0,
            }
            for s in self._sector_assignment
        ]
        self._opt = OptimizerWrap(
            drones_cfg=drones_for_opt,
            budget_B=self._cfg["budget_B"],
            round_rate_hz=self._cfg["round_rate_hz"],
            T_inner=self._cfg["T_inner"],
            c1=self._cfg["c1"],
            c2=self._cfg["c2"],
            noise_bound=self._cfg["noise_bound"],
            seed=seed,
            optimizer_type=self.optimizer_type,
        )

    def _overshoot_aware_filter(
        self,
        target: np.ndarray,
        drone,
        phi_mid: float,
        phi_half: float,
        r_max: float,
    ) -> np.ndarray:
        """Pull target toward sector centerline if predicted drone position
        would exit the wedge.

        Mechanism (control-barrier-function flavour):
          1. Predict drone position at t + lookahead by simple ballistic
             integration of the current velocity. Lookahead = 0.3s ≈ one
             cascaded-PID time constant.
          2. Compute angular offset of predicted position from sector
             centerline (wrapped to [-π, π]).
          3. If the predicted offset is within ``phi_half - margin``
             (safe envelope), return the raw target unchanged.
          4. Otherwise, blend the target toward the sector centerline.
             Stronger blend the more the prediction exceeds the safe envelope.

        This is enforced in BOTH the training env and the deployment
        ``rl_planner_node`` so the policy sees identical action semantics in
        training and at runtime — no sim-to-real action-space drift.

        Disabled for point-mass dynamics: under point-mass the drone reaches
        target instantly so velocity-based overshoot prediction is moot.
        """
        if self.dynamics_model == "point_mass":
            return target

        lookahead = 0.3
        vel = drone.velocity
        pred = drone.position + vel * lookahead

        pred_phi = math.atan2(float(pred[1]), float(pred[0]))
        d_phi = ((pred_phi - phi_mid + math.pi) % (2 * math.pi)) - math.pi

        # Safe envelope: 5° margin inside the wedge boundary.
        margin = math.radians(5.0)
        safe_limit = max(0.0, phi_half - margin)
        if abs(d_phi) <= safe_limit:
            return target

        # Predicted offset exceeds safe envelope. Blend target toward
        # centerline by a factor proportional to how much we're exceeding.
        # alpha = 0 → use raw target; alpha = 1 → snap to centerline.
        excess = abs(d_phi) - safe_limit
        alpha = float(min(1.0, excess / max(math.radians(5.0), 1e-6)))

        # Decompose target in polar around sector centerline.
        t_r = float(np.linalg.norm(target))
        t_phi = math.atan2(float(target[1]), float(target[0]))
        t_d_phi = ((t_phi - phi_mid + math.pi) % (2 * math.pi)) - math.pi
        # Pull angular offset toward 0.
        new_d_phi = (1.0 - alpha) * t_d_phi
        new_phi = phi_mid + new_d_phi
        t_r = min(t_r, r_max)
        return np.array(
            [t_r * math.cos(new_phi), t_r * math.sin(new_phi)],
            dtype=np.float32,
        )

    def _spawn_drone(self):
        """Construct a fresh drone matching ``self.dynamics_model``.

        For cascaded_pid + randomize_dynamics=True, samples a per-drone
        PidGains within ``randomize_gains()``'s ±25%-of-firmware-default
        ranges. This is sim2real-standard: each episode the policy sees a
        slightly different controller, so the learned policy is robust to
        the firmware-vs-2D-model mismatch and SITL/HW manufacturing variance.
        """
        if self.dynamics_model == "point_mass":
            return PointMassDrone(
                initial_xy=(0.0, 0.0),
                altitude=self._cfg["altitude"],
                max_velocity=self._cfg["max_velocity"],
                streamer_rate_hz=self._cfg["setpoint_rate_hz"],
            )
        # cascaded_pid
        gains = (
            randomize_gains(self._np_random)
            if self.randomize_dynamics
            else PidGains.from_firmware()
        )
        # max_velocity_override in the env config sets the velocity-setpoint
        # cap on the FIRMWARE side. iter9 uses 1.0 m/s; deployment SITL ships
        # with 1.0 m/s by default (PID_POS_VEL_X_MAX), matching exactly.
        gains.vel_setpoint_max = float(self._cfg["max_velocity"])
        return CascadedPidDrone(
            initial_xy=(0.0, 0.0),
            altitude=self._cfg["altitude"],
            gains=gains,
        )

    def _build_runtime(self):
        # Drones (4), each at origin to start.
        self._drones = [self._spawn_drone() for _ in self._sector_assignment]
        # Sensors (one per drone, all sharing the same fire field).
        self._sensors = [
            ThermalSensor(
                field=self._field,
                altitude=self._cfg["altitude"],
                fov_deg=self._cfg["sensor_fov_deg"],
                resolution=16,
                noise_sigma=0.5,
                latency_seconds=0.030,
                sensor_rate_hz=5.0,
                rng=np.random.default_rng(int(self._np_random.integers(0, 2**31))),
            )
            for _ in self._sector_assignment
        ]
        # Shared map (mirrors thermal_mapper_node.py).
        self._map = ThermalMap(
            length_x=5.0,
            length_y=5.0,
            resolution=0.01,
            center_xy=(0.0, 0.0),
            sensor_footprint=2.0 * 0.6 * math.tan(math.radians(self._cfg["sensor_fov_deg"]) / 2.0),
            sensor_resolution=16,
        )

    def _estimate_q(self, slot_idx: int) -> float:
        sector_cfg = self._cfg["sectors"][self._sector_assignment[slot_idx]]
        return estimate_q_for_sector(
            field=self._field,
            t=self._t,
            phi_mid_rad=math.radians(sector_cfg["phi_mid_deg"]),
            phi_half_rad=math.radians(sector_cfg["phi_half_deg"]),
            r_outer=sector_cfg["r_max"],
            ambient=self._ambient,
            scale=self._qi_scale,
        )

    # ------------------------------------------------------------------
    # Observation + reward
    # ------------------------------------------------------------------
    def _build_obs(self) -> dict[str, np.ndarray]:
        """Default obs for our_drone (the trained policy's drone)."""
        return self._build_obs_for_drone(self._my_drone_idx)

    def build_obs_for_drone(self, drone_idx: int) -> dict[str, np.ndarray]:
        """Public alias of _build_obs_for_drone — used by eval for multi-agent runs."""
        return self._build_obs_for_drone(drone_idx)

    def _build_obs_for_drone(self, drone_idx: int) -> dict[str, np.ndarray]:
        drone = self._drones[drone_idx]
        sector_cfg = self._cfg["sectors"][self._sector_assignment[drone_idx]]
        phi_mid = math.radians(sector_cfg["phi_mid_deg"])
        phi_half = math.radians(sector_cfg["phi_half_deg"])
        r_max = sector_cfg["r_max"]
        # Policy sees the effective leash (with min_leash floor applied) so
        # its r_fraction multiplies against the same quantity the env uses.
        leash = max(float(self._opt.leashes[drone_idx]), self.min_leash)

        # Thermal sensor: read latest available (with latency).
        # Make sure we have at least one sample to avoid the cold-start zero.
        if self._sensors[drone_idx]._last_sample_t < 0.0:
            self._sensors[drone_idx].sample(drone.position, self._t)
        thermal = self._sensors[drone_idx].read(self._t)

        # Age crop (sector-wide).
        age = self._map.sector_age_crop(
            now=self._t,
            phi_mid_rad=phi_mid,
            phi_half_rad=phi_half,
            r_max=r_max,
            crop_size=self.age_crop_size,
            unobserved_age_cap=360.0,
        )

        # Vector inputs.
        x, y = float(drone.position[0]), float(drone.position[1])
        r = math.hypot(x, y)
        phi = math.atan2(y, x)
        vec = np.concatenate([
            np.array([leash], dtype=np.float32),
            np.array([r, phi], dtype=np.float32),
            drone.velocity.astype(np.float32),
            np.array([phi_mid, phi_half], dtype=np.float32),
            np.concatenate(list(self._past_actions), axis=0).astype(np.float32),
            np.array([self._t / max(self.episode_seconds, 1e-6)], dtype=np.float32),
        ])
        return {
            "thermal": thermal.astype(np.float32),
            "age": age.astype(np.float32),
            "vec": vec.astype(np.float32),
        }

    def _snapshot_fov_ages(self, xs: np.ndarray, ys: np.ndarray) -> dict[str, np.ndarray]:
        """Read pre-fusion (age, was_visited) for FOV cells. Called before update."""
        phi_mid = math.radians(self._cfg["sectors"][self._sector_assignment[self._my_drone_idx]]["phi_mid_deg"])
        phi_half = math.radians(self._cfg["sectors"][self._sector_assignment[self._my_drone_idx]]["phi_half_deg"])
        r_max = self._cfg["sectors"][self._sector_assignment[self._my_drone_idx]]["r_max"]
        cols = ((xs - (self._map.center_x - self._map.length_x / 2.0)) / self._map.resolution).astype(np.int32)
        rows = ((ys - (self._map.center_y - self._map.length_y / 2.0)) / self._map.resolution).astype(np.int32)
        in_bounds = (cols >= 0) & (cols < self._map.cols) & (rows >= 0) & (rows < self._map.rows)
        r_cells = np.sqrt(xs ** 2 + ys ** 2)
        phi_cells = np.arctan2(ys, xs)
        d_phi = np.mod(phi_cells - phi_mid + np.pi, 2 * np.pi) - np.pi
        in_sector = (np.abs(d_phi) <= phi_half) & (r_cells <= r_max)
        valid = in_bounds & in_sector
        safe_rows = np.clip(rows, 0, self._map.rows - 1)
        safe_cols = np.clip(cols, 0, self._map.cols - 1)
        lut = self._map.last_update_time[safe_rows, safe_cols]
        was_visited = ~np.isnan(lut) & valid
        # Cap age at episode_seconds for unvisited; use t - lut for visited.
        age_before = np.where(was_visited, self._t - lut, np.float32(self.episode_seconds))
        age_before = np.clip(age_before, 0.0, self.episode_seconds).astype(np.float32)
        return {
            "valid": valid,
            "was_visited": was_visited,
            "age_before": age_before,
        }

    def _compute_reward(
        self,
        action: np.ndarray,
        snapshot: dict[str, np.ndarray],
        fov_xs: np.ndarray,
        fov_ys: np.ndarray,
    ) -> tuple[float, dict[str, float]]:
        drone = self._drones[self._my_drone_idx]
        # Use effective_leash (= max(opt_leash, min_leash)) for reward terms.
        # The min_leash floor is the authorized operating range during warmup;
        # the soft leash penalty fires only when the policy tries to push
        # PAST that authorized range (slack 1.03 enables rare brief
        # overshoots during fast transients).
        opt_leash = float(self._opt.leashes[self._my_drone_idx])
        leash = max(opt_leash, self.min_leash)
        # Fix 3: compute fraction of FOV cells currently "hot" (T > T_ambient + 30°C).
        # Used by edge_pull to amplify edge_riding only when fire is at the edge.
        truth_fov = self._field.evaluate(fov_xs, fov_ys, self._t)
        hot_density = float((truth_fov > self._ambient + 30.0).mean())

        # Temperature at FOV cells at the current sim time (no noise — we're
        # using the underlying truth field for the reward signal, which is
        # the policy's "objective." Noise enters via the observation, not the
        # reward.).
        temps = self._field.evaluate(fov_xs, fov_ys, self._t)

        valid = snapshot["valid"]
        was_visited_before = snapshot["was_visited"]
        age_before = snapshot["age_before"]

        # v_before: cell priority just before our visit (post-step, the cell
        # WILL have been visited; pre-step it had age_before).
        v_before = cell_value(
            temps, age_before, was_visited_before, self._ambient,
            self.reward_ctx.weights, self.reward_ctx.candidate,
        )
        # v_after: cell priority right after our visit (age=0, was_visited=True).
        zeros = np.zeros_like(temps)
        v_after = cell_value(
            temps, zeros, np.ones_like(valid, dtype=bool), self._ambient,
            self.reward_ctx.weights, self.reward_ctx.candidate,
        )
        delta = np.where(valid, np.maximum(v_before - v_after, 0.0), 0.0)
        coverage_term = float(delta.sum())

        r_drone = math.hypot(drone.position[0], drone.position[1])
        phi_drone = math.atan2(drone.position[1], drone.position[0])
        # Sector geometry for this drone slot.
        sector_cfg = self._cfg["sectors"][self._sector_assignment[self._my_drone_idx]]
        phi_mid = math.radians(sector_cfg["phi_mid_deg"])
        phi_half = math.radians(sector_cfg["phi_half_deg"])
        # iter17: angular-coverage sweep — map the drone's φ to a sector bin, age all bins,
        # collect (and reset) the visited bin's accumulated staleness (capped). The drone
        # maximizes by sweeping the full arc, esp. the stale sector-ends → a WIDE sweep.
        nb = self._angular_bin_age.shape[0]
        d_phi = ((phi_drone - phi_mid + math.pi) % (2 * math.pi)) - math.pi  # 0 = sector center
        frac = float(np.clip((d_phi + phi_half) / max(1e-6, 2.0 * phi_half), 0.0, 0.999))
        b = int(frac * nb)
        self._angular_bin_age += self.tick_dt
        sweep_term = float(min(self._angular_bin_age[b], self.reward_ctx.weights.sweep_age_cap))
        self._angular_bin_age[b] = 0.0
        total, parts = compute_reward(
            ctx=self.reward_ctx,
            coverage_term=coverage_term,
            r_drone=r_drone,
            r_leash=leash,
            action=action,
            prev_action=self._prev_action_raw,
            hot_density=hot_density,
            phi_drone=phi_drone,
            phi_mid=phi_mid,
            phi_half=phi_half,
            t=self._t,
            sweep_term=sweep_term,
        )
        parts["hot_density"] = hot_density   # diagnostic only (not part of total)
        total = float(sum(v for k, v in parts.items() if k != "hot_density"))
        return total, parts

    def _build_info(self) -> dict[str, Any]:
        drone = self._drones[self._my_drone_idx]
        sector_cfg = self._cfg["sectors"][self._sector_assignment[self._my_drone_idx]]
        opt_leash = float(self._opt.leashes[self._my_drone_idx])
        effective_leash = max(opt_leash, self.min_leash)
        r_drone = math.hypot(drone.position[0], drone.position[1])
        return {
            "t": self._t,
            "drone_xy": drone.position.copy(),
            "r_drone": r_drone,
            "r_leash": opt_leash,
            "r_leash_effective": effective_leash,
            "edge_utilization": r_drone / max(effective_leash, 1e-6),
            "sector_name": sector_cfg["name"],
            "gate_state": self._opt.gate_state,
        }
