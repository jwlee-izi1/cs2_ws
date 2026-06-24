"""RL deployment planner — loads a SB3 PPO checkpoint and publishes
``/cfN/policy_target`` at 5 Hz. Drop-in replacement for
``coverage_planner_node`` / ``quadrant_figure8_node``.

Subscriptions:
  - /coverage/leash (coverage_optimizer_interfaces/Leash)
  - /cfN/odom (nav_msgs/Odometry)
  - /cfN/thermal/raw (thermal_mapping_interfaces/ThermalFrame)
  - /thermal_map (grid_map_msgs/GridMap) — layer ``age_seconds``
Publishes:
  - /cfN/policy_target (geometry_msgs/PoseStamped)

The observation built here MUST match the training env's
``CoverageEnv._build_obs_for_drone`` (thermal 16×16 + age 32×32 +
28-vector). The action mapping uses the same sector-polar + min_leash
floor + slack as the env.
"""

from __future__ import annotations

# ----------------------------------------------------------------------
# IMPORTANT: prepend the conda/SB3 site-packages dir BEFORE numpy/rclpy
# are imported. ROS sources /usr/lib/python3/dist-packages first, which
# pins numpy to 1.26 and breaks loading SB3 checkpoints saved against
# numpy 2.x ("No module named 'numpy._core.numeric'"). We can't undo
# that once numpy is cached, so we must adjust sys.path at top-of-module.
# Path candidates are tried in order; the first one with a numpy>=2 wins.
# ----------------------------------------------------------------------
import os as _os
import sys as _sys


def _hoist_modern_python_libs():
    candidates = []
    env_override = _os.environ.get('RL_PLANNER_SITE_PACKAGES', '').strip()
    if env_override:
        candidates.append(env_override)
    conda_prefix = _os.environ.get('CONDA_PREFIX', '').strip()
    if conda_prefix:
        candidates.append(_os.path.join(conda_prefix, 'lib', 'python3.12', 'site-packages'))
    candidates.append('/home/rk32226/miniconda3/envs/crazyflie/lib/python3.12/site-packages')
    candidates.append(_os.path.expanduser('~/.local/lib/python3.12/site-packages'))
    for c in candidates:
        if not _os.path.isdir(c):
            continue
        numpy_init = _os.path.join(c, 'numpy', '__init__.py')
        if not _os.path.isfile(numpy_init):
            continue
        # Prefer numpy>=2 (has _core submodule).
        if not _os.path.isdir(_os.path.join(c, 'numpy', '_core')):
            continue
        if c not in _sys.path:
            _sys.path.insert(0, c)
        return c
    return None


_HOISTED_SITE_PACKAGES = _hoist_modern_python_libs()

import math
import os
import sys
from collections import deque

import numpy as np
import rclpy
from coverage_optimizer_interfaces.msg import Leash
from geometry_msgs.msg import PoseStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool
from thermal_mapping_interfaces.msg import ThermalFrame
from visualization_msgs.msg import Marker, MarkerArray


class RLPlannerNode(Node):

    def __init__(self):
        super().__init__('rl_planner')
        # ---- Parameters (mirror coverage_planner_node + RL-specific) ----
        self.declare_parameter('drone_name', 'cf1')
        self.declare_parameter('center_xy', [0.0, 0.0])
        self.declare_parameter('altitude', 0.6)
        self.declare_parameter('phi_mid_deg', 0.0)
        self.declare_parameter('phi_half_deg', 45.0)
        self.declare_parameter('r_max', 1.9)
        self.declare_parameter('min_leash', 0.4)
        self.declare_parameter('action_slack', 1.0)
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('planner_rate_hz', 5.0)
        self.declare_parameter('episode_seconds', 360.0)
        self.declare_parameter('age_crop_size', 32)
        self.declare_parameter('past_action_history', 10)
        self.declare_parameter('device', 'cpu')
        # Directory containing the ``rl_training`` package. The SB3 zip pickles
        # ``rl_training.training.feature_extractor.CoverageFeatureExtractor`` by
        # reference, so unpickling needs ``rl_training`` importable.
        # Default resolves to ``<cs2_ws>/src/rl_demo`` from the install path of
        # this node's package (install/cf_coverage_planner/lib/python3.X/site-packages/...).
        self.declare_parameter('rl_training_src_dir', '')
        # Optional site-packages dir to inject (for SB3/gymnasium/torch when ROS
        # runs against system Python but those libs live in a conda env).
        # Empty string -> auto-detect from $CONDA_PREFIX, then a known fallback.
        self.declare_parameter('python_site_packages', '')

        self.drone_name = str(self.get_parameter('drone_name').value)
        cx_param = self.get_parameter('center_xy').value
        self.center_xy = (float(cx_param[0]), float(cx_param[1]))
        self.altitude = float(self.get_parameter('altitude').value)
        self.phi_mid = math.radians(float(self.get_parameter('phi_mid_deg').value))
        self.phi_half = math.radians(float(self.get_parameter('phi_half_deg').value))
        self.r_max = float(self.get_parameter('r_max').value)
        self.min_leash = float(self.get_parameter('min_leash').value)
        self.action_slack = float(self.get_parameter('action_slack').value)
        self.episode_seconds = float(self.get_parameter('episode_seconds').value)
        self.age_crop_size = int(self.get_parameter('age_crop_size').value)
        self.past_n = int(self.get_parameter('past_action_history').value)
        # Airborne gate: don't publish policy_target until the drone has at
        # least this altitude (matches takeoff completion check). Without
        # this, the streamer fights the high-level takeoff during the first
        # ~3 seconds and drones never reach hover altitude.
        self.declare_parameter('airborne_z_threshold', 0.3)
        self.airborne_z = float(self.get_parameter('airborne_z_threshold').value)
        # Path to a per-drone debug log file (empty = stdout/INFO logger only).
        self.declare_parameter('debug_log_path', '')
        log_path = str(self.get_parameter('debug_log_path').value)
        self._dbg_file = open(log_path, 'w') if log_path else None
        if self._dbg_file:
            self._dbg_file.write(
                f"# rl_planner_node debug log for drone={self.get_parameter('drone_name').value}\n"
                f"# columns: t,phase,event,key=value...\n"
            )
            self._dbg_file.flush()
        self._last_debug_log_t = -1.0

        # ---- Load policy (lazy import — node fails loud if SB3 missing) ----
        checkpoint_path = str(self.get_parameter('checkpoint_path').value)
        if not checkpoint_path:
            raise RuntimeError(
                "RLPlannerNode: 'checkpoint_path' must be set (path to ppo *.zip)")
        device = str(self.get_parameter('device').value)
        self._load_policy(checkpoint_path, device)

        # ---- Topic state ----
        self.latest_odom: Odometry | None = None
        self.latest_leash: float | None = None
        self.latest_thermal: np.ndarray | None = None  # (16, 16) float32
        self.latest_age_layer: np.ndarray | None = None  # (rows, cols) float32
        self.latest_grid_map: GridMap | None = None

        # ---- History (matches env state) ----
        self.past_actions: deque = deque(maxlen=self.past_n)
        for _ in range(self.past_n):
            self.past_actions.append(np.zeros(2, dtype=np.float32))
        # prev_position stays None until the first tick reads odom, so the
        # initial velocity matches the env's drone.velocity == (0, 0) at reset.
        self.prev_position: np.ndarray | None = None
        self.start_time = None
        self.last_tick_elapsed = None

        # ---- Subscriptions ----
        self.create_subscription(Leash, '/coverage/leash', self._leash_cb, 10)
        # 2026-06-05: odom was RELIABLE depth-10 -> the high-rate Gazebo-bridge /odom flooded
        # the executor with a backlog to deserialize (GIL), starving the 5Hz tick to ~0.9Hz.
        # The figure-8 path doesn't subscribe odom, which is why it ran at full rate. Use
        # best-effort keep-last-1 so only the LATEST odom is ever processed.
        self.create_subscription(
            Odometry, f'/{self.drone_name}/odom', self._odom_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            ThermalFrame, f'/{self.drone_name}/thermal/raw', self._thermal_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(GridMap, '/thermal_map', self._gridmap_cb, 10)
        # /demo/stopped (latched): when True, stop publishing policy_target so the
        # optimizer's land command isn't fought (demo freezes). See radial_coverage_node.
        self._demo_stopped = False
        self.create_subscription(
            Bool, '/demo/stopped', self._demo_stopped_cb,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        # ---- Publisher + tick timer ----
        self.target_pub = self.create_publisher(
            PoseStamped, f'/{self.drone_name}/policy_target', 10)
        # Leash-arc marker (LINE_STRIP across the sector wedge at r=leash) so
        # RViz can show the optimizer's allocation. Identical message format
        # to coverage_planner_node's marker for compatibility with the
        # existing coverage.rviz config.
        self.marker_pub = self.create_publisher(
            MarkerArray, f'/{self.drone_name}/coverage_state_markers', 10)
        rate_hz = float(self.get_parameter('planner_rate_hz').value)
        # 2026-06-05: put the tick timer in its OWN callback group + run a
        # MultiThreadedExecutor (see main) so the 5 Hz tick is NOT serialized behind the
        # heavy /thermal_map GridMap parse (_gridmap_cb). Diagnosed: the tick itself is ~2ms
        # but ran at 0.83 Hz because the single-threaded executor was blocked ~1.16s/tick by
        # the GridMap callback -> undersampled sweep + cf2 couldn't track the leash-shrink.
        self._timer_cbg = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / rate_hz, self._tick, callback_group=self._timer_cbg)

        self.get_logger().info(
            f'RLPlanner({self.drone_name}): checkpoint={checkpoint_path}, '
            f'phi_mid={math.degrees(self.phi_mid):.0f}deg, '
            f'phi_half={math.degrees(self.phi_half):.0f}deg, '
            f'r_max={self.r_max}, min_leash={self.min_leash}, '
            f'slack={self.action_slack}, device={device}'
        )

    # ------------------------------------------------------------------
    # Policy loading
    # ------------------------------------------------------------------
    def _load_policy(self, checkpoint_path: str, device: str):
        self._ensure_python_site_packages_on_path()
        self._ensure_rl_training_on_path()
        # 2026-06-05: pin torch to 1 CPU thread. We run ONE rl_planner process PER drone
        # (4 total); each torch defaults to grabbing ALL cores -> 4*Ncores thread
        # oversubscription -> thrashing -> the airborne tick took ~1.16s (0.86 Hz instead of
        # 5 Hz), which undersampled the sweep and starved the controller (cf2 couldn't track
        # the leash-shrink -> diverged -> crash cascade). 1 thread/process removes the
        # contention; inference is tiny so single-threaded is plenty at 5 Hz.
        import torch
        torch.set_num_threads(1)
        from stable_baselines3 import PPO
        self.policy = PPO.load(checkpoint_path, device=device)

    def _ensure_python_site_packages_on_path(self):
        """Make sb3/gym/torch importable. The top-of-module hoist already
        prepends a modern site-packages dir; this handles the explicit
        parameter override case (rare) and reports the result."""
        try:
            import stable_baselines3  # noqa: F401
            if _HOISTED_SITE_PACKAGES:
                self.get_logger().info(
                    f'RLPlanner: SB3 ready (hoisted {_HOISTED_SITE_PACKAGES})')
            return
        except ImportError:
            pass
        override = str(self.get_parameter('python_site_packages').value).strip()
        if override and os.path.isdir(os.path.join(override, 'stable_baselines3')):
            sys.path.insert(0, override)
            self.get_logger().info(f'RLPlanner: added {override} to sys.path for SB3')
            return
        raise RuntimeError(
            'RLPlanner: stable_baselines3 not importable. Either pip-install '
            'into the active Python, set RL_PLANNER_SITE_PACKAGES env var, or '
            'set "python_site_packages" parameter to the dir that contains it.')

    def _ensure_rl_training_on_path(self):
        """Make ``rl_training.*`` importable before SB3 unpickles the policy."""
        try:
            import rl_training  # noqa: F401
            return
        except ImportError:
            pass
        # Honor explicit override first.
        override = str(self.get_parameter('rl_training_src_dir').value).strip()
        if override:
            candidates = [override]
        else:
            # Auto-resolve: ascend to the cs2_ws root and look for src/rl_demo.
            here = os.path.dirname(os.path.abspath(__file__))
            cur = here
            candidates = []
            for _ in range(8):
                cur = os.path.dirname(cur)
                guess = os.path.join(cur, 'src', 'rl_demo')
                if os.path.isdir(os.path.join(guess, 'rl_training')):
                    candidates.append(guess)
                    break
            # Last-resort hardcoded fallback to the user's workspace.
            candidates.append('/home/rk32226/cs2_ws/src/rl_demo')
        for c in candidates:
            if os.path.isdir(os.path.join(c, 'rl_training')) and c not in sys.path:
                sys.path.insert(0, c)
                self.get_logger().info(f'RLPlanner: added {c} to sys.path for rl_training import')
                return
        raise RuntimeError(
            'RLPlanner: could not locate rl_training package. Set '
            "'rl_training_src_dir' parameter to the directory containing it "
            '(e.g. /home/rk32226/cs2_ws/src/rl_demo).')

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
    def _leash_cb(self, msg: Leash):
        try:
            idx = list(msg.drone_names).index(self.drone_name)
        except ValueError:
            return
        self.latest_leash = float(msg.radii[idx])

    def _odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def _thermal_cb(self, msg: ThermalFrame):
        if msg.width == 0 or msg.height == 0:
            return
        # 2026-06-05: throttle the parse to ~5Hz (the tick rate). Parsing every frame held
        # the Python GIL and starved the timer thread -> 0.88Hz tick. The policy only needs
        # the latest at tick time, so a ~5Hz parse is plenty.
        import time as _tt
        if (_tt.perf_counter() - getattr(self, '_last_thermal_parse', 0.0)) < 0.18:
            return
        self._last_thermal_parse = _tt.perf_counter()
        try:
            data = np.array(msg.data, dtype=np.float32).reshape(
                int(msg.height), int(msg.width))
        except ValueError:
            return
        self.latest_thermal = data

    def _gridmap_cb(self, msg: GridMap):
        # 2026-06-05: throttle the GridMap parse to ~5Hz. This (global /thermal_map, parsed
        # by all 4 planners) was the main GIL hog starving the timer thread (0.88Hz tick).
        import time as _tg
        if (_tg.perf_counter() - getattr(self, '_last_gm_parse', 0.0)) < 0.18:
            return
        self._last_gm_parse = _tg.perf_counter()
        try:
            idx = list(msg.layers).index('age_seconds')
        except ValueError:
            return
        info = msg.info
        try:
            resolution = float(info.resolution)
            length_x = float(info.length_x)
            length_y = float(info.length_y)
            # MATCH PUBLISHER CONVENTION (grid_map_helpers.grid_shape):
            #   shape  = (n_rows, n_cols) = (length_x/res, length_y/res)
            #   flat   = F-order (column-major) ravel
            #   row    = floor((cx + length_x/2 - x) / res)   (mirrored x)
            #   col    = floor((cy + length_y/2 - y) / res)   (mirrored y)
            n_rows = int(round(length_x / resolution))
            n_cols = int(round(length_y / resolution))
            flat = np.array(msg.data[idx].data, dtype=np.float32)
            if flat.size != n_rows * n_cols:
                return
            self.latest_age_layer = flat.reshape((n_rows, n_cols), order='F')
            self.latest_grid_map = msg
        except (ValueError, ZeroDivisionError):
            return

    # ------------------------------------------------------------------
    # Tick: build obs → predict → publish
    # ------------------------------------------------------------------
    def _demo_stopped_cb(self, msg: Bool):
        if msg.data and not self._demo_stopped:
            self._demo_stopped = True
            self.get_logger().info(
                '/demo/stopped=True → halting policy_target (demo over)')

    def _tick(self):
        if self._demo_stopped:
            return
        if self.latest_odom is None or self.latest_leash is None:
            self._dbg('waiting',
                      odom=None if self.latest_odom is None else 'yes',
                      leash=None if self.latest_leash is None else self.latest_leash)
            return

        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        t_elapsed = (now - self.start_time).nanoseconds * 1e-9

        # Airborne gate: don't publish setpoints until drone has lifted off.
        # While grounded, keep the prev_position fresh so the first publishing
        # tick sees a sane (near-zero) velocity.
        oz = float(self.latest_odom.pose.pose.position.z)
        if oz < self.airborne_z:
            self.prev_position = np.array([
                self.latest_odom.pose.pose.position.x,
                self.latest_odom.pose.pose.position.y,
            ], dtype=np.float32)
            self.last_tick_elapsed = t_elapsed
            self._dbg('grounded', z=oz, t=t_elapsed)
            return

        import time as _time
        self._t_obs0 = _time.perf_counter()
        obs = self._build_obs(t_elapsed)
        self._t_obs1 = _time.perf_counter()
        # SB3 expects batched dict
        batched = {k: np.expand_dims(v, 0) for k, v in obs.items()}
        action, _ = self.policy.predict(batched, deterministic=True)
        self._t_pred1 = _time.perf_counter()
        action = action[0]
        theta_raw = float(action[0])
        r_raw = float(action[1])
        # iter14 HYBRID: the trained iter12 policy drives the angle (θ); the
        # radius is hardcoded (NOT the policy's r_raw). Mirrors the 2D hybrid
        # expert (viz_expert_gif.make_hybrid_expert: PPO θ + r_frac lock at 0.99).
        # r_frac math: fed_dcsa keeps Σ_leash ≤ B=8.0 → Σ_drone ≈ 7.84 (under
        # B_op=8.15); Lagrangian leash transients hit Σ_leash ≈ 8.78 → Σ_drone
        # ≈ 8.60 (over B_op, sustained) → visible drops during the wind-handoff.
        # SWEEP_CLIP: limit the policy's θ to a fraction of the wedge. Originally 0.75
        # (the bare minimum clearing the ~0.30 m BVC margin) because BVC was BLIND —
        # at full sweep adjacent drones converged to ~0.13 m. BVC is now actually fed
        # peer positions (server packed-extpos broadcast + cf2-sitl firmware rebuilt),
        # so it holds the near-boundary close approaches and we can widen the sweep.
        # Widened 0.75 → 0.8 (±36° of ±45°). 0.9 gave more sweep but pushed the streamer
        # radial overshoot to max r 2.1–2.35 m (>1.9); 0.8 keeps most of the sweep gain
        # with less overshoot. See CRAZYSIM_MIGRATION.md (BVC section).
        SWEEP_CLIP = 0.8   # run-1 baseline (reverted 2026-06-04 from 0.5)
        theta = max(-SWEEP_CLIP, min(SWEEP_CLIP, theta_raw))  # policy θ, clipped
        r_frac = 0.99              # hybrid — hardcoded radius (NOT policy r_raw)

        effective_leash = max(self.latest_leash, self.min_leash)
        target_r = min(r_frac * effective_leash, self.r_max)
        # Sideways (linear back-and-forth) motion perpendicular to sector
        # centerline. Drone patrols a straight LINE at the leash radius
        # rather than tracing an arc. The "theta" variable (clamped +
        # sweep) drives lateral offset; the drone never leaves the wedge
        # because lateral amplitude is geometrically bounded.
        # Reuse target_phi for logging/marker but compute tx,ty differently.
        target_phi = self.phi_mid + theta * self.phi_half  # log/marker only

        ox = float(self.latest_odom.pose.pose.position.x)
        oy = float(self.latest_odom.pose.pose.position.y)

        # Overshoot-aware action filter (MIRROR of CoverageEnv._overshoot_aware_filter).
        # Predict the drone's position at t + lookahead under current velocity;
        # if the predicted position would exit the wedge, blend the target's
        # angular component toward the sector centerline. lookahead = 0.3s ≈
        # one cascaded-PID time constant for xy position settle.
        lookahead = 0.3
        # Approx velocity from finite-diff vs prev_position (only set after
        # first publishing tick — fall back to 0 otherwise).
        if self.prev_position is not None and self.last_tick_elapsed is not None:
            dt_v = max(0.05, t_elapsed - self.last_tick_elapsed)
            vx_for_pred = (ox - float(self.prev_position[0])) / dt_v
            vy_for_pred = (oy - float(self.prev_position[1])) / dt_v
        else:
            vx_for_pred = vy_for_pred = 0.0
        pred_x = ox + vx_for_pred * lookahead
        pred_y = oy + vy_for_pred * lookahead
        pred_phi = math.atan2(pred_y - self.center_xy[1],
                              pred_x - self.center_xy[0])
        pred_d_phi = ((pred_phi - self.phi_mid + math.pi) % (2 * math.pi)) - math.pi
        margin = math.radians(5.0)
        safe_limit = max(0.0, self.phi_half - margin)
        filter_active = abs(pred_d_phi) > safe_limit
        if filter_active:
            excess = abs(pred_d_phi) - safe_limit
            alpha = float(min(1.0, excess / max(math.radians(5.0), 1e-6)))
            # Pull the angular offset of the TARGET toward 0.
            t_d_phi = ((target_phi - self.phi_mid + math.pi) % (2 * math.pi)) - math.pi
            new_d_phi = (1.0 - alpha) * t_d_phi
            target_phi = self.phi_mid + new_d_phi

        # Sector-rescue (kept as a backstop): if the drone is ALREADY well
        # outside the wedge, the filter alone may not suffice. Snap target to
        # centerline. Triggers only when drone has crossed boundary by 5°+.
        phi_drone = math.atan2(oy - self.center_xy[1], ox - self.center_xy[0])
        d_phi = ((phi_drone - self.phi_mid + math.pi) % (2 * math.pi)) - math.pi
        rescue_active = abs(d_phi) > (self.phi_half + math.radians(5.0))
        if rescue_active:
            target_phi = self.phi_mid
            target_r = max(self.min_leash * 0.8, min(target_r, effective_leash))

        # Arc sweep — drone traces along the leash circumference from one
        # side of its wedge to the other. Standard polar projection.
        tx = self.center_xy[0] + target_r * math.cos(target_phi)
        ty = self.center_xy[1] + target_r * math.sin(target_phi)

        out = PoseStamped()
        out.header.stamp = now.to_msg()
        out.header.frame_id = 'world'
        out.pose.position.x = float(tx)
        out.pose.position.y = float(ty)
        out.pose.position.z = float(self.altitude)
        out.pose.orientation.w = 1.0
        self.target_pub.publish(out)

        # Publish leash arc marker for RViz (matches coverage_planner_node).
        self.marker_pub.publish(self._make_leash_marker(now, effective_leash))

        # Throttled (~1Hz) debug log: full obs scalars, action, target, odom.
        if t_elapsed - self._last_debug_log_t > 1.0:
            ox = float(self.latest_odom.pose.pose.position.x)
            oy = float(self.latest_odom.pose.pose.position.y)
            r = math.hypot(ox, oy)
            phi = math.degrees(math.atan2(oy, ox))
            import time as _tmod
            self._dbg('publishing' if not rescue_active else 'rescue', t=t_elapsed,
                      obs_ms=round((self._t_obs1 - self._t_obs0) * 1e3, 1),
                      pred_ms=round((self._t_pred1 - self._t_obs1) * 1e3, 1),
                      tot_ms=round((_tmod.perf_counter() - self._t_obs0) * 1e3, 1),
                      odom=(ox, oy, oz), r=r, phi_deg=phi,
                      leash=self.latest_leash, eff_leash=effective_leash,
                      vec_leash=float(obs['vec'][0]), vec_r=float(obs['vec'][1]),
                      vec_phi=float(obs['vec'][2]), vec_vx=float(obs['vec'][3]),
                      vec_vy=float(obs['vec'][4]),
                      age_mean=float(obs['age'][obs['age'] > 0].mean()) if (obs['age'] > 0).any() else 0.0,
                      thermal_max=float(obs['thermal'].max()),
                      action_raw=(theta_raw, r_raw),
                      action_clipped=(theta, r_frac),
                      target=(tx, ty), target_r=target_r,
                      target_phi_deg=math.degrees(target_phi),
                      d_phi_deg=math.degrees(d_phi))
            self._last_debug_log_t = t_elapsed

        # History update
        self.past_actions.append(np.array([theta, r_frac], dtype=np.float32))
        self.prev_position = np.array([
            self.latest_odom.pose.pose.position.x,
            self.latest_odom.pose.pose.position.y,
        ], dtype=np.float32)
        self.last_tick_elapsed = t_elapsed

    # ------------------------------------------------------------------
    # Leash arc marker for RViz
    # ------------------------------------------------------------------
    def _make_leash_marker(self, now, leash):
        from geometry_msgs.msg import Point
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = now.to_msg()
        m.ns = f'{self.drone_name}_leash_arc'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.a = 0.8
        # Color by drone — fixed per-drone for visual distinction.
        colors = {'cf1': (0.9, 0.2, 0.2), 'cf2': (0.2, 0.9, 0.2),
                  'cf3': (0.2, 0.4, 0.9), 'cf4': (0.9, 0.6, 0.2)}
        r_, g_, b_ = colors.get(self.drone_name, (0.7, 0.7, 0.7))
        m.color.r, m.color.g, m.color.b = r_, g_, b_
        n = 32
        for k in range(n + 1):
            phi = (self.phi_mid - self.phi_half +
                   2.0 * self.phi_half * k / n)
            p = Point()
            p.x = float(self.center_xy[0] + leash * math.cos(phi))
            p.y = float(self.center_xy[1] + leash * math.sin(phi))
            p.z = float(self.altitude)
            m.points.append(p)
        arr.markers.append(m)
        return arr

    # ------------------------------------------------------------------
    # Debug logging helper
    # ------------------------------------------------------------------
    def _dbg(self, event: str, **kv):
        if self._dbg_file is None:
            return
        t_str = ''
        if self.start_time is not None:
            t_now = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
            t_str = f"{t_now:.3f}"
        kvs = ' '.join(f"{k}={v}" for k, v in kv.items())
        self._dbg_file.write(f"{t_str},{event},{kvs}\n")
        self._dbg_file.flush()

    # ------------------------------------------------------------------
    # Observation builder — must mirror CoverageEnv._build_obs_for_drone
    # ------------------------------------------------------------------
    def _build_obs(self, t_elapsed: float) -> dict[str, np.ndarray]:
        # Thermal: 16×16 sensor reading
        if self.latest_thermal is None:
            thermal = np.full((16, 16), 22.0, dtype=np.float32)  # ambient
        elif self.latest_thermal.shape != (16, 16):
            thermal = self._resize_to_16x16(self.latest_thermal)
        else:
            thermal = self.latest_thermal.astype(np.float32)

        # Age crop: 32×32 sector-wide downsample from fused map
        if self.latest_age_layer is None or self.latest_grid_map is None:
            age = np.full((self.age_crop_size, self.age_crop_size), 360.0, dtype=np.float32)
        else:
            age = self._extract_age_crop()

        # Vector inputs (28 scalars)
        ox = float(self.latest_odom.pose.pose.position.x)
        oy = float(self.latest_odom.pose.pose.position.y)
        r = math.hypot(ox - self.center_xy[0], oy - self.center_xy[1])
        phi = math.atan2(oy - self.center_xy[1], ox - self.center_xy[0])
        # Velocity via finite difference at policy tick rate (~0.2s). On the
        # very first tick, prev_position is None — return (0, 0) so we match
        # the env's drone.velocity == (0, 0) at reset and don't feed the
        # policy a transient takeoff-jump velocity (which would be out of
        # distribution).
        if self.prev_position is None:
            vx = vy = 0.0
        else:
            dt = 0.2 if self.last_tick_elapsed is None else max(0.05, t_elapsed - self.last_tick_elapsed)
            vx = (ox - self.prev_position[0]) / dt
            vy = (oy - self.prev_position[1]) / dt
        effective_leash = max(float(self.latest_leash), self.min_leash)

        vec = np.concatenate([
            np.array([effective_leash], dtype=np.float32),
            np.array([r, phi], dtype=np.float32),
            np.array([vx, vy], dtype=np.float32),
            np.array([self.phi_mid, self.phi_half], dtype=np.float32),
            np.concatenate(list(self.past_actions), axis=0).astype(np.float32),
            np.array([t_elapsed / max(self.episode_seconds, 1e-6)], dtype=np.float32),
        ])
        return {
            'thermal': thermal.astype(np.float32),
            'age': age.astype(np.float32),
            'vec': vec.astype(np.float32),
        }

    def _resize_to_16x16(self, arr: np.ndarray) -> np.ndarray:
        """Pad / center-crop to (16, 16). Same fallback if upstream sensor uses other size."""
        H, W = arr.shape
        out = np.full((16, 16), 22.0, dtype=np.float32)
        h = min(H, 16)
        w = min(W, 16)
        sy = (H - h) // 2
        sx = (W - w) // 2
        ty = (16 - h) // 2
        tx = (16 - w) // 2
        out[ty:ty + h, tx:tx + w] = arr[sy:sy + h, sx:sx + w]
        return out

    def _extract_age_crop(self) -> np.ndarray:
        """Sample the published age_seconds layer at the same world coordinates
        as the env's ThermalMap.sector_age_crop, then mask outside-wedge cells.

        The deployment publisher (grid_map_helpers) stores arr[row, col] with
        row = floor((cx + Lx/2 - x)/res) and col = floor((cy + Ly/2 - y)/res)
        — both axes MIRRORED, and array shape = (Lx/res, Ly/res). This is the
        opposite of the env's ThermalMap convention, so we must use the
        publisher's formula when sampling.
        """
        age_full = self.latest_age_layer
        info = self.latest_grid_map.info
        resolution = float(info.resolution)
        cx_world = float(info.pose.position.x)
        cy_world = float(info.pose.position.y)
        length_x = float(info.length_x)
        length_y = float(info.length_y)
        n_rows, n_cols = age_full.shape

        # Sector wedge bounding box in world frame (matches env)
        n_sample = 32
        angles = np.linspace(self.phi_mid - self.phi_half, self.phi_mid + self.phi_half, n_sample)
        bx = self.r_max * np.cos(angles)
        by = self.r_max * np.sin(angles)
        x_lo = float(min(0.0, bx.min()))
        x_hi = float(max(0.0, bx.max()))
        y_lo = float(min(0.0, by.min()))
        y_hi = float(max(0.0, by.max()))
        pad = 0.02
        x_lo -= pad; x_hi += pad; y_lo -= pad; y_hi += pad

        xs = np.linspace(x_lo, x_hi, self.age_crop_size, dtype=np.float32)
        ys = np.linspace(y_lo, y_hi, self.age_crop_size, dtype=np.float32)
        gx, gy = np.meshgrid(xs, ys, indexing='xy')

        # Publisher's mirrored world→grid (matches grid_map_helpers.world_to_rowcol).
        row_idx = np.floor((cx_world + length_x * 0.5 - gx) / resolution).astype(np.int32)
        col_idx = np.floor((cy_world + length_y * 0.5 - gy) / resolution).astype(np.int32)
        in_bounds = (
            (row_idx >= 0) & (row_idx < n_rows) &
            (col_idx >= 0) & (col_idx < n_cols)
        )
        safe_rows = np.clip(row_idx, 0, n_rows - 1)
        safe_cols = np.clip(col_idx, 0, n_cols - 1)
        gathered = age_full[safe_rows, safe_cols]
        cap = 360.0
        observed = ~np.isnan(gathered) & in_bounds
        crop = np.where(observed, np.minimum(gathered, cap), cap).astype(np.float32)

        # Mask outside-wedge cells (identical to env when center_xy=(0,0))
        r_cells = np.sqrt(gx ** 2 + gy ** 2)
        phi_cells = np.arctan2(gy, gx)
        d_phi = np.mod(phi_cells - self.phi_mid + np.pi, 2 * np.pi) - np.pi
        in_wedge = (np.abs(d_phi) <= self.phi_half) & (r_cells <= self.r_max)
        crop = np.where(in_wedge, crop, 0.0).astype(np.float32)
        return crop


def main(args=None):
    rclpy.init(args=args)
    node = RLPlannerNode()
    # 2026-06-05: EventsExecutor (C++-backed, event-driven). The default Single/Multi-Threaded
    # executors REBUILD the entire wait set (ExitStack over every entity) on EVERY spin
    # iteration; in the full sim's busy DDS graph (16 bridged topics + flood traffic) that
    # per-iteration rebuild crowds out the 5 Hz timer deadline -> the tick collapses to ~0.9 Hz
    # (rclpy #1452/#1223/#1389; MultiThreaded makes it WORSE, #1223). EventsExecutor does NOT
    # rebuild a wait set per iteration -> the timer holds 5 Hz. Verified importable in Jazzy.
    executor = EventsExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
