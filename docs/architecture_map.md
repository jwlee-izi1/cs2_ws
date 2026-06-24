# Architecture → code map (where every module lives)

**This doc is the code index.** For each part of the system it gives the *real file
path* (clickable), its `ros2 run` entry-point name, and the topics/services it consumes
and produces. The architecture *rationale* (why CrazySim, sim↔HW invariants, the diagram)
lives in [CRAZYSIM_MIGRATION.md §1](../CRAZYSIM_MIGRATION.md) — this doc does not redraw
it. Per-project status/tuning lives in the `docs/<project>.md` files.

All paths and topic contracts below were verified by reading each package's `setup.py`
and grepping the actual `create_publisher` / `create_subscription` / `create_client`
calls (not paraphrased).

> Note on the docs you may have already read: the inline Python in
> `CRAZYSIM_MIGRATION.md §1.4` (`my_mapping_node.py`, `eval_node.py`) is **illustrative
> pattern code, not real files** — the actual nodes are the files tabled below. The one
> genuine exception is the HW debug scripts in §5 of this doc.

---

## 1. The five layers

```
  RESEARCH APPS    fed_dcsa · cf_coverage_planner · thermal_mapping · rl_demo · cf_payload_world · multinash
        │                ROS 2 topics + services (same names sim & HW)
  CONTROL PLANE    crazyswarm2 → crazyflie_server   (/cfN/takeoff, go_to, odom, cmd_position, upload_trajectory …)
        │                cflib UDP (sim)                       cflib radio (HW)
  SIM BACKEND      CrazySim: cf2 firmware in docker ↔ gz_crazysim_plugin ↔ Gazebo   |   HW: real CF2.1 + Crazyradio
```

The application layer never branches on sim-vs-HW; only the `config/crazyflies_*.yaml`
and one launch arg change. See [CRAZYSIM_MIGRATION.md §1.1–1.5](../CRAZYSIM_MIGRATION.md).

---

## 2. Infrastructure layer

| Role | File / dir | Notes |
|---|---|---|
| HW control-plane server | [src/crazyswarm2/crazyflie/scripts/crazyflie_server.py](../src/crazyswarm2/crazyflie/scripts/crazyflie_server.py) | cflib backend; publishes `/cfN/odom`, `/cfN/pose`; serves takeoff/go_to/upload_trajectory/arm/land |
| Control-plane launch | [src/crazyswarm2/crazyflie/launch/launch.py](../src/crazyswarm2/crazyflie/launch/launch.py) | `backend:=cflib crazyflies_yaml_file:=…` |
| Service/msg contracts | [src/crazyswarm2/crazyflie_interfaces/](../src/crazyswarm2/crazyflie_interfaces/) | `Position`, `Takeoff`, `GoTo`, `UploadTrajectory`, `StartTrajectory`, `Arm`, … |
| Sim firmware↔Gazebo bridge | [crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp](../crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp) | UDP CRTP ↔ Gazebo physics, per drone |
| Sim launcher (1 drone) | [crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh](../crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh) | Gazebo + 1 cf2 docker container |
| Sim launcher (4 drones) | [crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh](../crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh) | Gazebo + N cf2 containers in a square |
| cf2 SITL image | [crazyflie-firmware/Dockerfile.cf2-sitl](../crazyflie-firmware/Dockerfile.cf2-sitl) | Ubuntu 22.04 (cf2 broken on 24.04 host) |
| cflib (editable) | [cflib-src/](../cflib-src/) | UDP + radio driver; reboot-safe location |
| Sim/HW configs | [config/](../config/) | `crazyflies_sitl_multi.yaml` (udp://), `crazyflies_hw.yaml` (radio://), `motion_capture.yaml` (Vicon) |
| Orchestration (untracked) | [scripts/](../scripts/) | `coverage_restart.sh`, `thermal_demo.sh`, `nuke.sh` |

---

## 3. Federated coverage (the CDC/CoRL demo) — 4 packages

The optimizer→planner→streamer→firmware circuit. Topic contracts verified from source.

### 3.1 Optimizer — `fed_dcsa`

| Role | File | `ros2 run` | Subscribes | Publishes |
|---|---|---|---|---|
| Radial coverage optimizer (FedDCSA) | [radial_coverage_node.py](../src/fed_dcsa/fed_dcsa/radial_coverage_node.py) | `fed_dcsa radial_coverage_node` | `/coverage/sector_weights` (Float64MultiArray) | `/coverage/leash` (Leash); `/demo/stopped` (Bool, latched) |
| ↳ algorithm (pure, no ROS) | [radial_coverage_algorithm.py](../src/fed_dcsa/fed_dcsa/radial_coverage_algorithm.py) | — | — | — |
| ↳ Lagrangian baseline | [lagrangian_baseline.py](../src/fed_dcsa/fed_dcsa/lagrangian_baseline.py) | — | — | — |
| Payload-tilt optimizer (separate project) | [payload_optimizer_node.py](../src/fed_dcsa/fed_dcsa/payload_optimizer_node.py) | `fed_dcsa payload_optimizer` | — | — |

### 3.2 Contract — `coverage_optimizer_interfaces`

The hot-swap boundary. Any optimizer publishes this; any planner consumes it.

- [msg/Leash.msg](../src/coverage_optimizer_interfaces/msg/Leash.msg) — `string[] drone_names`, `float32[] radii` (rᵢᵏ, metres), `uint8 gate_state` (0 = constraint-correcting, 1 = feasible).

### 3.3 Planner + streamer — `cf_coverage_planner`

Planners are **throwaway** (lawnmower → figure-8 → RL); streamers **survive** the RL swap.
Full entry-point list in [setup.py](../src/cf_coverage_planner/setup.py).

| Role | File | `ros2 run` | Subscribes | Publishes / calls |
|---|---|---|---|---|
| Planner — polar lawnmower (launch default) | [coverage_planner_node.py](../src/cf_coverage_planner/cf_coverage_planner/coverage_planner_node.py) (wraps [polar_lawnmower.py](../src/cf_coverage_planner/cf_coverage_planner/polar_lawnmower.py)) | `cf_coverage_planner coverage_planner_node` | `/coverage/leash`, `/cfN/odom` | `/cfN/policy_target` (PoseStamped), `/cfN/coverage_state_markers` |
| Planner — figure-8 (HW demo default) | [quadrant_figure8_node.py](../src/cf_coverage_planner/cf_coverage_planner/quadrant_figure8_node.py) | `cf_coverage_planner quadrant_figure8_node` | `/coverage/leash`, `/cfN/odom`, `/demo/stopped` | `/cfN/policy_target`, `/cfN/coverage_state_markers` |
| Planner — RL inference (deployment slot) | [rl_planner_node.py](../src/cf_coverage_planner/cf_coverage_planner/rl_planner_node.py) | `cf_coverage_planner rl_planner_node` | `/coverage/leash`, `/cfN/odom`, `/cfN/thermal/raw`, `/thermal_map`, `/demo/stopped` | `/cfN/policy_target`, `/cfN/coverage_state_markers` |
| Streamer — setpoint (lawnmower/fig-8 path) | [setpoint_streamer_node.py](../src/cf_coverage_planner/cf_coverage_planner/setpoint_streamer_node.py) | `cf_coverage_planner setpoint_streamer_node` | `/cfN/policy_target`, `/cfN/odom`, `/demo/stopped` | `/cfN/cmd_position` (Position) → firmware |
| Streamer — receding horizon (RL path) | [receding_horizon_streamer_node.py](../src/cf_coverage_planner/cf_coverage_planner/receding_horizon_streamer_node.py) | `cf_coverage_planner receding_horizon_streamer_node` | `/cfN/policy_target`, `/cfN/odom`, `/coverage/leash`, `/demo/stopped` | service clients `/cfN/upload_trajectory`, `/cfN/start_trajectory` |
| Other streamers (legacy/experimental) | `trajectory_streamer_node.py`, `polynomial_streamer_node.py`, `fullstate_streamer_node.py` | see [setup.py](../src/cf_coverage_planner/setup.py) | — | — |
| Shared geometry helper | [sector_geometry.py](../src/cf_coverage_planner/cf_coverage_planner/sector_geometry.py) | — | — | — |
| **Config (single source of truth)** | [config/arena_4drone.yaml](../src/cf_coverage_planner/config/arena_4drone.yaml) | — | sector geometry, optimizer/planner/streamer params | — |
| **Main launch** | [launch/coverage_demo.launch.py](../src/cf_coverage_planner/launch/coverage_demo.launch.py) | — | args: `optimizer` (fed_dcsa\|constant\|sinusoidal), `planner_type` (polar_lawnmower\|figure8\|rl), `streamer_type`, `rl_checkpoint`, `policy_warmup_sec`, `enable_thermal/takeoff/rviz` | — |

### 3.4 Mock optimizers — `baseline_optimizers`

Publish a fake `/coverage/leash` so the planner stack can be tested without the real solver.

| File | `ros2 run` | Publishes |
|---|---|---|
| [constant_leash_node.py](../src/baseline_optimizers/baseline_optimizers/constant_leash_node.py) | `baseline_optimizers constant_leash_node` | `/coverage/leash` (constant radii) |
| [sinusoidal_leash_node.py](../src/baseline_optimizers/baseline_optimizers/sinusoidal_leash_node.py) | `baseline_optimizers sinusoidal_leash_node` | `/coverage/leash` (sinusoidal radii) |

---

## 4. Thermal mapping (data plane for coverage) — `thermal_mapping` + `_interfaces`

| Role | File | `ros2 run` | Subscribes | Publishes |
|---|---|---|---|---|
| Per-drone thermal sensor | [thermal_sensor_node.py](../src/thermal_mapping/thermal_mapping/thermal_sensor_node.py) | `thermal_mapping thermal_sensor_node` | `/cfN/odom` | `/cfN/thermal/raw` (ThermalFrame, 5 Hz) |
| Shared heat-map mapper | [thermal_mapper_node.py](../src/thermal_mapping/thermal_mapping/thermal_mapper_node.py) | `thermal_mapping thermal_mapper_node` | `/cfN/thermal/raw`, `/coverage/leash` (round counter), `/cfN/odom` (constraint mode) | `/thermal_map` (GridMap: thermal/age_seconds/…), `/coverage/status_text`, `/coverage/violation_sphere` |
| q_i sector-weight estimator | [qi_estimator_node.py](../src/thermal_mapping/thermal_mapping/qi_estimator_node.py) | `thermal_mapping qi_estimator_node` | (reads fire field YAML) | `/coverage/sector_weights` (Float64MultiArray) → feeds the optimizer |
| Ground-truth field (eval/RViz) | [thermal_ground_truth_node.py](../src/thermal_mapping/thermal_mapping/thermal_ground_truth_node.py) | `thermal_mapping thermal_ground_truth_node` | — | `/thermal_truth` (GridMap) |
| Contract | [thermal_mapping_interfaces/msg/ThermalFrame.msg](../src/thermal_mapping_interfaces/msg/ThermalFrame.msg) | — | Header + Pose + width/height + `float32[] data` (16×16) | — |
| Fire-field config | [thermal_mapping/config/thermal_field.yaml](../src/thermal_mapping/config/thermal_field.yaml) | — | — | — |
| Launch | [thermal_mapping/launch/thermal_mapping_demo.launch.py](../src/thermal_mapping/launch/thermal_mapping_demo.launch.py) | — | — | — |

---

## 5. RL — `rl_demo` (training harness; deploys via `rl_planner_node`)

`rl_demo` has **no ROS node of its own** — it is the offline training/eval harness. The
trained policy is deployed by `cf_coverage_planner rl_planner_node` (§3.3).

| Role | File / dir | Notes |
|---|---|---|
| Gym environment | [rl_training/env/coverage_env.py](../src/rl_demo/rl_training/env/coverage_env.py) | obs = {thermal 16×16, age 32×32, vec 28}; action = [θ, r_frac] |
| Reward | [rl_training/env/rewards.py](../src/rl_demo/rl_training/env/rewards.py) | iter17/18 commensurate + angular-sweep terms |
| Optimizer-in-the-loop wrapper | [rl_training/env/optimizer_wrap.py](../src/rl_demo/rl_training/env/optimizer_wrap.py) | runs FedDCSA/Lagrangian in-process (imports `fed_dcsa`) |
| PPO trainer | [rl_training/training/train_ppo.py](../src/rl_demo/rl_training/training/train_ppo.py) | SB3 PPO, vectorised envs |
| Trained policies | [exp1/rl_training/](../exp1/rl_training/) | e.g. `checkpoints/ppo_coverage_iter17_final.zip` |
| Sim helper scripts | [rl_demo/scripts/](../src/rl_demo/scripts/) | `waypoint_player.py` (trajectory playback), `tune_tracking.py`, … |

---

## 6. Payload & multinash

| Role | File | Notes |
|---|---|---|
| Payload world (CrazySim) | [src/cf_payload_world/worlds/payload_world_crazysim.sdf](../src/cf_payload_world/worlds/payload_world_crazysim.sdf) | nested models, 50 g payload |
| Payload full-stack launch | [src/cf_payload_world/launch/payload_hover_crazysim.launch.py](../src/cf_payload_world/launch/payload_hover_crazysim.launch.py) | Gazebo + 4 cf2 + crazyswarm2 |
| Payload tuning log | [src/cf_payload_world/TUNING_LOG.md](../src/cf_payload_world/TUNING_LOG.md) | working config: kp_z=5, kd_z=0.3, velGain_z=10 |
| Multinash bridge | [src/multinash_cs2_bridge/multinash_cs2_bridge/multinash_bridge.py](../src/multinash_cs2_bridge/multinash_cs2_bridge/multinash_bridge.py) | in-progress; publishes `/cfN/planned_path` |

---

## 7. Federated-coverage pipeline — file at each hop

```
thermal_mapping/qi_estimator_node.py
    │  /coverage/sector_weights   (Float64MultiArray, 0.5 Hz)
    ▼
fed_dcsa/radial_coverage_node.py                          ← OPTIMIZER
    │  /coverage/leash  (Leash: radii = rᵢᵏ)              ← hot-swap contract
    ▼
cf_coverage_planner/quadrant_figure8_node.py              ← PLANNER (throwaway)
    │  (or coverage_planner_node.py / rl_planner_node.py)    swap line is HERE
    │  /cfN/policy_target  (PoseStamped, 5 Hz)
    ▼
cf_coverage_planner/setpoint_streamer_node.py             ← STREAMER (survives swap)
    │  (RL path → receding_horizon_streamer_node.py: upload_trajectory/start_trajectory)
    │  /cfN/cmd_position  (Position, 20 Hz)
    ▼
crazyswarm2 crazyflie_server  →  firmware (sim cf2 / real STM32)
```

Everything at/below `/coverage/leash` (optimizer output) and both streamers are reused
unchanged when RL replaces the lawnmower/figure-8 planner — that is the point of the
leash contract. Started by `scripts/coverage_restart.sh fed_dcsa` →
[coverage_demo.launch.py](../src/cf_coverage_planner/launch/coverage_demo.launch.py).

---

## 8. ⚠️ Scripts that live ONLY in docs (no repo file)

These HW-debug tools have **no file in the repo** — they exist solely as inline code in
[hw_drone_verification.md](hw_drone_verification.md) and are hand-copied into `/tmp` on
demand, which `/tmp` wipes on reboot. (This is why `vicon_probe.py` was missing and had
to be recreated at the start of a recent session.) If you need one, copy it out of the
doc section listed:

| Script | Source (inline) | What it does |
|---|---|---|
| `/tmp/vicon_probe.py` | [hw_drone_verification.md §5.3](hw_drone_verification.md) | one-shot read of Vicon **unlabeled** markers (count + xyz) |
| `/tmp/vicon_monitor.py` | [hw_drone_verification.md §5.2](hw_drone_verification.md) | continuous unlabeled-marker monitor (dropout detection) |
| `/tmp/flight_logger.py` | [hw_drone_verification.md §5.1](hw_drone_verification.md) | time-aligns `/poses` (mocap) vs `/cfN/odom` (firmware EKF) |

Persisting these into [scripts/](../scripts/) (so they survive reboots) is a known
follow-up, deliberately left out of this map's scope.
