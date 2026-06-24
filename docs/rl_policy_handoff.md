# RL Coverage Policy — Server Training & Eval Handoff

This doc covers everything the next chat (or you, on the server) needs to
train **iter10**, deploy it in crazysim, and ship it for the CDC 2026 paper.

---

## Status snapshot (2026-05-28)

| Item | Status | Notes |
|------|--------|-------|
| Pure-Python env (cascaded PID + domain randomization) | ✅ done | `src/rl_demo/rl_training/env/dynamics.py` |
| Obs builder bit-parity (env vs ROS rl_planner_node) | ✅ verified | `/tmp/test_full_obs_parity.py` |
| ROS deployment node (`rl_planner_node`) | ✅ done | airborne gate, sector-rescue, debug logs |
| Cardinal-spawn (cf1 East, cf2 North, ... matching arena sectors) | ✅ done | `sitl_multiagent_square.sh` + `coverage_restart.sh` |
| iter10 sanity training (50k steps on cascaded PID) | ✅ done | OOS dropped 95% → 4.7%, **policy will work** |
| iter10 full training (1-2M steps) | ⏳ TODO on server | this doc |
| Crazysim transfer test of iter10 | ⏳ TODO after training | uses existing `coverage_restart.sh fed_dcsa rl` |

---

## Key change vs iter9: cascaded PID training dynamics

iter1–9 trained on `PointMassDrone` (instantaneous position-tracking, no
inertia). When deployed against the crazyflie firmware's cascaded PID
controller (`stabilizer.controller = 1`, default in cf2-sitl and on HW),
**iter9 spent 95% of timesteps out of its assigned sector** because it had
never seen overshoot dynamics during training.

iter10 trains on `CascadedPidDrone`, a 2D approximation of the firmware's
exact cascade:

```
target  → [position P, sat=±1m/s]  → velocity_setpoint
                                     ↓
        [velocity PI, deg per m/s]  → tilt angle (clipped ±20°)
                                     ↓
              accel = g·tan(tilt)   → integrate → vel, pos
```

Gains pulled directly from
`crazyflie-firmware/src/platform/interface/platform_defaults_sitl.h`:

| Constant | Value | What it does |
|---|---|---|
| `PID_POS_X_KP` | 2.0 m/s per m | Position error → velocity command |
| `PID_POS_VEL_X_MAX` | 1.0 m/s | Hard cap on velocity setpoint |
| `PID_VEL_X_KP` | 25.0 deg per m/s | Velocity error → tilt angle |
| `PID_VEL_X_KI` | 1.0 deg·s per m/s | Integral (causes overshoot) |
| Tilt cap | ~20 deg | Limits max acceleration ≈ 3.4 m/s² |

Step response (calibration plot at `/tmp/cascaded_pid_step_response.png`):
- Rise time ~1.2s (vs 0.9s for point-mass)
- ~2.5% overshoot from velocity-loop integral wind-up
- Phase-portrait shows curved trajectories (overshoot) vs point-mass straight

Plus **domain randomization** (`randomize_gains` in dynamics.py): per-episode
sample of Kp/Kd/Ki within ±25% of firmware defaults, covering battery sag,
prop wear, manufacturing variance, ground effect. Standard sim2real practice.

---

## Server training procedure

Server hardware: 2× AMD EPYC 9554 (128 cores), 755 GiB RAM, 5× NVIDIA RTX
6000 Ada (49 GiB VRAM each).

### 1. Sync workspace to server

From your laptop, via Tailscale:

```bash
# Set the server hostname/IP once
export CS_SERVER=server.taillxxxx.ts.net   # your tailnet hostname

# Sync (excludes build artifacts; first sync ~2 min, incremental ~10 sec)
rsync -avzh --progress \
  --exclude '.git/' --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  --exclude 'exp1/rl_training/checkpoints/' --exclude 'exp1/rl_training/tb/' \
  --exclude '*.bag' --exclude '__pycache__' --exclude '.crazyswarm2-upstream-backup/' \
  ~/cs2_ws/ "$CS_SERVER:cs2_ws/"
```

### 2. SSH in (VSCode Remote-SSH or terminal)

```bash
ssh "$CS_SERVER"
cd cs2_ws

# One-time conda env setup if not already there:
#   conda env create -f environment.yml   # or pip install -r requirements.txt
# Required pkgs: stable-baselines3, torch (CUDA), gymnasium, numpy>=2, pyyaml

conda activate crazyflie  # or whatever the env is called
```

### 3. Bump training config for full run

```bash
# edit src/rl_demo/rl_training/configs/ppo_config.yaml
ppo:
  total_timesteps: 2_000_000   # bump from 100_000 to ≥2M
  n_envs: 64                   # bump from 8 to 64 (one core per env)
  # keep everything else
```

### 4. Run training (background + log)

```bash
cd src/rl_demo
mkdir -p /tmp/iter10_logs
nohup /home/rk32226/miniconda3/envs/crazyflie/bin/python \
  -m rl_training.training.train_ppo \
  --ckpt-suffix iter10 \
  > /tmp/iter10_logs/train.log 2>&1 &
echo $! > /tmp/iter10_logs/train.pid

# Tail progress
tail -f /tmp/iter10_logs/train.log
```

Expected wall-clock on the server: **2-4 hours for 2M steps with n_envs=64
on CPU**. GPU isn't a bottleneck here — the PPO policy network is tiny;
env throughput is the cap.

### 5. Monitor via TensorBoard

```bash
# On server, expose TB port
tensorboard --logdir exp1/rl_training/tb --port 6006 --bind_all
```

From laptop:

```bash
ssh -L 6006:localhost:6006 "$CS_SERVER"
# then browse http://localhost:6006
```

Healthy training looks like:
- `rollout/ep_rew_mean` climbs from ~-2 to >50 over the first 200k steps,
  then plateaus near 100-200 by 1M
- `train/value_loss` decreases monotonically (modulo noise) from ~5 to <1
- `train/approx_kl` stays under 0.02 (clip_range is doing its job)
- `train/explained_variance` rises from ~0 toward >0.8

### 6. Pull checkpoint back to laptop

From your laptop:

```bash
rsync -avh --progress \
  "$CS_SERVER:cs2_ws/exp1/rl_training/checkpoints/ppo_coverage_iter10_final.zip" \
  ~/cs2_ws/exp1/rl_training/checkpoints/

# Also pull the TB logs if you want to revisit
rsync -avh --progress \
  "$CS_SERVER:cs2_ws/exp1/rl_training/tb/" \
  ~/cs2_ws/exp1/rl_training/tb/
```

---

## Crazysim transfer test (after iter10 is back)

```bash
cd ~/cs2_ws
./scripts/coverage_restart.sh fed_dcsa rl  /home/rk32226/cs2_ws/exp1/rl_training/checkpoints/ppo_coverage_iter10_final.zip
```

The script now (post-2026-05-28) does:
- Spawns drones at **cardinal sector centerlines** (`COVERAGE_SPAWN_LAYOUT=cardinal`)
- Disables coverage_demo.launch's redundant takeoff (`enable_takeoff:=false`)
- Adds `policy_warmup_sec:=5` so the planner waits for takeoff to settle
- Matches deployment streamer `max_setpoint_velocity` to training (1.0 m/s)
- Per-drone debug logs at `/tmp/rl_planner_cf{1..4}.log` (every ~1s: obs
  scalars, action raw + clipped, target, leash, rescue activations)

Expected behavior with iter10:
- All 4 drones stay within their assigned wedges (>95% of time)
- Drones ride near the leash edge (`r/leash` ≈ 0.95-1.0)
- During the t=180s wind handoff, the FedDCSA optimizer keeps Σc_i r² < B_op
- Under `coverage_restart.sh lagrangian rl`: sustained Σc_i r² > B_op in the
  60s post-handoff window → demonstrable data loss

### Diagnostic plot

After running for ~200s, kill the sim and run:

```bash
/home/rk32226/miniconda3/envs/crazyflie/bin/python /tmp/plot_rl_logs.py
# saves /tmp/rl_transfer_plot.png
```

Shows: drone XY trajectories vs sector wedges, target trajectories, per-drone
d_phi (angle from sector centerline), r over time vs leash, theta action history.

---

## Reference: files modified for iter10

- `src/rl_demo/rl_training/env/dynamics.py` — `CascadedPidDrone` + `randomize_gains`
- `src/rl_demo/rl_training/env/coverage_env.py` — `dynamics_model` + `randomize_dynamics` params
- `src/rl_demo/rl_training/configs/ppo_config.yaml` — `dynamics_model: cascaded_pid`
- `src/rl_demo/rl_training/training/train_ppo.py` — pass-through of new env knobs
- `src/cf_coverage_planner/cf_coverage_planner/rl_planner_node.py` — deployment node (debug logs, sector-rescue, airborne gate, sys.path hoists for SB3/numpy/rl_training)
- `src/cf_coverage_planner/cf_coverage_planner/trajectory_streamer_node.py` — alternative streamer using `/cfN/go_to` (firmware polynomial). Built but NOT wired in by default — only needed if iter10 still drifts after the cascaded PID retrain.
- `src/cf_coverage_planner/launch/coverage_demo.launch.py` — `planner_type:=rl` branch + `rl_checkpoint` launch arg
- `src/cf_coverage_planner/config/arena_4drone.yaml` — `rl:` block with defaults
- `src/cf_coverage_planner/setup.cfg` — installs scripts to `lib/<pkg>/` so `ros2 run` finds them
- `scripts/coverage_restart.sh` — RL planner support + cardinal spawn + warmup
- `crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh` — `COVERAGE_SPAWN_LAYOUT=cardinal|corners` switch

## Gotchas the next chat needs to know

1. **rl_planner_node hoists `sys.path` at module load** to put conda site-packages
   ahead of `/usr/lib/python3/dist-packages`. Required because ROS sources
   system numpy 1.26 first, which can't load SB3 checkpoints saved with
   numpy 2.x.
2. **The cv2/atari numpy 1.x import warning in rl_planner_node stderr is
   non-fatal** — SB3 catches it; the policy still loads. Don't chase it.
3. **Training checkpoint pickles `rl_training.training.feature_extractor`**.
   The deployment node auto-resolves `cs2_ws/src/rl_demo` and prepends to
   `sys.path` so the unpickle works. Param `rl_training_src_dir` overrides.
4. **Cascaded PID model uses firmware's small-angle approximation
   `accel = g·tan(tilt_rad)`**, which is exact for the controller's output
   range (±20°) since tan(20°) = 0.364 vs the linear approx 0.349 — 4%
   error, well within the domain-randomization band.
5. **Spawn layout matters**. The old `corners` layout (cf1 NE, cf2 SE, cf3
   SW, cf4 NW) was the figure-8 demo's spawn. RL planner needs `cardinal`
   (cf1 E, cf2 N, cf3 W, cf4 S) to match the arena_4drone.yaml sector
   centerlines.
