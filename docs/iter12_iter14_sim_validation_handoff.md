# Iter12 / Iter14-hybrid sim validation handoff

**Created 2026-06-03 for handing off live-sim validation to a parallel chat.**

The main chat is running iter15 PPO retrain (~30 min). This doc gives a fresh chat the context it needs to validate iter12 and iter14-hybrid candidates in the **actual ROS+Gazebo+RViz live sim** while iter15 trains.

## Mission for this chat

We have 3 RL policy candidates that all passed Phase 0 pure-Python validation. Before committing to one in the paper, we need to confirm the live ROS+Gazebo sim produces the **same Σ_drone violation profile** that the pure-Python env predicted. The translation gap (cascaded PID in env vs polynomial streamer + Crazyflie firmware in deployment) could shift the numbers.

**Your job:** for each candidate, run the live sim end-to-end (Gazebo + crazyflie_server + RL planner + mapper + RViz), record the demo window [0, 300s], confirm:
1. The drone trajectories visually match the pure-Python predictions (sweep behavior, fire-tracking)
2. Σ_drone violation count is within ±30% of the pure-Python prediction (live demo can vary)
3. r_max (1.9 m) is not breached in any drone
4. FedDCSA never produces drops; Lagrangian produces visible white drops in RViz around t=160-220s
5. No regressions vs the current iter11 deployment (95% confidence the iter12 work didn't break the Gazebo bringup chain)

Final deliverable: **a short report (~1 page) ranking the 3 candidates by live-sim demo quality + a recommendation for which to ship as the iter12-era deployment.**

## The 3 candidates

| ID | Checkpoint | rl_planner_node.py policy.predict usage | Pure-Python prediction (Lagrangian, demo window [0,300s]) |
|---|---|---|---|
| **iter12 raw** | `exp1/rl_training/checkpoints/ppo_coverage_iter12_final.zip` | Use both θ and r_frac from policy | 11 violations, 33s window, max r=1.907m ⚠ |
| **iter14 hybrid** | Same as iter12 (`...iter12_final.zip`) | θ from policy, **hardcode `r_frac = 0.99`** in deployment | 44 violations, 43s window, max r=1.849m ✓ |
| **sweep_full (reference)** | No checkpoint — pure hardcoded sweep | `theta = 1.0 * sin(2π(t+phase)/8.0)`, `r_frac = 0.99` | 263 violations, 56s window, max r=1.826m ✓ |

## Live sim invariants (don't touch)

These are locked across iterations. The pure-Python validation depends on them:

- `arena_4drone.yaml`: K=200, budget_B=8.0, c_1=0.015, c_2=0.18, r_max=1.9 for all 4 drones, r_star=1.85, q=[10, 1.5, 1.0, 0.5], min_leash=0.8, action_slack=1.0
- `thermal_field.yaml`: random_seed=42, randomize_seed=0, wind rotation 0.25°/s, suppression max_radius=1.4
- `coverage_demo.launch.py` mapper params: constraint_B_op=8.15, constraint_scale=2.5, constraint_max_drop=0.95, dropped_ttl_seconds=10.0, publish_status_marker=True
- `rl_planner_node.py`: min_leash=0.8, action_slack=1.0, **r_max safety must remain 1.9m**

## Files to modify per candidate

### iter12 raw
1. `src/cf_coverage_planner/cf_coverage_planner/rl_planner_node.py` lines 340-358 — REMOVE the SWEEP_AMP/PHASE block and `r_frac = 0.99` hardcode. REPLACE with:
   ```python
   action = ...  # already computed earlier by policy.predict(obs)
   theta = float(action[0])           # use policy's θ
   r_frac = float(action[1])          # use policy's r_fraction
   ```
2. `scripts/coverage_restart.sh` line 22 — change `RL_CHECKPOINT` default to `$WS/exp1/rl_training/checkpoints/ppo_coverage_iter12_final.zip`

### iter14 hybrid
1. Same `rl_planner_node.py` edit BUT keep `r_frac = 0.99` hardcoded:
   ```python
   action = ...  # already computed earlier
   theta = float(action[0])           # use policy's θ
   r_frac = 0.99                      # hybrid — hardcode (NOT from policy)
   ```
2. `coverage_restart.sh` line 22 — same iter12_final.zip path (hybrid uses iter12 weights)

### sweep_full reference
- Keep current rl_planner_node.py (the SWEEP_AMP=1.0, SWEEP_PERIOD=8.0 hardcoded sweep) — already validated as sweep_full in Phase 0
- Make sure `SWEEP_AMP=1.0` not 0.55 (the file MAY still have iter11's 0.55, check first)

## Validation procedure (per candidate)

1. **Apply the rl_planner_node.py edit + coverage_restart.sh edit** for the candidate
2. `cd /home/rk32226/cs2_ws && colcon build --packages-select cf_coverage_planner --symlink-install`
3. **Full top-down kill** of any existing sim:
   ```bash
   pkill -9 -f "ros2 launch|rviz2|gz sim|crazyflie_server|polynomial_streamer_node|rl_planner_node|radial_coverage_node|thermal_mapper_node|thermal_sensor_node|thermal_ground_truth_node|qi_estimator_node|coverage_rviz|thermal_demo|teleop|motion_capture|joy_node"
   docker rm -f $(docker ps -aq --filter "name=cf2-")
   ```
4. Run **Lagrangian sim** for ~250s (covers k=0 to past k=110 = sim t=220+):
   ```bash
   nohup ./scripts/coverage_restart.sh lagrangian rl > /tmp/sim_lag.log 2>&1 &
   disown
   ```
5. Wait for "stack is up" + all 4 drone odoms publishing, then start the recorder for 260s:
   ```bash
   PYTHONPATH=src/rl_demo:src:src/thermal_mapping:src/fed_dcsa python3 \
     exp1/paper_figures/result3_data_loss_comparison/watch_and_render.py \
     --tag iter12_live_lagrangian
   ```
   (this records every `/thermal_map` frame, writes per-violation PNGs live)
6. After 260s, kill the sim (same pkill). Inspect `/home/rk32226/cs2_ws/exp1/paper_figures/result3_data_loss_comparison/live_iter12_live_lagrangian/` for the per-frame PNGs.
7. Repeat for **FedDCSA**: same kill, `coverage_restart.sh fed_dcsa rl`, same recorder (tag `iter12_live_feddcsa`). Should produce ZERO violation PNGs.
8. Record final stats:
   - Total violation frames captured
   - Window (first violation frame t, last violation frame t)
   - Max drone r observed across all 4 drones (`grep r_drone /tmp/sim_lag.log`)
   - Visual quality of the white drops in RViz (clustered/sustained vs scattered/flickering)

## Pre-flight verification (run before any candidate)

Quick sanity check the workspace state matches the locked invariants:

```bash
# arena_4drone.yaml — K=200, budget=8.0, min_leash floor for RL=0.8
grep -E "^  K:|^  budget_B:|min_leash" src/cf_coverage_planner/config/arena_4drone.yaml

# rl_planner_node.py — r_max default 1.9, action_slack default 1.0
grep -E "self.declare_parameter.*r_max|self.declare_parameter.*action_slack|self.declare_parameter.*min_leash" src/cf_coverage_planner/cf_coverage_planner/rl_planner_node.py

# coverage_demo.launch.py — mapper params for RL mode
grep -A 8 "if planner_type == 'rl'" src/cf_coverage_planner/launch/coverage_demo.launch.py

# thermal_field.yaml — random_seed=42, rotation 0.25, suppression max_radius=1.4
grep -E "^random_seed|^randomize_seed|rotation_rate|max_radius" src/thermal_mapping/config/thermal_field.yaml
```

## Known gotchas

1. **RViz Ogre crash at k~75 (sim t~150s)**: rapid sphere-marker DELETEALL+ADD cycle in the mapper triggers `Ogre::InvalidParametersException: Cannot destroy a null MovableObject`. RViz dies, takes the whole `ros2 launch` with it. **Workaround**: record the violation PNGs via watch_and_render.py (saves them to disk regardless of RViz state). Don't rely on RViz screen recording for the final paper figure.

2. **Flaky Gazebo startup**: ~10% of restarts produce only 3/4 drone odom publishers (one cf2-sitl container loses its connection during the takeoff handshake). Symptom: one drone in RViz shows no markers, the mapper reports only 3 active sensors. **Fix**: full kill (the long pkill above) + retry. Don't dig deeper unless 3+ restarts fail.

3. **The polynomial_streamer node is `polynomial_streamer_node`, NOT `setpoint_streamer_node`**. The earlier kill scripts missed this and old streamers survived → drones didn't move because two streamers fought. The pkill list above includes both — keep it as-is.

4. **iter12's late-episode FedDCSA "violations" (29 frames at t=333-398s)** happen AFTER the optimizer K=200 limit (sim t=400s). Those are fire-decay-phase artifacts and **DO NOT COUNT** against the demo (hardware demo cuts off at ~300s anyway). Same for any post-300s violations in any candidate.

5. **The "drops" in RViz are 0.378m × 0.378m white squares** — the drone's thermal sensor FOV. They appear right where the drone IS at the moment of drop, then fade after the 10s TTL. They are NOT centered on the drone position at view-time because the drone has moved since.

## Reference data from pure-Python validation

The pure-Python env's predictions (must roughly match the live sim):

| | iter12 raw Lagrangian | iter14 hybrid Lagrangian | sweep_full Lagrangian |
|---|---|---|---|
| Σ_drone max | 8.62 | 8.58 | 8.65 |
| Σ_leash max | 8.81 | 8.81 | 8.81 |
| Violation frames | 20 total / 11 in demo window | 44 in demo window | 263 in demo window |
| Demo violation window | 161.4 → 193.6 s | 175.6 → 217.8 s | 162.0 → 218.4 s |
| Max drone r | 1.907 m ⚠ | 1.849 m ✓ | 1.826 m ✓ |

NPZ files for each are at `/home/rk32226/cs2_ws/exp1/rl_training/phase0_eyeball/`:
- `ppo_iter12_lagrangian_400s.npz` / `ppo_iter12_feddcsa_400s.npz`
- `hybrid_iter12_lagrangian_400s.npz` / `hybrid_iter12_feddcsa_400s.npz`
- `sweep_full_lagrangian_400s_K200.npz`

## What the main chat is doing in parallel

Running iter15: same iter13 reward config (w_anchor=80 symmetric, action_slack=1.0) but **resuming from iter12** (not from BC). 200k more steps, ~30 min. Goal: see if PPO-resumed-from-iter12 with the stronger anchor produces a policy whose RAW outputs (no hybrid override) match or beat the hybrid's demo numbers. iter15 output: `exp1/rl_training/checkpoints/ppo_coverage_iter15_final.zip`. The main chat will validate iter15 in pure-Python first, then hand it off to YOU for live-sim validation if it passes.

## Don't touch (out of scope)

- The reward function (rewards.py) — iter15 is using it, don't modify mid-train
- The env (coverage_env.py) — also being used by iter15
- arena_4drone.yaml — locked
- thermal_field.yaml — locked
- Any docs in docs/ — final lock-down is a separate phase
- Don't kill the running iter15 PPO process (`pgrep -af train_ppo`)
- Don't run any pure-Python rollouts from src/rl_demo/rl_training/eval/ — those would compete with iter15 training for CPU

## Tools you'll use

| File | Purpose |
|---|---|
| `scripts/coverage_restart.sh` | Bring up Gazebo + crazyswarm2 + coverage demo stack |
| `exp1/paper_figures/result3_data_loss_comparison/watch_and_render.py` | Record /thermal_map frames live, dump PNGs per violation |
| `src/cf_coverage_planner/cf_coverage_planner/rl_planner_node.py` | The deployment-side RL node; edit for each candidate |
| `src/cf_coverage_planner/launch/coverage_demo.launch.py` | Launch file. Read-only (mapper/optimizer args live here) |

## Ground truth: when in doubt

Read `/home/rk32226/.claude/plans/i-want-to-start-valiant-babbage.md` — the master plan file with full iteration history (iter11 through iter15 candidates, decisions, what failed and why).

Also `/home/rk32226/.claude/projects/-home-rk32226-cs2-ws/memory/MEMORY.md` for the workspace's persistent notes (project context, locked params, hardware safety).
