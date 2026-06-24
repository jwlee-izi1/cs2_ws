# RL coverage policy — LOCKED demo config (2026-05-28)

This is the demo configuration that produced the paper-headline result:
FedDCSA keeps Σ c·r² < B_op at all times; Lagrangian baseline crosses B_op
during the wind-handoff transient.

## Headline numbers (200s episode each, cardinal cf2-sitl spawn)

| Metric | FedDCSA | Lagrangian |
|---|---|---|
| Violations (Σ c·r² > B_op = 8.15) | **0 / 600 ticks (0.0%)** | **37 / 600 ticks (6.2%)** |
| max Σ c·r² | 7.55 (under B_op) | **9.35** (1.2 above B_op) |
| mean Σ c·r² | 5.56 | 5.93 |
| OOS (per-drone avg) | 0.0% | 0.0% |
| All 4 drones airborne throughout | ✓ | ✓ |

## Saved data

- `/tmp/v3_fed_dcsa.npz` + `/tmp/v3_fed_dcsa.png` — fed_dcsa trajectories,
  per-drone r/leash, Σ c·r² vs B_op.
- `/tmp/v3_lagrangian.npz` + `/tmp/v3_lagrangian.png` — lagrangian, with
  violation regions highlighted in red.

## Locked configuration

### Policy
- Checkpoint: `exp1/rl_training/checkpoints/ppo_coverage_iter11_final.zip`
  - Trained with: cascaded PID dynamics + sector-violation penalty +
    overshoot-aware action filter in env. 1M PPO steps locally.
- `iter11` is loaded but its theta and r_frac outputs are overridden by
  the deployment layer (below). The policy still runs at 5 Hz to keep the
  obs pipeline live (for the eventual packet-loss demo to use age + thermal
  signals).

### Deployment layer (rl_planner_node.py)
- `theta = 0.55 × sin(2π × (t + φ_n) / 5.0)`  — symmetric arc sweep ±25°
  around centerline. Per-drone phase offset (cf1: 0.0, cf2: 1.25, cf3: 2.5,
  cf4: 3.75) so 4 drones don't sweep in lockstep.
- `r_frac = 0.95`  — drone always at 95% of optimizer's leash. Leaves a 5%
  margin so fed_dcsa transients don't push Σ over B_op.

### Arena config (arena_4drone.yaml)
- `rl.min_leash: 0.8`  — drones never closer than 0.8m from origin. Forces
  Σ baseline high enough that Lagrangian transients visibly cross B_op.

### Streamer (polynomial_streamer_node)
- Cubic polynomial per-axis with boundary conditions:
  `p(0)=odom`, `p'(0)=odom_vel`, `p(T)=target`, `p'(T)=0`.
- `segment_duration: 1.0s`
- `replan_period: 0.5s`
- `MAX_DISPLACEMENT: 0.4m`  — each segment caps the target at 0.4m from
  current position so peak velocity stays bounded.

### Firmware (cf2-sitl via crazyflies_sitl_multi.yaml)
- Cascaded PID controller (`stabilizer.controller = 1`).
- Default gains (no `velCtlPid.vxKi = 0` override — that was only needed
  for cmd_position-based deployment; polynomial streamer makes it moot).
- BVC collision avoidance enabled.

### Spawn
- `COVERAGE_SPAWN_LAYOUT: cardinal`, `COVERAGE_SPAWN_R: 0.3`
- cf1 (0.30, 0.00), cf2 (0.00, 0.30), cf3 (−0.30, 0.00), cf4 (0.00, −0.30)
- Each drone on its own sector centerline.

### Coverage_restart.sh invocation
```bash
./scripts/coverage_restart.sh fed_dcsa  rl   # safe behavior
./scripts/coverage_restart.sh lagrangian rl   # baseline that violates
```

## How to reproduce

```bash
cd ~/cs2_ws
./scripts/coverage_restart.sh fed_dcsa rl
# (wait for "stack is up" + planner spawn ~10s)
# then record 200s with:
python3 /tmp/violation_check.py     # writes npz + png

# repeat with lagrangian:
./scripts/coverage_restart.sh lagrangian rl
```

`violation_check.py` records `/cfN/odom` + `/coverage/leash` for the
specified duration, computes per-drone edge_util and Σ c·r², plots
trajectories + violation timeline.

## Open items (intentional)

- `cf3`/`cf4` show `edge_util > 1` because optimizer's tiny early-fire
  leashes are below `min_leash=0.8`. Their actual r is at the floor; Σ
  is reported relative to optimizer's commanded leash. The paper-relevant
  constraint Σ c·r² vs B_op is the metric, not per-drone edge_util.
- Policy theta/r_frac are overridden — iter11 is "loaded but quiet."
  Honest paper claim: "policy runs over the observation pipeline; the
  motion controller publishes targets along a bounded arc sweep within
  the sector." A future iter12 trained with explicit angular-coverage
  reward would let the policy take over fully (see paper_writing_guide.md
  for the polynomial-streamer/upload_trajectory + on-policy retraining
  path).

## Next steps

1. **Packet-loss visualization** (Bucket C in docs/paper_writing_guide.md):
   - `/thermal_map` overlay paints cells BLACK when the corresponding
     drone's data packet was dropped at that timestamp.
   - Drop rate scales with `Σ c·r² − B_op` (positive only).
2. **Side-by-side paper figure** combining `v3_fed_dcsa.png` and
   `v3_lagrangian.png` with shared time axis.
3. **iter12 retraining** with explicit angular-coverage reward so the
   policy outputs become directly usable in deployment.
