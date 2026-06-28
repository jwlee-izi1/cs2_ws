# Sim validation log

Chronological ledger of **baseline** sim-validation runs (CrazySim + crazyswarm2) that
establish reference low-level behaviour before/against research changes. Distinct from
[federated_coverage.md](federated_coverage.md) (the RL / CoRL research doc) — this file is
the plain "does the low-level stack track?" record.

Reproduction commands live in [../CRAZYSIM_MIGRATION.md](../CRAZYSIM_MIGRATION.md) "How to
run" (single-drone flow). This log records only scenario + result + verdict, not the full
command sequence.

---

## Test 1 — single-drone figure-8 (arc-sweep) tracking — 2026-06-27 ✅

**Scenario.** One drone (cf1), `quadrant_figure8_node` **unmodified**, through its real
path: `/coverage/leash` → planner → `/cf1/policy_target` → `setpoint_streamer` →
`/cf1/cmd_position` → firmware PID. Constant leash 1.0 m (via `ros2 topic pub`), φ_mid=0°
(East sector), ±36° sweep, 9 s period, z=0.6 m. BVC is **no-op** here (single drone,
nOthers=0) — this is a pure low-level tracking check, and the **baseline** for the upcoming
2-drone BVC crossing test (Test 2).

> Note: `quadrant_figure8_node` traces a tangential **arc sweep**, not a self-crossing
> figure-8 — the name is a legacy planner-slot name. Fine as a tracking benchmark.

**Result** (36 s = 4 sweep periods, steady state t≥3 s; odom vs cmd_position):

| Metric | Value |
|---|---|
| Cross-track error (off-path, radial) | RMS **1.8 cm**, max 3.0 cm |
| Along-track error (phase lag) | RMS **17.1 cm**, max 28.1 cm |
| Altitude hold (z) | max **0.1 cm** |
| Radius from origin | 0.96–1.02 m (target 0.99) |

**Verdict.** Low-level PID tracking **OK**. The drone sits on the commanded path to ~1.8 cm
RMS; the 17 cm along-track figure is expected phase lag behind a ~0.43 m/s moving setpoint,
not off-path drift. Altitude rock-solid. Adopt as the **baseline** that the 2-drone BVC
crossing (Test 2) is compared against. GPU split verified clean (Gazebo on RTX via
`gz-gpu`, desktop on Intel).

---

## Test 2 — two-drone head-on position swap — BVC first real engagement — 2026-06-27 ✅

**Scenario.** Two drones facing each other on the x-axis (cf1 spawn (−1,0), cf2 (+1,0)),
commanded to **swap** positions (cf1→(+1,0), cf2→(−1,0)) at the same z=1.0 — a direct
head-on collision course (same y=0). Setpoints streamed straight-line to each drone's
`/cfN/cmd_position` at 50 Hz (no planner); the only thing that can bend the trajectory is
the onboard firmware BVC. Unlike Test 1, `nOthers>0` here, so BVC must actually engage.
Headless (Gazebo `gz sim -s` only, `gui:=false`) — verdict is quantitative (odom), not
visual. This is the **baseline** the future CBF swap will be compared against.

**Preconditions verified (BVC can't work without these):**
- `peer_broadcast_hz: 30.0` under `all:` → server logs *"BVC peer-position broadcast ON at
  30 Hz (peer ids {cf1:1, cf2:2})"*. 30 Hz ≪ firmware 5 s peer-staleness window.
- `colAv.enable: 1`; firmware defaults ellipsoidRadii 0.3/0.3/0.9 m, maxSpeed 0.5 m/s,
  horizon 1.0 s, sidestepThreshold 0.25 m → horizontal cell geometry guarantees
  center-to-center separation ≥ 2·0.3 = **0.60 m**.
- Frame check: `/cfN/odom` reports absolute Gazebo world pose (CrazySim plugin feeds it to
  the estimator as `CrtpExtPose` — "perfect mocap"), so `cmd_position` and peer-broadcast
  share one world frame. Confirmed at rest: cf1 odom (−1,0), cf2 (+1,0).

**Result — BVC ON vs OFF control** (OFF = same scenario, `peer_broadcast_hz: 0` →
`nOthers=0` → no-op):

| Metric | **BVC ON** | **BVC OFF (control)** |
|---|---|---|
| Min horizontal separation | **0.613 m** (= cell guarantee 0.60 m) | **0.097 m** (near pass-through) |
| Lateral excursion at crossing (x≈0) | cf1 y=−0.56, cf2 y=+0.57 (pre-emptive, symmetric) | y≈±0.05 then chaotic post-collision kick (cf1 overshot to x=1.78, y=0.89) |
| Swap completed | yes (cf1→+1, cf2→−1) | yes, but via near-collision + violent recovery |
| Deadlock | none | n/a |

**Evidence BVC actively avoided (vs paths merely not overlapping):**
1. **On vs off min-separation**: 0.613 m vs 0.097 m. With peers off the two paths *fully*
   overlapped (drove to ~10 cm = on top of each other); with peers on, separation never
   dropped below the cell wall. The broadcaster is the trigger.
2. **Sidestep signature**: commanded y≡0 the whole run, yet BVC-on actual trajectories
   bulged to |y|≈0.57 m, with **cf1→−y and cf2→+y** — exactly the firmware `sidestepGoal`
   prediction (`goal × ẑ`: cf1 goal +x ⇒ −ŷ, cf2 goal −x ⇒ +ŷ). The BVC-off lateral motion
   is reactive/asymmetric (post near-collision), not this clean pre-emptive split.

**Verdict.** BVC **OK** — first real (nOthers>0) engagement verified. Holds the 0.6 m
geometric guarantee on a worst-case symmetric head-on, resolves the swap via sidestep
(no deadlock), and the on/off control proves the avoidance is BVC's doing, not geometry.
Adopt the BVC-on numbers (min sep 0.61 m, sidestep ±0.57 m) as the **baseline** for the
CBF swap comparison.

**Reproduce** (headless; commands in `CRAZYSIM_MIGRATION.md` "How to run" → 2-drone BVC):
config `config/crazyflies_sitl_2drone.yaml` (+ `_nobvc.yaml` control), launcher
`scripts/sitl_2drone_headon.sh`, driver `scripts/bvc_headon_test.py`. CSVs in `/tmp/bvc/`.
