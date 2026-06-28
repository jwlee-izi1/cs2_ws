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
