# Federated Coverage Demo (CoRL paper)

> **Disambiguation:** the `src/fed_dcsa/` package has historically hosted multiple
> Fed-DCSA problem formulations. This doc describes the **radial coverage / leash**
> variant (CoRL, the current project). Other variants in the same package:
> *payload tilt correction* (`payload_optimizer_node`) — used by the payload project,
> see [payload.md](payload.md); and *distributed surveillance* (charging stations +
> swap pairs), which has been archived to
> [archive/surveillance_fed_dcsa.md](archive/surveillance_fed_dcsa.md).

## Goal

Distributed resource allocation with **persistent safety guarantees** for a multi-drone
mapping team. A federated optimizer at the server publishes a per-drone scalar
**leash** $r_i^k$ at each round $k$, which constrains how far drone $i$ may extend into
its angular sector. The aggregate emission/transmit-power constraint

$$\sum_i c_i (r_i^k)^2 \le B$$

must be satisfied at **every iterate** $k$ — that's the paper's contribution
(Theorem 1: feasibility at every algorithmic iterate, not just at convergence).

This project demonstrates the algorithm running on 4 Crazyflies in Gazebo (CoRL
paper figures + supplementary video), with eventual deployment on real hardware.

Source-of-truth 2D matplotlib design lives at
[/home/rk32226/drone-rl-2d/multi_drone/experimental_setup_plan.md](/home/rk32226/drone-rl-2d/multi_drone/experimental_setup_plan.md);
the 3D Gazebo version is a faithful port of that design.

## Packages

| Package | Role |
|---|---|
| `src/fed_dcsa` | FedDCSA algorithm + ROS node (`radial_coverage_node.py`). Publishes `/coverage/leash`. |
| `src/coverage_optimizer_interfaces` | `Leash.msg` — the topic contract between optimizer and planner. Hot-swappable: any optimizer can publish this contract. |
| `src/cf_coverage_planner` | Per-drone Level-2 planner (radial-spokes policy in `polar_lawnmower.py`) + 20 Hz rate-limited setpoint streamer (`setpoint_streamer_node.py`). |
| `src/baseline_optimizers` | Mock leash publishers (`constant_leash_node`, `sinusoidal_leash_node`) for testing the planner standalone. |

**Shared package used (not owned by this project):**

- `src/thermal_mapping` — the data plane. Drones publish synthesised thermal sensor
  readings; a shared mapper accumulates a `/thermal_map` GridMap that gets rendered in
  RViz. See [thermal_mapping_demo.md](thermal_mapping_demo.md) for the contract.

## Architecture / topic contracts

```
fed_dcsa/radial_coverage_node ── /coverage/leash ──→ cf_coverage_planner/coverage_planner_node
(or any optimizer)                (Leash.msg)        (one per drone, ROS shim)
                                                            │
                                                       /cfN/policy_target
                                                       (PoseStamped, 5 Hz)
                                                            ▼
                  cf_coverage_planner/setpoint_streamer_node ── /cfN/cmd_position ──→ crazyflie firmware
                  (one per drone, rate-limited 20 Hz)        (Position, 20 Hz)
```

**Key invariant — surviving contracts** (these don't change when the lawnmower is
replaced by an RL policy):

- `/coverage/leash` (`coverage_optimizer_interfaces/Leash`): `drone_names`, `radii`,
  `gate_state`. Any optimizer publishes; any policy subscribes.
- `/cfN/policy_target` (`geometry_msgs/PoseStamped`): policy → streamer. Defined at
  altitude $h$ (currently 1.0 m).
- `/cfN/cmd_position` (`crazyflie_interfaces/Position`): streamer → firmware. Same on
  sim and hardware (per [CRAZYSIM_MIGRATION.md §1.2](../CRAZYSIM_MIGRATION.md)).

**Phase B additions** (dynamic fire + oracle $q_i$):

- `/coverage/sector_weights` (`std_msgs/Float64MultiArray`): `qi_estimator_node`
  → `radial_coverage_node`. Length-N array (one $q_i$ per drone) at `round_rate_hz`.
  Optimizer uses YAML values as fallback until first message; thereafter uses
  the live $q_i$ each round. Any estimator (oracle today, learned later) can
  publish this contract.
- `/thermal_truth` (`grid_map_msgs/GridMap`): `thermal_ground_truth_node` →
  RViz/rosbag/eval. Pure visualization of $T(x, y, t)$ on a fixed grid. Sensors
  do NOT subscribe — they evaluate `field.evaluate(x, y, t)` in-process.
- `/thermal_map` layers (`grid_map_msgs/GridMap`): `thermal_mapper_node` → consumers.
  Now last-write-wins instead of running-mean. Three layers: `thermal`
  (last-observed value), `age_seconds` (seconds since last write at publish
  time), `last_update_time` (wall-clock seconds-since-epoch of last write).
  The staleness layers are what the RL policy will eventually consume to
  prioritize re-visits.

**What's throwaway** (replaced when RL lands): `coverage_planner_node.py`,
`polar_lawnmower.py`, and `quadrant_figure8_node.py`. **What survives**:
`setpoint_streamer_node.py`, `Leash.msg`, both topic contracts above, the YAML
config schema, the qi_estimator + dynamic fire pipeline (the substrate the
policy trains against).

## Dynamic fire substrate

The optimizer chases a **non-stationary** $q_i(t)$ produced from a synthetic
moving fire. This is the substrate the RL policy will eventually train against
and the per-iterate-feasibility paper claim rests on.

### Fire model (M4 — rotating-wind wavefront + burn profile)

Closed-form, vectorized, fully reproducible from
[`src/thermal_mapping/config/thermal_field.yaml`](../src/thermal_mapping/config/thermal_field.yaml).
Implemented in
[`src/thermal_mapping/thermal_mapping/thermal_field.py`](../src/thermal_mapping/thermal_mapping/thermal_field.py)
(`FireWavefront` + `BurnProfile` classes).

$$T(x, y, t) = T_{\text{ambient}} + T_{\text{peak}} \cdot \varphi(\tau), \quad \tau = (t - t_{\text{ign}}) - \frac{d}{v_{\text{eff}}(\theta, t)}$$

- $d = \|(x,y) - p_{\text{ign}}\|$, $\theta = \mathrm{atan2}(y - y_{\text{ign}}, x - x_{\text{ign}})$.
- $v_{\text{eff}}(\theta, t) = v_{\text{front}} \cdot \bigl(1 + a \cdot \cos(\theta - \theta_w(t))\bigr)$ with wind direction $\theta_w(t) = \theta_{w,0} + \omega \cdot (t - t_{\text{ign}})$.
- $\varphi(\tau)$ is a piecewise-linear rise → plateau → decay bell:
  cell ignites at $\tau{=}0$, climbs linearly over `rise_duration`, holds peak for
  `plateau_duration`, cools linearly to ambient over `decay_duration`.
- Bounded fuel: cells outside an arena disc of radius `arena_radius = 1.9 m`
  contribute zero (matches drone HW $r_i^{\max}$).

Locked Phase A values: ignition at origin, $v_{\text{front}} = 0.01$ m/s,
wind rotating at $0.25°/\text{s}$ (90° sweep over the 6-min run), anisotropy
$a = 0.8$, burn = 10s rise / 15s plateau / 25s decay (50s total per cell),
peak 80°C above ambient. Designed for the **chase regime**: cf1 dominates
early (downwind for first ~3 min), then a sharp handoff to cf2 around
$t \approx 180s$ — the slow rotation lets cf1 fully build up $r_1$ to $r_\star$
before the transition, producing a large transient the baseline lags
through (and FedDCSA absorbs via its gate).

### Firefighter radius-cap suppression (2026-05-26)

Paper narrative: the 1.9 m `arena_radius` is a **firebreak ridge** built by
the fire crew before the demo. Firefighters then **hold an inner line** along
the cf3/cf4 arc — the fire crew is concentrated there because that's where
the dominant wind direction would otherwise push the front. cf1+cf2 sectors
have no inner line, so fire burns all the way out to the ridge there.

Mechanically: a world-frame angular **arc** caps the radius the fire can
reach in cf3/cf4 directions. For a cell at world angle $\theta_{\text{cell}}$
from arena center and radius $r_{\text{cell}}$, let
$\Delta = \mathrm{wrap}(\theta_{\text{cell}} - \theta_{\text{arc}})$. Inside
the arc ($|\Delta| \le \text{half\_angle}$) the cell can only burn if
$r_{\text{cell}} \le r_{\max}$; otherwise it contributes zero. In the edge
band of width `edge_taper`, the effective $r_{\max}$ linearly interpolates
between the suppression value and `arena_radius`. Outside the arc the cell
follows the normal `arena_radius` fuel disc.

Locked values (seed 0 default): arc center 225° (midpoint between cf3 and cf4
axes), half-angle 90° (covers both sectors entirely), max_radius 1.4 m,
edge taper 5°. Result: fire visibly **stops at 1.4 m** in the cf3/cf4 arc
(clear 0.5 m buffer zone before the 1.9 m ridge) while cf1/cf2 still ridge
out at 1.9 m. cf3 and cf4 still contribute meaningful $q_i$ (inner part of
their sectors burns); cf1 and cf2 are unaffected. With locked tuning
(rotation 0.25°/s, $q_i$ scale 0.20), baseline max $\sum c_i r^2 = 8.83$
vs FedDCSA's bounded 8.09 — per-iterate violation contrast **9.56×**
(close to the 10× pre-suppression reference, paper claim preserved).

### Run-to-run randomization (opt-in)

`fire.randomize_seed` in YAML. 0 (default) = deterministic legacy reproduction;
the locked paper figure uses this. Any value > 0 applies a deterministic
seeded rotation to the wind init AND the suppression cone (the cone is
**wind-locked** across seeds — dominant-vs-suppressed labels stay consistent),
plus a small uniform-disc jitter to the ignition point bounded by
`ignition_jitter_radius` (default 0.15 m). Different seeds produce different
dominant-sector sequences across the four drones — useful for RL training
data variety.

### Oracle $q_i$ estimator

Pure function in
[`src/thermal_mapping/thermal_mapping/qi_estimator.py`](../src/thermal_mapping/thermal_mapping/qi_estimator.py)
(`compute_sector_qi`). For each sector wedge:

$$q_i(t) = \alpha \cdot \text{mean}\bigl(T(x, y, t) - T_{\text{ambient}}\bigr) \text{ over cells in sector } i$$

where $\alpha$ = `qi.scale` from YAML (currently 0.18). Mean-over-all-cells is the
default; a `threshold > 0` mode (mean-over-hot-cells-only) is available but
rejected during Phase A because it flattens ratios between active sectors —
the opposite of what concentrates budget on the dominant drone.

Integration radius $r_{\text{outer}} = \text{arena\_radius} = 1.9$ m (matches
the fuel disc; using the larger $r_i^{\max}$ would dilute the mean with
always-ambient cells outside the fuel).

The ROS-side wrapper
[`qi_estimator_node.py`](../src/thermal_mapping/thermal_mapping/qi_estimator_node.py)
publishes `/coverage/sector_weights` (Float64MultiArray) at `round_rate_hz`.
The optimizer subscribes and folds the live `q_i` array into the algorithm
state at each round.

**Upgrade path** (deferred to RL session): replace the oracle with a learned
estimator from drone-local thermal observations. Topic contract unchanged —
any estimator publishes `/coverage/sector_weights`, optimizer doesn't care
where the values come from.

### Locked YAML (don't touch without rerunning Phase A previewer)

| File | What's locked |
|---|---|
| [`src/thermal_mapping/config/thermal_field.yaml`](../src/thermal_mapping/config/thermal_field.yaml) | `fire:` block (M4 wavefront + burn profile + bounded fuel + rotating wind); `qi:` block (`scale: 0.18`, `threshold: 0.0`). |
| [`src/cf_coverage_planner/config/arena_4drone.yaml`](../src/cf_coverage_planner/config/arena_4drone.yaml) | Optimizer params (`B=8`, `K=100`, `T=5`, `c1=0.015`, `c2=0.18`); sector geometry (`q=[10, 1.5, 1, 0.5]`, uniform `r_star=1.85`, `r_max=1.9`); figure-8 params (`min_leash=0.4`, `figure8_speed=2.0`, `center_alpha=0.5`, `amp_beta=0.25`); streamer ceiling (`max_setpoint_velocity=1.5`). |

## Offline previewer + tooling

The previewer is the **single source of truth** for tuning. Phase A locked the
YAML by iterating on these scripts; Phase B then verified that live ROS
trajectory reproduces what the previewer predicts.

| Script | Purpose |
|---|---|
| [`scripts/preview_thermal_scenario.py`](../scripts/preview_thermal_scenario.py) | Offline 7-row figure: truth-field snapshots + $q_i(t)$ + $r_i^k(t)$ (FedDCSA + Lagrangian baseline overlaid) + lag/constraint panel + drone-reach vs fire radial extent per sector. Imports `RadialCoverageFedDCSA` + `LagrangianBaseline` + `ThermalField` + `compute_sector_qi` directly. Pure Python, no ROS. |
| [`scripts/animate_thermal_scenario.py`](../scripts/animate_thermal_scenario.py) | GIF/MP4 side-by-side: FedDCSA arena \| Lagrangian baseline arena \| $\sum c_i r^2$ violation panel. Used to visually communicate the "FedDCSA stays feasible, baseline transients" story. |
| [`scripts/test_figure8_speed.py`](../scripts/test_figure8_speed.py) | 4-panel diagnostic for figure-8 loop traversal speeds. Identifies highest `figure8_speed` value where peak velocity stays under `max_setpoint_velocity`. Verified safe ladder: speed=1.0 (peak 0.46 m/s) → 2.0 (0.92) → 3.0 (1.37), all under the 1.5 m/s ceiling. |

### Tests (32/32 pass)

| File | What's verified |
|---|---|
| [`src/thermal_mapping/test/test_field_spread.py`](../src/thermal_mapping/test/test_field_spread.py) | 17 tests: pre-ignition ambient; monotonic rise at front; plateau; decay; late-K burnout; bounded fuel disc; wind anisotropy direction. |
| [`src/thermal_mapping/test/test_qi_estimator.py`](../src/thermal_mapping/test/test_qi_estimator.py) | 7 tests: uniform field ⇒ equal $q_i$; one-sector hot ⇒ that sector dominates; threshold-mode behavior. |
| [`src/thermal_mapping/test/test_offline_optimizer.py`](../src/thermal_mapping/test/test_offline_optimizer.py) | 4 tests: FedDCSA on static $q_i$ converges to known KKT point. |
| [`src/thermal_mapping/test/test_baseline_lagrangian.py`](../src/thermal_mapping/test/test_baseline_lagrangian.py) | 4 tests: Lagrangian baseline has same I/O signature; converges under static $q_i$; **per-iterate violation strictly larger than FedDCSA's during transient $q_i$ change** (the paper-claim contrast). |

Run: `pytest src/thermal_mapping/test/`.

## Status

| Feature | State | Notes |
|---|---|---|
| FedDCSA algorithm (2D-faithful) | ✅ | Decaying step $\gamma_k = c_1/\sqrt{k+1}$, decaying gate tolerance $\eta_k = c_2/\sqrt{k+1}$, bounded uniform noise per Remark 1. Output averaging accumulator. |
| Radial-spokes planner | ✅ | $N=8$ spokes per sector, `inner_radius=0.5` m (adaptive for small leashes), cruise = 1.2 m/s. Sweep is OUTBOUND → INBOUND → next spoke → repeat, oscillating across the wedge. |
| 20 Hz rate-limited setpoint streamer | ✅ | Smooths snap-back retractions; firmware tracks the smoothed setpoint. |
| Mock optimizers (constant, sinusoidal) | ✅ | For planner debugging in isolation. |
| RViz visualisation | ✅ | Per-drone leash arcs colored by mode, thermal GridMap, drone TF. |
| One-command bringup | ✅ | `~/cs2_ws/scripts/coverage_restart.sh fed_dcsa\|constant\|sinusoidal` |
| Sector-midpoint spawn (`COVERAGE_4DRONE=1`) | ✅ | Drones boot at sector $\phi_{\text{low}}$ corners, ready for first spoke. |
| Unit tests | ✅ | 13/13 pass: lawnmower state-machine + streamer rate-limit. |
| **Hardware bringup** | ✅ | First HW flight (cf4 single-drone, mocap-driven autonomous hover) 2026-05-17. fed_dcsa 4-drone sim regression 2026-05-18. Multi-drone HW staircase 2026-05-23 (hover) → 2026-05-23/24 (figure-8 + BVC) → 2026-05-25 (Stage 4 fed_dcsa with figure-8 + thermal + BVC + dynamic leash, KKT within 0.07 m). Per-test log: [hw_multi_drone_verification.md §7](hw_multi_drone_verification.md#7-per-test-multi-drone-verification-log). |
| **Dynamic thermal field (Phase A — offline)** | ✅ | `ThermalField.evaluate(x, y, t)` extended with M4 rotating-wind wavefront + burn profile, bounded fuel disc, **firefighter suppression cone** (wind-locked, paper narrative: 1.9 m firebreak with concentrated suppression on cf3/cf4 arc), optional `randomize_seed` for run-to-run dominant-sector rotation. Oracle `qi_estimator.compute_sector_qi`, Lagrangian baseline + `RadialCoverageFedDCSA` head-to-head in `scripts/preview_thermal_scenario.py` / `scripts/animate_thermal_scenario.py`. 37/37 tests pass. YAML locked at `thermal_field.yaml`. |
| **Dynamic thermal field (Phase B — live ROS plumbing)** | ✅ | New nodes: `qi_estimator_node` → `/coverage/sector_weights`, `thermal_ground_truth_node` → `/thermal_truth`. Updated nodes: `thermal_sensor_node` (scenario-anchored `T(x,y,t)`), `thermal_mapper_node` (last-write-wins fusion + `age_seconds` + `last_update_time` layers), `radial_coverage_node` (subscribes to `/coverage/sector_weights`, gated on first-message). Figure-8 planner: `min_leash=0.4` safety floor, `figure8_speed=2.0`. `coverage_restart.sh` defaults to `planner_type:=figure8`. Live sim trajectory reproduces Phase A previewer to 2 decimal places (cf1→1.79, cf2→1.74, cf3→1.14, cf4→0.83 at k=80). |
| **Dynamic thermal field (Phase B-HW — HW verification)** | ⏳ | Pending. Re-run Stage 4 with the new launch on real drones to confirm dynamic-q chase matches sim. |
| **RL policy swap** | ❌ | Placeholder package `src/rl_demo`, not implemented. Will replace `coverage_planner_node.py`+`polar_lawnmower.py`+`quadrant_figure8_node.py` 1-for-1. |

## Next steps

Done so far (history compressed):

- ~~**Hardware bringup.**~~ ✅ 2026-05-25. fed_dcsa 4-drone HW with figure-8 +
  thermal + BVC + dynamic leash; KKT-faithful within 0.07 m of 2D prediction.
- ~~**Phase A — offline dynamic-fire substrate.**~~ ✅ 2026-05-26. M4 model,
  oracle $q_i$ estimator, Lagrangian baseline showing 9× max / 28× mean
  violation contrast vs FedDCSA. YAML locked.
- ~~**Phase B — live ROS plumbing.**~~ ✅ 2026-05-26. qi_estimator_node +
  thermal_ground_truth_node + last-write-wins mapper + gated optimizer +
  figure-8 default with min_leash=0.4 and figure8_speed=2.0. Live sim
  trajectory reproduces Phase A previewer to 2 decimal places.

Remaining (in priority order):

1. **Phase B-HW — dynamic-fire HW verification.** Re-run the Stage 4 procedure
   ([hw_multi_drone_verification.md §7](hw_multi_drone_verification.md#7-per-test-multi-drone-verification-log))
   on real drones with the new launch: qi_estimator + thermal_ground_truth +
   last-write-wins mapper + gated radial_coverage_optimizer + figure8 planner
   with `min_leash=0.4`, `figure8_speed=2.0`. Pre-flight checks unchanged
   (single-marker yaw≈0, channel 100, BVC enabled). Expected: KKT-faithful
   $q \to r$ chase identical to sim. New dated row in
   [hw_multi_drone_verification.md §7](hw_multi_drone_verification.md#7-per-test-multi-drone-verification-log)
   after the test.

2. **RL policy.** Separate planning session — produces a dedicated handoff doc
   (e.g. `docs/rl_policy_handoff.md`) capturing observation space (`local_obs`,
   $r_i^k$, staleness), action space (`/cfN/policy_target`), reward design,
   training environment (CrazySim + dynamic fire substrate from Phase A/B),
   pre-HW gate ([§4.8 Stage 3c BVC sanity test](hw_multi_drone_verification.md#48-stage-3c--bvc-sanity-test)),
   and known sim/HW quirks. Drop-in replacement for `coverage_planner_node` /
   `polar_lawnmower.py` / `quadrant_figure8_node.py`; all topic contracts
   above unchanged.

3. *(Deferred — separate paper-story sessions)*
   - **ADMM baseline** (Story 1 strengthening): adds a stronger distributed
     baseline beyond the current Lagrangian dual ascent. Pending user choice
     on whether to bundle with Phase B-HW or leave for after RL.
   - **Packet-loss / data-drop visualization** (Story 2): injectable drops on
     federated aggregation step + degradation metric. Substrate already
     supports this — no upstream changes needed.

## How to run

**Full demo** (FedDCSA optimizer + spokes planner + thermal mapping + RViz):

```bash
~/cs2_ws/scripts/coverage_restart.sh fed_dcsa
```

The script does teardown → sim bringup with sector spawn (`COVERAGE_4DRONE=1`) → demo
launch in one command. Bringup takes ~90 s; RViz opens automatically.

**Mock optimizers** (for planner debugging):

```bash
~/cs2_ws/scripts/coverage_restart.sh constant      # fixed leash for all drones
~/cs2_ws/scripts/coverage_restart.sh sinusoidal    # sinusoidal leash, validates retraction
```

**Config (single source of truth):**

`src/cf_coverage_planner/config/arena_4drone.yaml`

Holds geometry (sector definitions, $r_i^\star$, $r_i^{\max}$, $q_i$, $c_i$, $B$),
optimizer params ($K$, $T$, $c_1$, $c_2$, `noise_bound`, `round_rate_hz`), planner
params (`num_spokes`, `inner_radius`, `footprint_radius`, `planner_rate_hz`), streamer
params (`max_setpoint_velocity`, `setpoint_rate_hz`), and sensor FOV override.

Both the optimizer launch and the planner launch load the same file.

## Tuning notes

Current values (2026-05-26, post Phase B sim verification — all in
[`arena_4drone.yaml`](../src/cf_coverage_planner/config/arena_4drone.yaml)
unless noted):

**Optimizer / constraint geometry**
- `B = 8` — constraint binds at steady state (cf1+cf2 hit cage edge while cf3+cf4 squeezed).
- `q = [10.0, 1.5, 1.0, 0.5]` — static YAML *fallback* used only until first
  `/coverage/sector_weights` arrives. Live values come from `qi_estimator_node`.
- `r_star = 1.85` uniform across all drones — equal preferred standoff; visual
  asymmetry now comes purely from `q`, not from per-drone `r_star`.
- `r_max = 1.9` uniform — HW cage limit per multi_drone_hw_handoff.md §7.2.
- `c_1 = 0.015`, `c_2 = 0.18`, `noise_bound = 0.015` — diminishing schedules
  $\gamma_k = c_1/\sqrt{k+1}$, $\eta_k = c_2/\sqrt{k+1}$; rescaled from 2D
  paper-scale per Phase A previewer.
- `K = 100`, `T = 5`, `round_rate_hz = 1.0`.

**Planner (figure-8)** — `planner_type:=figure8` is the default for the
federated coverage demo (set in `coverage_restart.sh`).
- `center_alpha = 0.5`, `amp_beta = 0.25` — figure-8 reach ≈ 0.75 × leash.
- `min_leash = 0.4` — planner safety floor. When optimizer leash < 0.4 (e.g.
  pre-fire-ignition), drones fly small figure-8s at (±0.2, ±0.2) instead of
  collapsing to origin. Adjacent worst-case approach 0.20 m sits at the BVC
  margin. Bump to 0.5 if BVC kicks fire during pre-ignition. **Paper claim
  unaffected** — `/coverage/leash` still carries the optimizer's true $r_i^k$.
- `figure8_speed = 2.0` — loop traversal multiplier (peak velocity ≈ 0.92 m/s,
  under the 1.5 m/s streamer ceiling). Ladder verified safe up to 3.0 — see
  [`scripts/test_figure8_speed.py`](../scripts/test_figure8_speed.py).

**Streamer**
- `max_setpoint_velocity = 1.5 m/s` — SAFETY CEILING on the rate-limited setpoint
  walk. NOT a target speed. The planner decides the desired profile; the streamer
  just refuses to overshoot the ceiling between ticks.
- `setpoint_rate_hz = 20.0`.

**Sensor**
- `fov_deg = 35.0` at `altitude = 0.6 m` — footprint 0.38 m × 0.38 m. Widened
  from 15 → 25 → 35° during Phase A for better central coverage under figure-8
  motion. Will be kept at 35° through RL training (sim2real realism: real
  FLIR-class sensors are 50–70°; 35° is already on the narrow side).

**Optimizer first-message gate** — `radial_coverage_node` waits for the first
`/coverage/sector_weights` before running round $k=0$. Prevents the YAML
fallback `q` from polluting the published per-iterate trajectory. With the
gate, $k=0$ runs cleanly against live $q$ (which may be all-zeros at fire
pre-ignition — that's correct).

If a future tweak is needed, prefer editing the YAML rather than the Python — both
launches pick up the new values on restart. **Don't tune in isolation** — re-run
the previewer (`scripts/preview_thermal_scenario.py`) before locking changes,
because Phase A's math substrate is the reference any live sim/HW behavior must
reproduce.
