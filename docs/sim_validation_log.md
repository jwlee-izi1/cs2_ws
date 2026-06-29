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

---

## Test 3 — CBF asymmetric head-on avoidance (first CBF migration engagement) — 2026-06-28 ✅

**Scenario.** Same 2-drone head-on geometry as Test 2, but avoidance moved **off the
firmware and into ROS/Python** and made **asymmetric**: cf2 is a pure **obstacle** (streamed
a straight constant-speed setpoint, firmware colAv is a no-op — run on
`crazyflies_sitl_2drone_nobvc.yaml`, `peer_broadcast_hz:0`), and only **cf1** avoids, via a
Control Barrier Function safety filter. cf1 is modelled as a 2D single integrator; its
desired velocity (P-control toward its goal, capped at `vmax`) is passed through a 2-variable
QP that enforces the barrier, and the filtered velocity becomes a short-lead position
setpoint streamed to `/cf1/cmd_position`. Firmware untouched. Headless; verdict quantitative.

    h(p) = ‖p_ego − p_obs‖² − R²              (h ≥ 0 ⇔ outside the safety disk)
    QP:  min ‖v − v_des‖²
         s.t. 2(p_ego − p_obs)·(v − v_obs) + α·h ≥ 0
    Params: R=0.5 m, α=2.0, vmax=0.4 m/s, lookahead 0.6 s, solver qpsolvers/quadprog
    (single linear constraint ⇒ never infeasible; exact analytic half-space projection as fallback).

**Head-on degeneracy (why `lat`).** In an *exact* head-on (cf1 goal directly behind cf2, all
on y=0) `v_des` points straight through the obstacle centre, the QP has no lateral gradient,
so v_y≡0 — the filter can only **brake**; cf1 stalls/retreats and never circumnavigates
(offline kinematic check: min-sep held but cf1 driven back to x≈−1.5, deadlock). This is the
single-integrator CBF analogue of the BVC `sidestepGoal` that Test 2's firmware used to break
symmetry. The runs break it geometrically with `--lat 0.2` (cf2 goal offset +0.2 m in y), a
near-head-on with a defined pass side.

**Result — CBF on vs the two controls** (run phase; `lat=0.2`):

| Metric | **CBF v_obs ON** | CBF v_obs=0 | **CBF off (bypass)** | _(BVC Test 2)_ |
|---|---|---|---|---|
| Min horizontal separation | **0.523 m** (≥ R) | 0.257 m (**< R**) | 0.087 m | _0.613 m_ |
| Barrier R=0.5 held? | **yes** | no (penetrates) | no (pass-through) | _n/a (0.6 cell)_ |
| Lateral excursion max\|y\| cf1 | 0.386 m | 0.355 m | 0.059 m | _0.567 m_ |
| cf1 reached goal? | yes (1.00, 0.00) | yes | yes (drove through) | _yes_ |
| CBF active ticks | 16 % of run | 11 % | 0 % | _—_ |

**Evidence the CBF (not geometry) did the avoiding:**
1. **on vs bypass min-sep**: 0.523 m vs 0.087 m. With the filter bypassed cf1 drives straight
   through cf2 (8.7 cm ≈ on top of it, like Test 2's BVC-off 0.097 m); with the filter on,
   separation bottoms out exactly at the R=0.5 barrier and no deeper.
2. **`v_obs` matters for a *moving* obstacle**: assuming the obstacle static (`v_obs=0`) the
   barrier is **violated** (0.257 m < 0.5) — the filter reacts too late to a closing target.
   Feeding the EMA-differentiated cf2 velocity restores the guarantee (0.523 m). This is the
   firmware-lag-amplified version of the offline prediction (0.39 m vs 0.51 m kinematic).
3. cf1 trajectory bulges to |y|≈0.39 m around the obstacle then returns to its goal — an
   active detour, not a path that merely missed.

**Verdict.** First ROS/Python CBF engagement **OK**. Holds its R=0.5 m barrier on a moving
obstacle (with `v_obs` on), circumnavigates, and reaches goal; the bypass and `v_obs=0`
controls bracket it (collision / barrier-violation). Asymmetry confirmed: cf2 flew straight
through, all avoidance was cf1's. Note the barrier radius is a **tunable** R (0.5 m here, set
< BVC's 0.6 m geometric cell for comparison) — not a fixed cell wall. Next knobs if needed:
raise α for earlier/firmer braking, raise R for more margin, shrink lookahead if tracking lags.

**Reproduce** (headless): launcher `scripts/sitl_2drone_headon.sh` + crazyswarm2 on
`config/crazyflies_sitl_2drone_nobvc.yaml` (`mocap:=False gui:=false`), driver
`scripts/cbf_headon_test.py`. CSVs + trajectory plot in `/tmp/cbf/`. Three runs:
`--lat 0.2` (v_obs on), `--lat 0.2 --no-vobs`, `--lat 0.2 --bypass`.

---

## Test 4 — CBF exact head-on (`lat=0`) deadlock cured by right-hand bias — 2026-06-28 ✅

**Motivation.** Test 3 only avoided because of `--lat 0.2` (cf2's goal offset in y makes it a
*near*-head-on with a defined pass side). The real HW target is an **exact same-altitude
head-on** (`lat=0`), and there the basic single-integrator CBF degenerates: `v_des` points
through the obstacle centre, the QP has no lateral gradient, so cf1 only **brakes/retreats**
— deadlock, no circumnavigation. Test 4 adds a **right-hand symmetry-breaking bias** so cf1
passes cleanly on `lat=0` without depending on the `lat` crutch.

**Bias design** (`right_bias()` in `scripts/cbf_headon_test.py`; added to the *goal velocity*
only — the QP/barrier is unchanged). When the obstacle is **near and frontal**, perturb
`v_des` sideways, always to the ego's **right**:

    bias = bias_gain · align · prox · right_hat,   then v_des←cap(v_des+bias, vmax) → QP
      align     = max(0, û_des·û_obs)                     heading-into-obstacle cosine (1 head-on, 0 lateral/behind)
      prox      = clip((R+margin − dist)/margin, 0, 1)    1 at/inside R, 0 beyond dist=R+margin (far ⇒ no bias)
      right_hat = (û_obs_y, −û_obs_x)                     û_obs rotated −90° = ego's right
    Params: bias_gain=0.3 m/s, bias_margin=0.7 m (bias arms at dist<R+margin=1.2 m). New flags
    `--bias-gain --bias-margin --no-bias`; `--selftest` runs an offline (no-sim) activation check.

Self-limiting (once cf1 veers right, û_des rotates off û_obs ⇒ align↓ ⇒ bias fades — no
oscillation) and **non-intrusive** (side/far passes have align·prox≈0, so ordinary tracking
is untouched). Sign is **fixed** so the manoeuvre is repeatable on video/HW. Offline
`--selftest` + a closed-loop kinematic check confirmed: strong −y bias at head-on, ~0 for
side/far/behind; noise-free kinematics deadlock at `lat=0` bias-off (cf1→x=−1.5) and reach
goal bias-on.

**Result — headless sim, 4 fresh-stack runs** (`v_obs` on; run+hold phase):

| Metric | **lat=0 bias OFF** (control) | **lat=0 bias ON** (fix) | lat=0.2 bias ON (regress) | far/side-pass bias ON |
|---|---|---|---|---|
| Min horiz separation | 0.474 m (**< R**, penetrates) | **0.518 m** (≥ R, held) | 0.553 m (≥ R) | 0.930 m |
| cf1 min-x (retreat?) | **−1.40 m** (backs up = deadlock) | −1.00 m (no retreat) | −1.00 m | −1.00 m |
| Symmetry-break onset | t≈6.9 s (**late**, noise-driven) | t≈5.2 s (early, commanded) | t≈5.2 s | never (no encounter) |
| max\|y\| cf1 (sidestep) | 0.53 m (−y, **uncontrolled**) | 0.557 m (−y, **right**) | 0.449 m (−y) | 0.060 m (≈straight) |
| Bias-active ticks | 0 % | 17 % | 15 % | 12 % (max\|bias\|=0.08, negligible) |
| Reached goal? | yes (late recovery) | yes | yes | yes |

**Reading the control (the deadlock is real, just noisy).** Pure kinematics deadlock *forever*
at `lat=0` bias-off; in sim, firmware/odom noise eventually tips cf1 off the y=0 knife-edge —
but only after it **retreats to x=−1.40 m** (the brake-and-back-up signature) and breaks the
symmetry **late** (t≈6.9 s), by which point it **penetrates the R barrier** (0.474 < 0.5) and
escapes to an *uncontrolled* side. So "reached goal" here is a degraded, late, barrier-breaking
recovery — not avoidance.

**Evidence the bias fixed it (lat=0 on vs off):**
1. **No retreat**: bias-on cf1 min-x = −1.00 (never backs up) vs −1.40 bias-off. The bias
   gives the QP a lateral gradient *immediately*, so cf1 steps aside instead of stalling.
2. **Barrier restored**: 0.518 m ≥ R bias-on vs 0.474 m < R bias-off. Acting early (t≈5.2 vs
   6.9 s) keeps separation at the R wall instead of letting it dip under.
3. **Controlled, repeatable side**: all bias-on runs pass on **−y** (ego's right), matching the
   `right_hat` sign and BVC Test 2's cf1→−y. Bias-off side is whatever noise picks.
4. **No tracking interference**: far/side pass keeps max\|y\|=0.06 m (≈straight to goal),
   bias peaks at 0.08 m/s and 0 % of CBF-active ticks — align·prox collapses when the obstacle
   isn't frontal. Bias does **not** degrade the previously-working `lat=0.2` case (0.553 m,
   clean −y, still reaches).

**Verdict.** Head-on deadlock **cured**. The right-hand bias turns the `lat=0` exact head-on
from a brake-retreat-late-noise-escape (barrier-violating) into a clean, early, **right-side**
circumnavigation that holds R and reaches goal — without the `--lat` crutch and without
disturbing normal tracking. `lat` is now only a scenario knob, not a correctness crutch.
Trade-off to watch on HW: bias_gain trades sidestep aggressiveness vs how hard it perturbs
tracking; 0.3 m/s (≈0.75·vmax peak) was clean here. Min-sep sits just above R (0.518) — raise
α or bias_gain for more margin if HW lag eats it.

**Reproduce** (headless, fresh stack per run — a landed drone won't re-takeoff until re-armed
by a fresh spawn): launcher `scripts/sitl_2drone_headon.sh` + crazyswarm2 on
`config/crazyflies_sitl_2drone_nobvc.yaml` (`mocap:=False gui:=false`), driver
`scripts/cbf_headon_test.py`. Four runs: `--lat 0.0 --no-bias`, `--lat 0.0`, `--lat 0.2`,
`--lat 3.0`. CSVs + `cbf_bias_trajectories.png` in `/tmp/cbf/`. Offline check: `--selftest`.

---

## Test 5 — CBF robustness to degraded HW peer feed (`v_obs` from 10 Hz / delayed / noisy cf2) — 2026-06-28 ✅

**Motivation.** Tests 3–4 fed the CBF cf2's *true* sim odom: smooth, 30 Hz, noise-free. On
hardware cf1 never sees that — it sees cf2's own EKF estimate relayed through the server hub,
re-broadcast at **10 Hz, position-only, with 2-hop relay latency and Vicon+EKF noise**
(`docs/hw_localization_path.md` §3). Since the peer payload carries no velocity, `v_obs` is
finite-differenced from that rough position. This test asks: **does the barrier survive when
`v_obs` (and `p_obs`) are built from realistically-rough cf2 position instead of perfect sim
state?** — and if not, what fixes it.

**What changed (isolation, not physics).** Gazebo physics and ego-pose injection are untouched
(sim stays perfect-state). A **`PeerFeed` degradation layer** (`scripts/cbf_headon_test.py`)
is inserted on the *single path* "cf2 position → CBF node": rate-downsample to `--peer-hz`
(zero-order hold between), fixed `--peer-delay` (2-hop relay), per-axis Gaussian `--peer-noise-std`,
optional `--proximity-degrade`. The CBF's `p_obs` **and** `v_obs` are built from this degraded
position; cf2's own straight-line obstacle control and the logged separation still use the
**true** position (so min-sep below is ground truth). `--ideal-peer` makes the layer transparent
(50 Hz / 0 / 0) = the original Test-4 path. `--vobs-ema` exposes the v_obs low-pass weight
(lower = stronger filter). Offline `--selftest-peer` measures v_obs roughness with no sim.

> ⚠️ **The degradation params are CONSERVATIVE ASSUMPTIONS, not measured.** The repo pins down
> none of hw's noise σ, relay delay, or close-range degradation (`hw_localization_path.md`
> "Open questions — NO REPO BASIS"). Defaults used: **10 Hz, 100 ms delay, σ=1.5 cm** — one
> plausible set to answer "breaks or holds?". Precise tuning waits for real-flight measurement.

**Offline `--selftest-peer`** (cf2 const-velocity −x @0.4 m/s; v_obs error vs truth):

| feed | v_obs bias | v_obs std | max\|err\| | jitter |
|---|---|---|---|---|
| ideal (50 Hz, 0, 0, ema0.3) | 0.000 | 0.000 | 0.000 | 0.000 |
| degraded (10 Hz, 100 ms, 1.5 cm, ema0.3) | 0.004 | 0.069 | 0.122 | 0.040 |
| degraded + strong LPF (ema0.15) | 0.010 | 0.041 | 0.117 | 0.019 |

Strong LPF cuts v_obs std 40 % / jitter 53 % — but at the cost of more lag (bias↑). That lag is
the catch, as the closed-loop sim shows.

**Result — headless sim, fresh stack per run** (`lat=0`, bias ON, `v_obs` ON; run+hold phase;
true horizontal min-sep, R=0.5 unless noted):

| Run | rate | delay | noise | v_obs filter | **min-sep** | vs R |
|---|---|---|---|---|---|---|
| **A** `--ideal-peer` | 50 Hz | 0 | 0 | ema0.3 | **0.528 m** | ✅ ≥ R (≈ Test 4's 0.518) |
| **B** full degrade | 10 Hz | 100 ms | 1.5 cm | ema0.3 | **0.465 m** | ✗ penetrates R by 0.035 |
| **C** B + strong LPF | 10 Hz | 100 ms | 1.5 cm | **ema0.15** | **0.438 m** | ✗ **worse** (−0.027 vs B) |
| F rate-only | 10 Hz | 0 | 0 | ema0.3 | 0.500 m | borderline |
| D rate+noise | 10 Hz | 0 | 1.5 cm | ema0.3 | 0.506 m | ✅ (noise ≈ no effect) |
| E rate+delay | 10 Hz | 100 ms | 0 | ema0.3 | 0.462 m | ✗ (≈ B — delay alone) |
| **G** full degrade + `--R 0.55` | 10 Hz | 100 ms | 1.5 cm | ema0.3 | **0.510 m** | ✅ ≥ 0.5 (margin fix) |

Plot: `/tmp/cbf/test5_peer_robustness.png` (true sep / |v_obs| / peer-position-error vs t).

**What dominates (ablation F→D→E vs B).**
1. **Rate downsample 30→10 Hz**: minor (0.528→0.500, −0.028). ZOH between broadcasts costs a little.
2. **Noise σ=1.5 cm**: *negligible* (D vs F: 0.500→0.506). Symmetric position noise doesn't bias
   the closing direction, and the bias+EMA absorb the v_obs jitter — even though that jitter is
   real (selftest std 0.069). **Jitter ≠ penetration.**
3. **100 ms relay delay**: the **dominant** factor (E 0.462 ≈ B 0.465). A fixed time delay against
   a fast-changing relative geometry makes `p_obs`/`v_obs` stale exactly at closest approach
   (visible as the peer-error spike during t≈4–8 s in the plot) → the barrier acts late.

**Why filtering harder backfires (C).** A stronger v_obs LPF *smooths* the estimate (C has the
lowest v_obs std of all, 0.076 in-encounter) but **adds lag on top of the relay delay** → min-sep
drops further (0.465→0.438). The failure mode here is *latency*, not noise, so low-pass filtering
treats the wrong problem. **Smoother v_obs ≠ safer barrier.**

**The fix that works (G).** A **design-R margin** directly compensates the latency: raising the
CBF radius 0.50→0.55 (≈ delay·v_rel = 0.1 s · ~0.8 m/s) restores **true** min-sep to 0.510 ≥ 0.5
under the full degraded feed (sidestep grows to max\|y\|=0.60 m, as expected). Equivalent levers:
raise `alpha`, or budget `R_design = R_safety + delay · v_rel,max`.

**Verdict.** Barrier **holds under rough peer input — but not for free, and not via filtering.**
The 10 Hz rate and 1.5 cm noise are tolerable; the 2-hop **relay delay is the one factor that
eats the R margin** (0.528→0.465). v_obs low-pass filtering is *counterproductive* (adds lag).
The correct mitigation is a **latency margin on R/alpha** (R 0.50→0.55 recovered ≥ R). Caveat:
all numbers use **assumed** hw delay/noise (no repo basis) and a single noise seed — once the
real relay delay and σ are measured (`hw_localization_path.md` open questions), set
`R_design = R_safety + measured_delay · v_rel,max` and re-confirm.

**Reproduce** (headless, fresh stack per run): launcher `scripts/sitl_2drone_headon.sh` +
crazyswarm2 on `config/crazyflies_sitl_2drone_nobvc.yaml` (`backend:=cflib gui:=False
mocap:=False`), driver `scripts/cbf_headon_test.py`. Cases: A `--lat 0.0 --ideal-peer`,
B `--lat 0.0`, C `--lat 0.0 --vobs-ema 0.15`, ablation `--peer-delay 0.0` / `--peer-noise-std 0.0`,
fix `--lat 0.0 --R 0.55`. CSVs + `test5_peer_robustness.png` in `/tmp/cbf/`. Offline checks:
`--selftest-peer` (v_obs roughness), `--selftest` (bias activation, unchanged).
