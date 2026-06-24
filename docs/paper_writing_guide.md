# Federated Coverage Paper — Writing Guide

> **Read this first.** This file is a navigation/index doc for a Claude project
> tasked with writing the CoRL paper about the federated coverage system. It
> tells you what artifacts exist, where they live, and which file answers which
> question. It does NOT contain the final result numbers or figures — the
> author will paste those in as they get produced.

## 1. Purpose

- **Who reads this:** a paper-writing Claude project that doesn't have direct
  access to the `~/cs2_ws/` workspace. This guide is the bridge.
- **What it provides:** project navigation, the three-bucket result
  organization, a recommended reading order, and a glossary of load-bearing
  terms.
- **What it does NOT provide:** result numbers, figure images, or the actual
  paper text. Those get added separately as the writing progresses.

## 2. Project motivation and approach

**Problem.** Autonomous robot teams operating in unknown environments must
coordinate sensing and coverage in real time while maintaining reliable
connectivity with a centralized coordinator that aggregates data and builds
a shared map. Each robot has limited communication and sensing budget; the
coordinator has imperfect knowledge of the world; and the world itself is
**non-stationary** — events of interest move, appear, or fade over time.
Existing distributed coordination methods either assume static demand
signals, or use coordination layers that don't guarantee safe, continuously-feasible allocations under non-stationary demand. 

**Approach.** We formulate this as a sequential decision-making problem and
propose a hierarchical control system with three coordinated layers running
at separate timescales:

1. **Demand estimation (slow, ~1 Hz)** — a server-side $q_i$ estimator
   monitors the shared map and infers per-sector task importance $q_i(t)$
   as the world evolves.
2. **Safe distributed allocation (slow, ~1-2 Hz)** — a federated optimizer
   (FedDCSA) consumes $q_i(t)$ and publishes per-robot resource
   allocations (the "leash" $r_i^k$). Key algorithmic guarantee:
   $\sum_i c_i (r_i^k)^2 \le B + \eta_K$ at *every* round, with
   $\eta_K \to 0$ — feasibility holds even mid-chase under non-stationary
   $q_i$.
3. **Policy + low-level control (fast, 5–20 Hz)** — a learned policy
   consumes the leash + local observations and emits drone-level coverage (its job is to balance tracking the evolving thermal dynamcis and explore the new unexplored space that the coordinator gives it )
   commands; a rate-limited streamer (20 Hz) + firmware inner loop
   (~1 kHz on the Crazyflie) track those commands.

**Why per-iterate feasibility matters in this architecture.** Layer 3
consumes every output of layer 2 *immediately* — there is no
"wait-for-convergence" buffer between optimizer and policy. Under
non-stationary $q_i$ the optimizer is *always* mid-chase, so
feasibility-at-convergence is not enough. The per-iterate guarantee is what
makes the layered timescale separation safe in real time.

**Contributions:**

1. **The integrated system.** End-to-end data-plane + demand-estimation +
   safe-allocation + learned-policy + drone-control stack, validated from
   simulation to real Crazyflie hardware.
2. **Algorithmic theorem (FedDCSA per-iterate feasibility).** Formally and
   empirically contrasted against a realistic distributed Lagrangian and admm
   baseline that achieves feasibility only asymptotically — 9× larger max
   violation, 28× larger mean violation on identical conditions under
   non-stationary $q_i$. The algorithm is FedDCSA Algorithm 1: gate-then-project
   with diminishing schedules $\gamma_k = c_1/\sqrt{k+1}$ (primal step) and
   $\eta_k = c_2/\sqrt{k+1}$ (gate tolerance); each round computes aggregate
   $G_k$, gates $b_k = 1$ if feasible within $\eta_k$ else $0$, then runs $T$
   inner gradient steps using $\nabla f$ (feasible branch) or $\nabla g$
   (infeasible branch) with bounded uniform noise per Remark 1.
3. **Graceful degradation under communication failure.** Same per-iterate
   property extends to bounded-violation under federated aggregation drops;
   the learned policy demonstrates compensatory behavior (future work).

**Three-radii vocabulary (always disambiguate):**

| Symbol | Meaning | Static? | Source |
|---|---|---|---|
| $r_i^{\max}$ | Hardware cage limit | static | YAML (`r_max = 1.9 m` for all drones) |
| $r_i^{\star}$ | Per-drone preferred standoff | static | YAML (`r_star = 1.85 m` uniform) |
| $r_i^k$ | Per-round optimizer output (the LEASH) | dynamic | `/coverage/leash` topic |

Never say "radius" without qualifying which one. The paper writer should
treat any ambiguous use as a bug to fix.

## 3. The three experiment buckets

### Bucket A — 2D, static $q_i$ (DONE; functionally background)

**Lives at:** `/home/rk32226/drone-rl-2d/multi_drone/`

**What it shows:** KKT-faithful convergence of FedDCSA under static $q_i$ on
paper-scale geometry. This is the reference implementation the 3D ROS port
was validated against.

**Role in the paper:** this validates **Contribution 2 (the algorithmic
theorem)** on its own at paper-scale geometry under static $q_i$. It's not
the headline experimental section — it's available on demand for any figure
that needs to show "FedDCSA converges to KKT" but the paper's primary story
is Bucket B (and later Bucket C). Frame Bucket A in **Methods / Algorithm
Validation**, not in Results.

**Key files to read (in order):**

1. `multi_drone/experimental_setup_plan.md` — the source-of-truth design doc
2. `multi_drone/optimizer/fed_alloc.py` — canonical FedDCSA algorithm
   implementation (the 3D ROS port is a faithful translation)
3. `leash_only_demo.py` / static-$q_i$ figures — the reference plots

**Result placeholder:**

> "FedDCSA reaches the KKT optimum within X.X cm by k=N rounds under
> [paper-scale geometry, $B$, $q$, $r_{\star}$]." [insert numbers + figure
> when ready]

### Bucket B — Dynamic fire + qi_estimator + Lagrangian baseline contrast (DONE; HEADLINE)

**Lives at:** `/home/rk32226/cs2_ws/` (this workspace; the federated coverage
ROS 2 implementation)

**What it shows:** the headline system showcase — **Contribution 1 (the
integrated system)** running end-to-end with the non-stationary regime that
justifies **Contribution 2 (per-iterate feasibility)**. FedDCSA's
per-iterate feasibility holds under non-stationary $q_i(t)$ produced from a
wind-driven moving fire, while a realistic distributed Lagrangian dual-ascent
baseline exhibits transient violations at each $q_i$ shift. The contrast is
quantitative: **9× larger max violation, 28× larger mean violation** for the
baseline, over the same 6-min run on identical parameters.

**Sub-results, in order of paper-relevance:**

1. **The contrast itself** — per-iterate $\sum_i c_i (r_i^k)^2 - B$ time
   series for FedDCSA vs Lagrangian baseline, with $B$ and $B + \eta_K$
   reference lines.
2. **The chase trajectory** — $r_i^k(t)$ per drone tracking $r_i^{\star}(t)$
   (the frozen-time KKT counterfactual), with bounded lag.
3. **Sim ↔ HW match** — Phase B-HW pending: dynamic-fire HW verification
   re-runs Stage 4 on real drones with the new launch. Until that's done,
   the HW evidence is Stage 4 STATIC-q from 2026-05-25 (KKT within 0.07 m of
   2D prediction on real hardware) — proves the streamer/firmware/BVC stack
   tracks per-iterate $r_i^k$ faithfully.

**Key files to read (in order):**

1. `docs/federated_coverage.md` — the comprehensive project doc (goal,
   algorithm, contracts, dynamic-fire substrate, tooling, status, HW results,
   tuning notes, deferred items). **The single most important file for the
   paper writer.**
2. `src/thermal_mapping/config/thermal_field.yaml` — locked fire model (M4
   rotating-wind wavefront + burn profile, bounded fuel disc, qi.scale=0.18).
   Comments inline defend each value.
3. `src/cf_coverage_planner/config/arena_4drone.yaml` — locked optimizer
   params (`B=8`, `K=100`, `T=5`, `c_1=0.015`, `c_2=0.18`), sector geometry,
   figure-8 planner params. Cite values from here.
4. `src/fed_dcsa/fed_dcsa/radial_coverage_algorithm.py` — top docstring is a
   complete FedDCSA Algorithm 1 description.
5. `src/fed_dcsa/fed_dcsa/lagrangian_baseline.py` — top docstring describes
   the realistic distributed baseline (primal-dual updates, no gate).
6. `scripts/preview_thermal_scenario.py` — generates the headline 7-row
   figure: truth-field snapshots / $q_i(t)$ / $r_i^k(t)$ vs $r_i^{\star}(t)$
   / lag / constraint / drone-reach. Imports the algorithm + baseline +
   field directly. Pure Python, no ROS.
7. `scripts/animate_thermal_scenario.py` — generates the side-by-side
   animation: FedDCSA arena | baseline arena | $\sum c_i r^2$ violation
   panel with red shading on baseline overshoots.
8. `docs/hw_multi_drone_verification.md §7` — per-test HW log (dated entries
   for cf2/cf3/cf4 single-drone, Stage 3a/3b/3c multi-drone, Stage 4 fed_dcsa
   4-drone). Raw evidence for the HW Results section.

**Result placeholders:**

> "FedDCSA's per-iterate violation is bounded by $\eta_K = c_2/\sqrt{K+1}$;
> empirically max violation 0.11 m at $k=80$. The Lagrangian baseline shows
> 9× larger max and 28× larger mean violation on the same run. [insert
> 7-row figure from preview_thermal_scenario.py + bag-derived numbers]"
>
> "HW transfer: Stage 4 (2026-05-25) demonstrated KKT-faithful convergence
> within 0.07 m of the 2D prediction on real Crazyflies under static $q_i$.
> [Phase B-HW pending: dynamic-$q_i$ variant on real drones]"

### Bucket C — RL policy + packet-loss visualization (FUTURE)

**Lives at:** `src/rl_demo/` (currently a placeholder)

**Role in the paper:** completes the system story (**Contribution 1**) by
adding the learned policy layer, and demonstrates **Contribution 3 (graceful
degradation under communication failure)** via the packet-loss visualization.

**What it will show:**

- A **learned planner** replaces the figure-8/lawnmower as a drop-in.
  Observation space: `(local_obs, r_i^k, staleness)`. Action space:
  `/cfN/policy_target` (`geometry_msgs/PoseStamped`). All topic contracts
  upstream of the planner stay the same.
- Under **federated aggregation packet drops**, FedDCSA's gate continues to
  bound per-iterate violation while the baseline diverges. The visualization
  story: **lost temporal frames render BLACK** in the `/thermal_map` overlay
  so the figure shows cause → effect over time: dropped data → black cells
  at that timestamp → baseline's $r_i^k$ drifts (violation grows) vs.
  FedDCSA's $r_i^k$ holds (violation stays bounded).

**Why bundled with RL:** the packet-loss story needs a learned policy that
can compensate for sensor-side data drops. Static figure-8/lawnmower planners
have no such adaptive capacity; their behavior under drops is uninformative.

**Plan to produce results:** documented in (will be) `docs/rl_policy_handoff.md`
— a future planning session.

**Result placeholders:** *"[pending RL training + packet-drop session]"*

## 4. Navigation map — which file answers which question

| Question | File |
|---|---|
| Paper claim + algorithm Methods section | `docs/federated_coverage.md` (Goal + Architecture + Status) |
| FedDCSA Algorithm 1 detail (Methods) | `src/fed_dcsa/fed_dcsa/radial_coverage_algorithm.py` (top docstring) |
| Lagrangian baseline math (Comparison section) | `src/fed_dcsa/fed_dcsa/lagrangian_baseline.py` (top docstring) |
| Tuning values to cite in Experimental Setup | `src/cf_coverage_planner/config/arena_4drone.yaml` (comments inline) and `docs/federated_coverage.md` "Tuning notes" section |
| Fire model defense (Experimental Setup) | `src/thermal_mapping/config/thermal_field.yaml` (comments inline) and `docs/federated_coverage.md` "Dynamic fire substrate" section |
| Oracle $q_i$ estimator math | `src/thermal_mapping/thermal_mapping/qi_estimator.py` and `docs/federated_coverage.md` "Oracle $q_i$ estimator" subsection |
| Topic contracts (for a paper system diagram) | `docs/federated_coverage.md` "Architecture / topic contracts" section |
| HW results | `docs/hw_multi_drone_verification.md §7` (raw log) and `docs/federated_coverage.md` "Hardware bringup" Status row (interpreted summary) |
| Three-radii / glossary | this file, §6 |
| What's deferred / future work | `docs/federated_coverage.md` "Next steps" section |
| 2D reference implementation (Methods validation) | `/home/rk32226/drone-rl-2d/multi_drone/experimental_setup_plan.md` |
| Workspace infrastructure + sim/HW invariants (Experimental Setup language) | `CRAZYSIM_MIGRATION.md` |

## 5. Recommended reading order for the paper writer

- **For the Methods section:**
  `docs/federated_coverage.md` (Goal + Architecture) → `radial_coverage_algorithm.py` top docstring → `arena_4drone.yaml` comments for parameter conventions.
- **For the Experimental Setup section:**
  `thermal_field.yaml` (fire model values + comments) → `scripts/preview_thermal_scenario.py` (the procedure that produced the headline figure) → `scripts/animate_thermal_scenario.py` (the animation pipeline).
- **For the Results section:**
  `docs/hw_multi_drone_verification.md §7` (raw HW evidence) → `docs/federated_coverage.md` "Hardware bringup" Status row (one-line interpretation per result).
- **For the Baseline Comparison section:**
  `lagrangian_baseline.py` top docstring (what the baseline is + why it's the *realistic* distributed comparison) → `preview_thermal_scenario.py` comparison panel output (the 9×/28× contrast figure).
- **For the Discussion / Future Work section:**
  `docs/federated_coverage.md` "Next steps" → Bucket C summary above.

## 6. Glossary

**Three-radii** (see §2 table) — $r_i^{\max}$, $r_i^{\star}$, $r_i^k$.
Never use bare "radius."

**$K$, $\gamma_k$, $\eta_k$** — round index and diminishing schedules.
$\gamma_k = c_1/\sqrt{k+1}$ (primal step size), $\eta_k = c_2/\sqrt{k+1}$
(gate tolerance band).

**Gate-then-project** — FedDCSA's per-round mechanism. Gate decides
feasible vs infeasible branch based on $G_k$ vs $\eta_k$; project means
each inner step uses the gradient of the active branch and clips to
$[0, r_i^{\max}]$. The combination gives **per-iterate feasibility**:
$\sum_i c_i (r_i^k)^2 \le B + \eta_K$ at every $k$, with $\eta_K \to 0$.

**Per-iterate feasibility** — the paper claim. Distinguish from "feasibility
at convergence" (asymptotic — most distributed methods get there eventually).
Under non-stationary $q_i$, the optimizer is always mid-chase, so per-iterate
is the only thing that matters.

**M4 fire model** — the wind-driven anisotropic wavefront model used as the
dynamic substrate. Front speed varies with direction relative to wind:
$v_{\text{eff}}(\theta, t) = v_{\text{front}} \cdot (1 + a \cos(\theta - \theta_w(t)))$
with rotating wind direction $\theta_w(t)$. M3 is the isotropic special case
($a=0$); the locked Phase A uses $a = 0.8$ with $0.5°/$s rotation.

**Rotating wind anisotropy** — the wind direction sweeps through angles
over the run, so the fire's hottest direction shifts through sectors,
producing a moving-target $q_i$. cf1 dominates while wind is +x; then cf2
as wind rotates to +y; then cf3 as it swings to −x.

**Oracle $q_i$** — the per-sector excess heat integrated from ground truth
$T(x,y,t)$. Provides the "ideal" $q_i$ values an estimator could achieve.
The paper deliberately uses an oracle to **isolate the optimizer claim** from
any estimator quality argument.

**Live $q_i$** — what the optimizer actually consumes at each round
(currently from the oracle estimator, eventually from a learned estimator
in the RL session). Topic: `/coverage/sector_weights`.

**BVC** — Buffered Voronoi Cell, the firmware-side onboard collision
avoidance (James Preiss). Enabled on the Crazyflies as a safety net under
both the planner and the future learned policy.

**min_leash floor** — a *planner-side* safety floor (`figure8.min_leash` in
arena_4drone.yaml). When optimizer leash $r_i^k$ is below the floor, the
figure-8 planner uses the floor for its center/amplitude computation
instead of collapsing drones to origin. **Paper claim untouched** —
`/coverage/leash` still carries the optimizer's true $r_i^k$; the floor only
affects how the planner interprets it for safety. Frame in the paper as
"planner-side safety floor decouples optimizer output from drone safety,"
not "we modified the algorithm."

## 7. CRAZYSIM_MIGRATION.md — workspace infrastructure context

This file is included in the paper-writing Claude project's knowledge alongside
this guide. It's the **workspace-level infrastructure doc** (sim/HW
architecture diagram, ROS topic conventions, sim/HW invariants, hardware
bringup philosophy, project index). Use it for:

- Experimental Setup language describing the sim ("ROS 2 + CrazySim
  multi-drone simulator with firmware-in-the-loop crazyswarm2 backend").
- Topic contracts at the **firmware boundary** (`/cfN/cmd_position`,
  `/cfN/odom`) and the sim/HW invariant that the same topics carry the
  same payload on both.
- Justification for the "sim transfers to HW" claim: the doc records that
  the streamer + firmware + BVC stack is identical on sim and HW, so any
  result demonstrated in sim ports to HW with no algorithm change.
- The **§5 project index** if the paper writer needs to know what other
  research projects share the workspace (none of which matter for this
  paper — federated coverage is the relevant one).

Don't go deeper than that — anything paper-specific lives in
`docs/federated_coverage.md`, not in CRAZYSIM_MIGRATION.md.

## 8. What's intentionally NOT in this guide

- **Detailed HW bringup procedure.** The paper cites HW *results*, not the
  bringup steps. `docs/hw_drone_verification.md` and
  `docs/multi_drone_hw_handoff.md` are internal runbooks.
- **Code internals beyond top-of-file docstrings.** The paper doesn't need
  line-level code understanding; the algorithm description is captured in
  the top docstrings of `radial_coverage_algorithm.py` and
  `lagrangian_baseline.py`.
- **Live result numbers / figure images.** Author adds these incrementally
  as figures get produced.
- **Other projects in the workspace.** `payload.md` (payload tilt
  correction), `docs/archive/*` (obsolete), `thermal_mapping_demo.md`
  (data plane setup) are unrelated to this paper.

## 9. Status snapshot (current as of 2026-05-27)

| Bucket | Status | What's missing for the paper |
|---|---|---|
| A — 2D static $q_i$ | ✅ | Figure regeneration if needed; numbers for any "FedDCSA reaches KKT within X m" claim |
| B — Dynamic fire + baseline (sim) | ✅ | The 7-row contrast figure + bag-derived violation numbers |
| B-HW — Dynamic fire on real drones | ⏳ | Phase B-HW session pending; will add a row to `hw_multi_drone_verification.md §7` |
| C — RL + packet-loss | ❌ | Future work section only for now; full results after the RL handoff + packet-drop session |
