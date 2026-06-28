# cs2_ws — Architecture Reference & CrazySim Migration Notes

**Branch:** `crazysim-migration` (created off `main`).
**Date:** 2026-05-05 (last meaningful update 2026-05-15).
**TL;DR:**
- Single-drone hover ✅. 4-drone free-flying takeoff + simultaneous `go_to` ✅.
- Payload world ported (plumbing ✅, hover tuning open).
- **Thermal mapping demo ✅** — see [docs/thermal_mapping_demo.md](docs/thermal_mapping_demo.md). One-command bringup: `~/cs2_ws/scripts/thermal_demo.sh up`.
- **Coverage planner layer ✅** (CoRL federated coverage demo) — see [docs/federated_coverage.md](docs/federated_coverage.md). One-command bringup: `~/cs2_ws/scripts/coverage_restart.sh fed_dcsa`.

If this branch is ever ugly, `git checkout main` reverts everything.

---

# 0. How to use these docs

This workspace hosts **multiple research projects** that share the same infrastructure
(CrazySim, crazyswarm2, hardware bringup). To keep things clean:

- **This file (`CRAZYSIM_MIGRATION.md`)** — workspace + infrastructure only. Architecture,
  sim/HW invariants, hardware bringup checklist, conventions, migration history.
  Project-agnostic.
- **`docs/<project>.md`** — per-project. Goal, packages owned, internal contracts,
  status, next steps, how to run. Project-specific.
- **`cs2_ws/CLAUDE.md`** — short auto-loaded protocol summary for any chat opened in this
  workspace.

## When to update which file

| You are... | Update |
|---|---|
| Starting a new project | Create `docs/<project>.md` (template below). Add a row to the **Project index in §4**. List any new packages in the **Workspace tree in §1.6**. |
| Finishing a feature for a project | Flip the row in that project doc's "Status" table. Update its "Next steps". (Root doc untouched.) |
| Adding a new package to an existing project | Add to that project doc's "Packages" + workspace tree in **§1.6**. |
| Changing infrastructure (sim/HW invariant, hardware step, convention) | Update **this file**. Don't push the change into project docs — link to the relevant section instead. |
| Tuning project parameters | Project doc only. |

**Anti-pattern:** duplicating infrastructure docs (sim/HW invariants, hardware bringup
checklist) inside project docs. Always link to this file's sections.

## Shared modules (one project's package used by another)

Sometimes a package built for project A turns out to be useful for project B (example:
`thermal_mapping` is its own project AND serves as the data plane for the federated
coverage demo). Convention:

- The **first project that owns the package** keeps it in `docs/<that_project>.md`
  as the canonical source of truth.
- Any **other project that uses it** lists it under "Packages" with a one-line note
  and a link to the owning project's doc. No duplication.
- If the package becomes truly **workspace-level infrastructure** (used by 3+ projects,
  or clearly cross-cutting like an optimizer-interface message), **promote** it: add a
  section in this root doc describing the contract; project docs link upward.

**Promotion is the user's decision, not automatic.** Heuristic for when to consider it:

| Number of projects sharing | What to do |
|---|---|
| 1–2 | Stay where it is. Cross-link via project docs. |
| 3 | Consider promotion. *Claude should prompt the user* when this threshold is hit. |
| 5+ | Strongly suggest promotion. |

When working in this workspace, Claude tracks shared-package counts and prompts you
when the threshold is crossed: *"`thermal_mapping` is now used by 3 projects — promote
to root infra?"* You can say yes / no / not yet — promotion isn't automatic.

## Project-doc template (`docs/<project>.md`)

```markdown
# <Project name>

## Goal

One paragraph: what this project is, why it exists, what paper/context it serves.

## Packages

- `src/foo_pkg` — what it does
- `src/bar_pkg` — what it does
(Plus any shared packages this project uses, with a note that they're shared.)

## Architecture / topic contracts (internal to this project)

Block diagram of the message/service flow between this project's packages.
What's hot-swappable, what's throwaway, what survives future changes.

## Status

| Feature | State | Notes |
|---|---|---|
| ... | ✅ / ⏳ / ❌ | ... |

## Next steps

What's open. What the next chat working on this project should pick up.
For broad infrastructure tasks (e.g. "bring up on hardware"), link to
`CRAZYSIM_MIGRATION.md §3` for the generic hardware bringup procedure + checklist.

## How to run

Commands + configuration files. Point at the single-source-of-truth config
file (usually under the package's `config/`).
```

---

# 1. Architecture (what runs where, sim & hardware)

## 1.1 The "one control plane, two backends" idea

This workspace is built so **the application code (planner, RL agent, federated allocation, mapping, perception) is identical between sim and real hardware**. The only thing that changes between flying in Gazebo vs. on real Crazyflies is which backend Crazyswarm2 talks to. Develop in sim, switch one launch arg and a YAML, fly for real.

```
                ┌─────────────────────────────────────────────┐
   YOUR CODE    │   fed_dcsa │ rl_demo │ multinash_cs2_bridge │  unchanged
   (planner /   │   custom mapping/perception nodes you add   │  unchanged
    perception) └─────────────────────────────────────────────┘
                                    │
                                    │ ROS 2 services and topics
                                    │ (crazyflie_interfaces — identical in both modes)
                                    ▼
                ┌─────────────────────────────────────────────┐
   CONTROL     │       crazyflie_server (Crazyswarm2)         │
   PLANE       │   /cfN/{takeoff, go_to, upload_trajectory,   │
                │    start_trajectory, land, arm, ...}         │
                │   /all/{takeoff, land, go_to, start_traj}    │
                │   /cfN/odom, /cfN/scan, /cfN/pose, /tf       │
                └─────────────────────────────────────────────┘
                            │                        │
                       cflib UDP                 cflib radio
                            │                        │
                            ▼                        ▼
        ┌────────────────────────┐    ┌──────────────────────────┐
   SIM  │ cf2 SITL firmware      │    │ real CF2.1 firmware      │ HARDWARE
        │ (1 docker container    │    │ on STM32, USB Crazyradio │
        │  per drone, port 19850 │    │ dongle on host           │
        │  + cf_id)              │    │                          │
        └────────────────────────┘    └──────────────────────────┘
                ▲
                │ UDP CRTP
                ▼
        ┌──────────────────┐
        │ gz_crazysim_     │   ← per drone, Gazebo plugin: bridges firmware
        │  plugin in       │     output (motor PWM) ↔ Gazebo physics ↔
        │  Gazebo Harmonic │     simulated IMU/baro/odom back to firmware
        └──────────────────┘
```

**Key invariant:** every service and topic the application uses (`/cfN/takeoff`, `/cfN/odom`, `/cfN/scan`, `/tf`, etc.) is **published by the same server with the same name and message type** in both modes. Your planner doesn't `if sim: ... else: ...` anywhere.

## 1.2 What's identical sim ↔ hardware

| Surface | Sim | Hardware | Notes |
|---|---|---|---|
| Service `/cfN/takeoff` | ✅ | ✅ | Same `crazyflie_interfaces/srv/Takeoff` |
| Service `/cfN/go_to` | ✅ | ✅ | Same |
| Service `/cfN/upload_trajectory` + `/cfN/start_trajectory` | ✅ | ✅ | Identical CRTP under the hood |
| Service `/all/...` broadcasts | ✅ | ✅ | Single call, all drones |
| Topic `/cfN/odom` (nav_msgs/Odometry) | ✅ | ✅ | Server publishes from firmware EKF state estimate |
| Topic `/cfN/pose` (PoseStamped) | ✅ | ✅ | Same |
| Topic `/cfN/scan` (LaserScan, *if* multiranger deck) | ✅ if model has lidar sensor | ✅ if deck attached | Same message type |
| Topic `/tf` (per-drone TF tree) | ✅ | ✅ | |
| Firmware controller (PID/Mellinger/Brescianini/INDI) | ✅ | ✅ | Literally the same C code in both: SITL is the firmware compiled for x86 |
| `firmware_params` in YAML | ✅ | ✅ | Set posCtlPid/velCtlPid gains; works either side |
| BVC collision avoidance (firmware) | ✅ (after our patch) | ✅ (always) | HW ran it in stock firmware; sim had it disabled until we enabled it — see §1.7 |

## 1.3 What differs (and why)

| Surface | Sim | Hardware |
|---|---|---|
| **Firmware execution** | x86 binary in Ubuntu 22.04 docker container, FreeRTOS Posix port | STM32F405 on board |
| **Sensor source** | Gazebo IMU/baro plugins → cf2 binary via UDP (CRTP) | Real MPU9250 + barometer |
| **Drone URI** | `udp://127.0.0.1:1985N` | `radio://0/80/2M/E7E7E7E7E0+N` |
| **Ground truth pose** | Yes — `/model/cfN/odometry` from Gazebo OdometryPublisher (use for evaluation only, *don't* feed it back into your planner) | No — you only have firmware's EKF estimate |
| **External positioning** | Optional motion capture sim (we have `motion_capture: enabled: false`) | Required for accurate global pose: motion-capture system or Lighthouse |
| **Real-time factor** | ~1× when system is light; payload world drops to <0.5× under solver load | Real-time, always |
| **Failure modes** | Gazebo can crash; container can OOM; ROS daemon can cache stale state | Battery dies; radio link interference; props strike |

## 1.4 Adding new nodes (sensor / mapping / perception) the right way

The rule of thumb: **subscribe to the topics that exist in both modes.** Your node never needs to know whether it's sim or hardware. Concrete patterns:

### Pattern: per-drone occupancy map from multiranger lidar
```python
# my_mapping_node.py — works in sim AND hardware unchanged
self.create_subscription(LaserScan, f'/{drone}/scan', self.on_scan, 10)
self.create_subscription(Odometry,  f'/{drone}/odom',  self.on_odom, 10)
self.create_publisher  (OccupancyGrid, f'/{drone}/map', 10)
```
For sim, the multiranger sensor must be present in the drone SDF (CrazySim's stock `model.sdf.jinja` already has it). For hardware, attach a Multiranger deck. Same code path either way.

### Pattern: evaluate state estimator drift in sim only
```python
# eval_node.py — runs only in sim, compares EKF output to ground truth.
self.create_subscription(Odometry, f'/{drone}/odom',           self.on_est, 10)   # firmware EKF
self.create_subscription(Odometry, f'/model/{drone}/odometry', self.on_gt,  10)   # ground truth (sim only)
# Publish drift metric. Hardware launch doesn't bring up this node.
```
Gate via launch arg or `if not os.environ.get('USE_SIM', '1') == '1': return`. Convention: keep eval-only nodes under a `eval_*.launch.py`.

### Pattern: vision-based perception (camera deck)
Add `<sensor type="camera">` to the sim drone SDF; the node subscribes to `/{drone}/camera/image_raw`. On hardware, add the AI deck or other camera and ensure it publishes the same topic name.

## 1.5 Sim ↔ hardware switching (concretely)

To run your planner against real hardware after sim development:

```bash
# Sim run (current)
ros2 launch crazyflie launch.py \
    backend:=cflib \
    crazyflies_yaml_file:=$HOME/cs2_ws/config/crazyflies_sitl_multi.yaml

# Hardware run (same node code, different config)
ros2 launch crazyflie launch.py \
    backend:=cflib \
    crazyflies_yaml_file:=$HOME/cs2_ws/config/crazyflies_hw.yaml
```

The hardware YAML differs only in:
- `uri: radio://...` instead of `udp://...`
- Optionally `motion_capture: enabled: true, tracking: librigidbodytracker, marker: ...`

Your planner / mapping / perception nodes don't change. The cells in the table below tell you what needs to be true on the hardware side for each capability:

| To use this pattern in HW... | Hardware requirement |
|---|---|
| Pattern 1 `/all/takeoff`, etc. | Just have ≥1 drone enabled in `crazyflies_hw.yaml` |
| Pattern 3 parallel `/cfN/go_to` | Each drone needs a global pose source: motion capture deck/markers OR Lighthouse OR onboard SLAM (Loco/UWB) |
| Pattern 4 `/all/start_trajectory` | Same — drones must know their own pose so the relative trajectory makes sense |
| Multiranger mapping | Multiranger deck on each drone |
| Camera-based perception | AI deck or external camera per drone |

## 1.6 Cheat sheet: the workspace from 30,000 ft

> **Per-node code map (file → `ros2 run` name → topics in/out for every node):**
> see [docs/architecture_map.md](docs/architecture_map.md). The tree below is quick
> orientation; that doc is the exhaustive index. The compact pipeline table follows it.

```
cs2_ws/
├── CLAUDE.md                        ← auto-loaded protocol summary for any cs2_ws chat
├── CRAZYSIM_MIGRATION.md            ← this doc (workspace + infrastructure reference)
├── docs/                            ← per-project docs (see §4 Project index)
│   ├── architecture_map.md          ← code index: every module → file path → topic contract
│   ├── federated_coverage.md        ← CoRL paper project
│   ├── thermal_mapping_demo.md      ← thermal mapping demo (also data plane for federated)
│   ├── payload.md                   ← payload coupling project
│   ├── hw_drone_verification.md     ← canonical per-drone HW verification (workspace infra)
│   ├── hw_multi_drone_verification.md ← canonical multi-drone (staged 2→3→4, figure-8 + BVC)
│   └── multi_drone_hw_handoff.md    ← transitional handoff invoking the two verification docs
├── crazyflie-firmware/              ← CrazySim fork; built into cf2-sitl:22.04 docker image
├── cflib-src/                       ← editable Python install of bitcraze/crazyflie-lib-python
│                                      (was /tmp/cflib-src — moved here for reboot survival)
├── src/
│   ├── crazyswarm2/                 ← CrazySim fork (UDP backend support); main control plane
│   ├── crazyflie-simulation/        ← submodule of crazyswarm2
│   ├── ros_gz_crazyflie/            ← old plugin path; kept for legacy payload_hover.launch.py
│   │
│   ├── cf_payload_world/            ← payload coupling project (sim, both legacy + crazysim variants)
│   │
│   ├── thermal_mapping/             ← thermal demo: thermal_sensor_node + thermal_mapper_node + qi_estimator_node (→ /coverage/sector_weights)
│   ├── thermal_mapping_interfaces/  ←   ThermalFrame.msg
│   │
│   ├── fed_dcsa/                    ← optimizer: radial_coverage_node.py → /coverage/leash (also payload_optimizer_node.py)
│   ├── coverage_optimizer_interfaces/  ← contract: Leash.msg (optimizer→planner hot-swap boundary)
│   ├── cf_coverage_planner/         ← planner(s) → /cfN/policy_target + streamer(s) → /cfN/cmd_position (see architecture_map.md §3.3)
│   ├── baseline_optimizers/         ← mock /coverage/leash publishers: constant_leash_node.py, sinusoidal_leash_node.py
│   │
│   ├── rl_demo/                     ← RL training harness (rl_training/); deploys via cf_coverage_planner rl_planner_node.py
│   └── multinash_cs2_bridge/        ← multinash bridge node (in-progress)
├── config/
│   ├── crazyflies_sitl.yaml         ← single-drone sim
│   ├── crazyflies_sitl_multi.yaml   ← 4-drone free-flying sim (firmware_logging.odom enabled — needed for thermal mapping)
│   ├── crazyflies_payload.yaml      ← 4-drone payload sim (CrazySim path)
│   ├── crazyflies_hw.yaml           ← 8-drone hardware (radio:// URIs)
│   └── crazyflies_sim.yaml          ← legacy plugin-mode sim
├── scripts/
│   ├── thermal_demo.sh              ← one-command bringup for the thermal mapping demo
│   ├── THERMAL_DEMO.md              ← skill doc: bringup + troubleshooting reference (referenced from docs/thermal_mapping_demo.md)
│   └── coverage_restart.sh          ← one-command teardown + sim bringup + coverage demo launch
│                                      (usage: `coverage_restart.sh [fed_dcsa|constant|sinusoidal]`)
└── install/, build/, log/           ← colcon outputs
```

Packages are grouped visually by project: cf_payload_world (payload), thermal_mapping +
thermal_mapping_interfaces (thermal demo), fed_dcsa + coverage_optimizer_interfaces +
cf_coverage_planner + baseline_optimizers (federated coverage). The first three groups
each have a corresponding `docs/*.md`.

**Federated-coverage pipeline → file (the load-bearing circuit).** Each row is one hop;
the arrow is a real ROS topic. Full per-node contracts in
[docs/architecture_map.md](docs/architecture_map.md):

| Stage | File | Output topic (type) |
|---|---|---|
| q_i estimator | `thermal_mapping/.../qi_estimator_node.py` | `/coverage/sector_weights` (Float64MultiArray) |
| **Optimizer** | `fed_dcsa/.../radial_coverage_node.py` | `/coverage/leash` (Leash) — the hot-swap contract |
| Planner (throwaway) | `cf_coverage_planner/.../quadrant_figure8_node.py` (or `coverage_planner_node` / `rl_planner_node`) | `/cfN/policy_target` (PoseStamped) |
| Streamer (survives) | `cf_coverage_planner/.../setpoint_streamer_node.py` (RL path: `receding_horizon_streamer_node.py`) | `/cfN/cmd_position` (Position) → firmware |

RL swap replaces only the planner row; everything at/below `/coverage/leash` is reused.
Config: `cf_coverage_planner/config/arena_4drone.yaml`. Launch:
`scripts/coverage_restart.sh fed_dcsa` → `cf_coverage_planner/launch/coverage_demo.launch.py`.

## 1.7 Collision avoidance (BVC): sim now matches hardware

BVC (Buffered Voronoi Cell) collision avoidance is **firmware that runs on each drone**.
The question of "sim or hardware" is only about *which firmware build turns it on*:

- **Hardware: always on.** The stock firmware compiles and runs `collision_avoidance.c`.
  Our real-drone flights were protected by it out of the box (the "firmware safety net" in
  [docs/hw_multi_drone_verification.md](docs/hw_multi_drone_verification.md)).
- **Sim: off upstream, now patched on.** The upstream CrazySim build *disabled* it — there
  was a literal `// TODO: Add collision avoidance to SITL` in `stabilizer.c`. We enabled it
  in SITL so the simulator behaves like the real drones and the safety net is testable in
  Gazebo before flying.

Three edits, captured as patches in [vendor/](vendor/DEPENDENCIES.md):

| Edit | Sim or HW | What it does |
|---|---|---|
| `sitl_make/CMakeLists.txt` (+`collision_avoidance.c`) | **sim only** | compile the CA module into the SITL firmware (HW build already had it) |
| `src/modules/src/stabilizer.c` (remove `#ifndef CONFIG_PLATFORM_SITL`) | **sim only** | let `collisionAvoidanceUpdateSetpoint()` run in sim (was HW-only) |
| `cflib .../localization.py` (`send_extpos_packed`, `EXT_POSITION_PACKED` ch) | **both** | pack peer `(id,x,y,z)` into one CRTP packet so each drone learns its neighbors' positions — the data BVC consumes; identical mechanism over sim UDP or real radio |
| `crazyswarm2 crazyflie_server.py` (peer broadcast) | **both** | the trigger: server broadcasts each drone its neighbors' poses at `peer_broadcast_hz` (set under `all:` in the crazyflies yaml) via `send_extpos_packed`. Without it the firmware sees no neighbors and BVC does nothing |

The forks themselves are external clones (not in this repo); the patch recipe + pinned
commits are in [vendor/DEPENDENCIES.md](vendor/DEPENDENCIES.md). The three BVC legs span all
three forks — firmware (run it), cflib (pack peers), crazyswarm2 server (broadcast peers).

---

# 2. Status & open work

## 2.1 Sim side

| Capability | State | Notes |
|---|---|---|
| Single-drone hover | ✅ done | z=1.000 ± 0.002 m for 30+ s with the firmware running real PID |
| 4-drone free-flying takeoff | ✅ done | `/all/takeoff`, all 4 reach 1.0 m simultaneously |
| 4-drone parallel `go_to` | ✅ done | Each lands within ±2 cm of target |
| 4-drone `upload_trajectory` + `/all/start_trajectory` | ✅ done | Verified in `four_patterns_demo.py` |
| Payload world plumbing (cf2 ↔ Gazebo ↔ crazyswarm2) | ✅ done | All 4 cf2 containers handshake; services register |
| **Payload world stable hover** | ⏳ open | Drones tumble after unpause. Two paths to try (see [docs/payload.md](docs/payload.md) Next steps) |
| Multiranger lidar in sim drone SDF | ⏳ partial | CrazySim's standard `model.sdf.jinja` includes it; not yet exposed via `/cfN/scan` topic name (sim publishes `/cf_N/lidar` — needs ros_gz_bridge entry) |
| Camera deck simulation | ❌ not started | Add `<sensor type="camera">` to model SDF when you need it |
| **Thermal mapping demo** | ✅ done | 16×16 simulated thermal sensor per drone, shared `grid_map_msgs/GridMap` mapper, RViz GridMap display. Verified end-to-end: all 3 configured hot spots read truth values within noise tolerance. One-command bringup: `~/cs2_ws/scripts/thermal_demo.sh up`. See [docs/thermal_mapping_demo.md](docs/thermal_mapping_demo.md). |
| **Federated coverage layer** (CoRL paper) | ✅ done | FedDCSA optimizer publishes per-drone leash $r_i^k$; per-drone planner converts leash to position targets via radial-spokes policy; rate-limited setpoint streamer drives firmware. 4 drones converge to KKT-faithful asymmetric leashes. One-command bringup: `~/cs2_ws/scripts/coverage_restart.sh fed_dcsa`. See [docs/federated_coverage.md](docs/federated_coverage.md). |

---

# 3. Hardware bringup

This section is the canonical reference for switching the workspace from CrazySim to
real Crazyflies. Generic and project-agnostic — every project that wants to fly on
hardware uses this same procedure.

**Per-drone verification procedure:** see
[docs/hw_drone_verification.md](docs/hw_drone_verification.md) — the
canonical single-marker per-drone verification procedure used for any
drone bringup, post-repair re-verification, or new-drone onboarding.
It is **workspace infrastructure**, not a one-off handoff: §1–6 is the
procedure + debug toolkit, and §7 is the per-drone verification log
(cf1..cfN history of tests, repairs, issues). Multi-drone and
project-specific docs invoke it per-drone.

**Multi-drone verification procedure:** see
[docs/hw_multi_drone_verification.md](docs/hw_multi_drone_verification.md)
— the canonical staged 2 → 3 → 4 multi-drone verification procedure,
sibling to the per-drone doc above. **Workspace infrastructure**:
§1–6 is the procedure + debug toolkit (figure-8 motion planner +
BVC firmware CA + per-second inter-drone spacing logger), §7 is the
per-test log. Use for any multi-drone bringup, demo morning, or
post-repair fleet check.

**Multi-drone phase handoff (1→4 + thermal):** see
[docs/multi_drone_hw_handoff.md](docs/multi_drone_hw_handoff.md) — a
transitional doc that invokes both verification procedures above
(per-drone, then multi-drone staged) and adds the thermal-mapping
demo video on top. Read it when you're moving from single-drone HW to
multi-drone HW.

**Lessons distilled** from the 2026-05-20 → 23 single-drone debug saga
also live as memories: `single-marker-yaw-zero-placement` (yaw≈0
placement requirement) and `hw-radio-link-intermittent` (channel-80
WiFi interference; fix = channel 100).

## 3.0 Known-working configuration (verified 2026-05-17/18)

After significant debugging, the following combination produces successful
autonomous mocap-driven hover:

| Component | Working version / setting |
|---|---|
| **Crazyflie firmware** | **2025.12** (NOT 2026.04 — see Gotcha A below) |
| crazyswarm2 (apt) | `ros-jazzy-crazyflie` 1.0.3 |
| cflib | 0.1.32 (PyPI) — has UDP driver, no editable install needed |
| Mocap | Vicon Tracker 3.10, `librigidbodytracker`, `default_single_marker` (1 marker per drone, centered on top of drone) |
| Controller | `stabilizer.controller: 1` (PID — Mellinger works too with same firmware) |
| Estimator | `stabilizer.estimator: 2` (Kalman, mocap-fused) |
| Other params | `commander.enHighLevel: 1`, `locSrv.extPosStdDev: 0.01`, NO `kalman.resetEstimation` |
| Pre-arm | Need `/cfN/arm` (Arm srv with `arm: true`) BEFORE `/cfN/takeoff` — new in firmware 2024+ |
| Hardware proven | cf4 single-drone hover ✅ (2026-05-17). 4-drone fed_dcsa HW ✅ (2026-05-25 — figure-8 + thermal + BVC + dynamic leash; see [docs/hw_multi_drone_verification.md §7](docs/hw_multi_drone_verification.md#7-per-test-multi-drone-verification-log)) |

### Gotcha A — firmware 2026.04 is BROKEN with current crazyswarm2

Crazyflie firmware **2026.04 removed V1 of the log/param protocol** (PR #1614). The apt
`ros-jazzy-crazyflie` 1.0.3 (packaged 2026-04-12, one day before 2026.04 firmware release)
still uses V1. Result: silent failures — drone connects but firmware EKF doesn't fuse
mocap pose, `stateEstimate.x/y/z` stay at 0, takeoff flies to absolute (0, 0, height)
and crashes into walls.

**Symptom in logs:** `Error no LogEntry to handle id=1` (V1 protocol mismatch),
`platform.send_arming_request is deprecated`.

**Fix:** Downgrade all drones to firmware 2025.12 via cfclient → Service tool. That
version has the V1 protocol AND the critical bugfix PR #1548 ("Kalman: Fix persistent
parameters being overwritten at init") which makes external position fusion actually
work.

When the next ros-jazzy-crazyflie apt release ships (uses V2 log/param), 2026.04+
should work again. Until then, stay on 2025.12.

### Gotcha B — process accumulation between runs

Multiple bringup/teardown cycles leave orphaned ROS nodes (especially `teleop` and
`joy_node`) that survive `pkill -9 -f "ros2 launch"` because they're spawned as
independent children. Memory accumulates; eventually Gazebo starts OOM-crashing.

**Fix:** Use [scripts/nuke.sh](scripts/nuke.sh) — kills everything aggressively
(ROS nodes, Gazebo, docker cf2 containers, ros2 daemon cache reset). `thermal_demo.sh up`
now calls nuke automatically at the start of every bringup. Run `nuke.sh` standalone
whenever the system feels crusty.

## 3.1 Recommended bringup order

**Strategy:** test the infrastructure in isolation before adding application complexity.

1. **Infra smoke test** — bring up the thermal mapping demo against real hardware:
   ```bash
   CFLIES_YAML=$HOME/cs2_ws/config/crazyflies_hw.yaml ~/cs2_ws/scripts/thermal_demo.sh up
   ```
   This is the smallest pipeline that exercises every infra concern: Crazyradio,
   positioning (Lighthouse / mocap), real EKF on `/cfN/odom`, and the shared mapper.
   If any of those is broken, you find out here — no application logic to confuse the
   diagnosis. If it works, you have a working hardware-flying baseline.

2. **Add application stacks on top** — once (1) passes, run the project you care about
   against the same infra. For the federated coverage project (CoRL paper):
   ```bash
   CFLIES_YAML=$HOME/cs2_ws/config/crazyflies_hw.yaml ~/cs2_ws/scripts/coverage_restart.sh fed_dcsa
   ```

3. **Iterate** — start with 1 drone, then 2, then 4. Each step exercises radio bandwidth
   + cross-drone interference + EKF convergence; isolate failures early.

> Both bringup scripts (`thermal_demo.sh` and `coverage_restart.sh`) currently hardcode
> the sim YAML path. Parameterising them to read a `CFLIES_YAML=...` env var is a
> one-line edit at the top of each — **do this once, then the same scripts drive sim
> OR hardware**.

## 3.2 Hardware open-work checklist

The items that need to be true on the hardware side before bringup (1) succeeds:

| Item | State | Why it matters |
|---|---|---|
| **Crazyradio 2.0 / PA dongle setup** | ✅ done (2026-05-17) | udev rule installed at `/etc/udev/rules.d/99-bitcraze.rules` (mode 0666 since LDAP users can't be added to plugdev). cflib reaches Crazyradio without sudo. |
| **`crazyflies_hw.yaml` URI assignment** | ✅ done (2026-05-17) | All 4 CFs assigned unique radio addresses E7E7E7E7E1..E4 (cf1..cf4) via cfclient. |
| **External positioning (mocap or Lighthouse)** | ✅ done (Vicon, 2026-05-17) | Vicon Tracker 3.10 on `192.168.0.62:801`, `librigidbodytracker` with `default_single_marker` (1 retro ball per drone). Mocap → firmware EKF fusion confirmed via `/cfN/odom` matching mocap pose. |
| **`CFLIES_YAML` env-var override in scripts** | ✅ done | Both `thermal_demo.sh` and `coverage_restart.sh` honor `CFLIES_YAML` env var. |
| **Hardware datastream bridge** | ✅ done | `firmware_logging.default_topics.odom @ 20Hz` in `crazyflies_hw.yaml`. `/cfN/odom` publishes for thermal mapper. |
| **Multi-drone radio bandwidth check** | ⏳ deferred to 4-drone HW | First flight was 1-drone (cf4). Single Crazyradio is fine for 4 drones at the rates we use (~80 pkt/s combined). |
| **Battery / safety supervisor** | ⏳ default exists | `voltage_warning: 3.8` / `voltage_critical: 3.7` in YAML. Single-drone supervisor refuses to arm if battery <3.7V — verified during testing. |
| **Drone calibration (motor balance, ESC)** | ✅ done (2026-05-17) | All 4 CFs bench-tested in cfclient: motors spin evenly, props intact, manual hover confirmed. |
| **Firmware version on all 4 drones** | ✅ 2025.12 (NOT 2026.04 — see §3.0 Gotcha A) | Flashed via cfclient Service tool. 2026.04 silently breaks EKF mocap fusion. |
| **`multinash_cs2_bridge` finalisation** | ⏳ in progress | Bridge between your federated planner and Crazyswarm2's services. Owned by the multinash bridge project. |
| **First HW autonomous flight (cf4 hover + land via mocap)** | ✅ done (2026-05-17) | After firmware downgrade + adding `/cfN/arm` step before `/cfN/takeoff`. cf4 hovered at z=0.3m stably. |
| **4-drone HW scale-up** | ✅ done (2026-05-23 hover; 2026-05-23/24 figure-8 + BVC; 2026-05-25 fed_dcsa) | Staged 2→3→4 procedure passed; per-stage log at [docs/hw_multi_drone_verification.md §7](docs/hw_multi_drone_verification.md#7-per-test-multi-drone-verification-log). |
| **fed_dcsa on HW** | ✅ done (2026-05-25) | KKT-faithful convergence within 0.07 m on real HW; BVC enforced; thermal map populated. Same `coverage_demo.launch.py optimizer:=fed_dcsa planner_type:=figure8` flow as sim. |

When each row above flips from ❌/⏳ to ✅, edit it here.

## 3.3 What's NOT going to break sim ↔ hardware switching

Things that work in sim today and will work on hardware unchanged:

- Mapping / perception nodes that subscribe to `/cfN/scan`, `/cfN/odom`, `/cfN/pose`,
  `/tf`. Same topic names + types on both sides.
- All four coordination patterns (broadcast / group_mask / parallel / synced trajectory).
- Federated coverage planner + setpoint streamer (publishes `cmd_position`, which is
  identical sim and HW per §1.2).
- RL training loops emitting any of the above. Firmware doesn't care if it's SITL or
  real silicon.
- Tuning `firmware_params.posCtlPid.*` / `velCtlPid.*` in YAML.

## 3.4 Things that probably WILL need attention when going live

Sim-to-hardware gotchas to expect, even after §3.2 is fully ✅:

- **EKF needs to converge** before drones hover well. On hardware this takes ~1-2 s
  after the first mocap/Lighthouse pose update; in sim it's instant because Gazebo
  feeds perfect ground truth into the firmware's data path. Plan a "warm-up" delay
  in your bringup script.
- **Hover thrust per drone is battery-dependent** on hardware (lower battery = more
  PWM for same lift). Sim gives you a constant. Account for this if your project does
  thrust-based reasoning.
- **Latency** from policy → service call → CRTP packet → motor: in sim ~5-10 ms; on
  hardware over radio 20-50 ms with occasional 100 ms+ stalls. Don't run inner loops
  faster than the slowest realistic latency.
- **Real Crazyflies can crash and break.** Sim drones come back. This shapes how you
  debug — start with one drone in a small space, scale up.

---

# 4. Project index

Each row links to a per-project doc owned by that project. See §0 for the doc protocol.

| Project | Doc | One-line status | Packages |
|---|---|---|---|
| **Federated coverage** (CoRL paper) | [docs/federated_coverage.md](docs/federated_coverage.md) | sim ✅, HW ✅ (2026-05-25) | `fed_dcsa`, `coverage_optimizer_interfaces`, `cf_coverage_planner`, `baseline_optimizers` (uses `thermal_mapping` as data plane) |
| **Thermal mapping demo** | [docs/thermal_mapping_demo.md](docs/thermal_mapping_demo.md) | sim ✅, HW ✅ (2026-05-25; also serves as data plane for federated coverage) | `thermal_mapping`, `thermal_mapping_interfaces` |
| **Payload coupling** | [docs/payload.md](docs/payload.md) | plumbing ✅, stable hover ⏳ | `cf_payload_world` |
| **Multinash bridge** | (no doc yet — package in `src/multinash_cs2_bridge`) | in progress | `multinash_cs2_bridge` |
| **RL policy** | (placeholder — `src/rl_demo`) | not started; will replace lawnmower in `cf_coverage_planner` | `rl_demo` |
| **Sim validation log** | [docs/sim_validation_log.md](docs/sim_validation_log.md) | baseline sim low-level-tracking checks (Test 1 ✅ 2026-06-27) | — (cross-cutting; CrazySim + crazyswarm2 stack) |

**Hardware bringup status:** federated coverage + thermal mapping HW bringup is
**complete** (2026-05-25 — fed_dcsa Stage 4 with figure-8 + thermal + BVC + dynamic
leash; see [docs/hw_multi_drone_verification.md §7](docs/hw_multi_drone_verification.md#7-per-test-multi-drone-verification-log)).
The streamer + firmware + BVC + thermal + mocap stack is verified end-to-end on real
hardware and is now the swap-ready substrate for the next workspace milestone.

**Next major workspace milestone:** the **RL policy swap** in
[`src/rl_demo`](src/rl_demo) — replaces `quadrant_figure8_node.py` /
`polar_lawnmower.py` 1-for-1 (consumes `/coverage/leash`, publishes
`/cfN/policy_target`). Everything below the planner stays untouched.
Pre-RL-HW gate: run **Stage 3c BVC sanity test** from
[docs/hw_multi_drone_verification.md §4.8](docs/hw_multi_drone_verification.md#48-stage-3c--bvc-sanity-test)
to prove the firmware safety net is engaged before flying a learned policy.

**Archived/abandoned projects** (kept for git history, not actively maintained) live
under [docs/archive/](docs/archive/). Notably the surveillance Fed-DCSA variant
(charging stations + swap pairs) lived there before being abandoned; the `fed_dcsa/`
package is now used for the payload tilt correction and the CoRL radial coverage
demo described above.

---

# 5. Migration history (how we got here, what changed and why)

The original `cf_fw_controller` package called `cffirmware` Python bindings directly from a ROS2 node. That hovered for ~5 seconds and then destabilised, because `controllerPid()` is designed to run inside the firmware's full infrastructure (commander, supervisor, complementary filter, anti-windup, attitude estimator). Calling it as a pure function with raw Gazebo state skips all of that.

We pivoted to **CrazySim** (`gtfactslab/CrazySim`), which runs the **full Crazyflie firmware as a SITL process per drone** and bridges it to Gazebo via UDP+CRTP. Same pipeline used by the maintainers for daily work. Crazyswarm2 talks to it through `cflib` over UDP — no code changes needed in your downstream `fed_dcsa`, `rl_demo`, etc.

**Twist:** the SITL `cf2` binary (FreeRTOS Posix port) is broken on Ubuntu 24.04 (the maintainer confirmed this in CrazySim issue #23 — pthread/signal-handling breakage in glibc 2.39+). The official workaround is to **run cf2 inside an Ubuntu 22.04 Docker container** with `--network=host`, while Gazebo Harmonic + the `gz_crazysim_plugin` stay on the host. That's exactly what we do.

---

## Architecture

```
HOST (Ubuntu 24.04, this machine):
  Gazebo Harmonic
    └─ libgz_crazysim_plugin.so (in each drone model)
         binds UDP 1985N (cflib side) + 1995N (firmware side)

  crazyswarm2 server (cflib backend, ROS2)
    └─ talks udp://127.0.0.1:1985N to plugin
         plugin forwards to corresponding cf2 container

CONTAINERS (Ubuntu 22.04, --network=host, 1 per drone):
  cf2-1 ↔ ports 19851/19951
  cf2-2 ↔ ports 19852/19952
  cf2-3 ↔ ports 19853/19953
  cf2-4 ↔ ports 19854/19954
  (cf_id starts at 1 to match cf1..cf4 naming;
   stock CrazySim uses cf_id 0 with port 19850/19950)
```

`--network=host` is the critical flag — host and container share `127.0.0.1`, so loopback UDP just works.

---

## One-time setup (already done on this machine)

> **External forks first.** The `crazyflie-firmware`, `cflib-src`, and `src/crazyswarm2`
> clones are not in this repo (they are gitignored external forks with small local edits).
> Before the steps below, clone all three at their pinned commits and apply the patches per
> [vendor/DEPENDENCIES.md](vendor/DEPENDENCIES.md) — that also copies `Dockerfile.cf2-sitl`
> into `crazyflie-firmware/`. **`deps.repos`'s crazyswarm2 entry is the llanesc `crazysim`
> fork** (UDP backend), not upstream IMRClab.

If you ever set this up on a fresh box, here's the recipe:

1. **Install Docker** (Ubuntu apt):
   ```bash
   sudo apt install docker.io
   sudo systemctl enable --now docker
   ```
2. **Grant yourself docker socket access** (LDAP users can't use `usermod -aG`; ACL works):
   ```bash
   sudo setfacl -m u:$USER:rw /var/run/docker.sock
   ```
   ⚠️ The ACL is **not persistent** across docker daemon restarts. If `docker ps` errors with permission denied later, just re-run the `setfacl` line.
3. **Build the cf2 SITL Docker image:**
   ```bash
   cd ~/cs2_ws/crazyflie-firmware
   docker build --network=host -f Dockerfile.cf2-sitl -t cf2-sitl:22.04 .
   ```
   Two things bite a fresh clone here — both already handled by the files copied in
   from `vendor/` (see [vendor/DEPENDENCIES.md](vendor/DEPENDENCIES.md)), listed so you
   know *why* if you ever regenerate them:
   - **`python: not found`** during the in-container `make` — the firmware's
     `tools/make/versionTemplate.py` is invoked as bare `python`, but `ubuntu:22.04`
     ships only `python3`. **Fix:** the bundled `Dockerfile.cf2-sitl` already installs
     `python-is-python3` (don't re-add it — it's in the vendored Dockerfile).
   - **`git rev-parse HEAD … exit status 128`** while generating `version.c` — the
     bundled `.dockerignore` excludes `.git` (so the 490 MB history isn't shipped into
     the image), but `versionTemplate.py` then can't read the revision from git. **Fix:**
     its intended no-git fallback is a `build_info.json` at the firmware root. Create it
     before building (it's gitignored, so it isn't part of the fork — that's why a fresh
     clone lacks it):
     ```bash
     echo '{"tag": "crazysim-aa6571dc"}' > ~/cs2_ws/crazyflie-firmware/build_info.json
     ```
4. **Build the host-side firmware + Gazebo plugin** (one-time):
   ```bash
   source /opt/ros/jazzy/setup.bash          # REQUIRED before cmake — see note below
   cd ~/cs2_ws/crazyflie-firmware
   mkdir -p sitl_make/build && cd sitl_make/build
   cmake .. && make all -j
   ```
   - **`Could not find … "gz-plugin"` (gz-plugin2 / gz-sim8) at cmake configure** — on
     this box Gazebo Harmonic is installed via the ROS `ros-jazzy-gz-*-vendor` packages,
     so its CMake config files live under the **ROS prefix**
     (`/opt/ros/jazzy/opt/gz_*_vendor/...`), not `/usr`. **Fix:** `source
     /opt/ros/jazzy/setup.bash` *before* cmake so the gz vendor dirs land on
     `CMAKE_PREFIX_PATH`. (This is build-time only; don't add it to `~/.bashrc` — see
     [§ env.sh](#env-sourcing-runtime).)
   - Same bare-**`python`** issue as the Docker build, but on the *host* this time
     (host also has only `python3`). **Fix:** `sudo apt install python-is-python3`. If
     you can't `sudo`, a user-local shim works: `mkdir -p ~/.local/bin && ln -sf
     "$(which python3)" ~/.local/bin/python` (ensure `~/.local/bin` is on `PATH`).
5. **Install `cflib` from source** (the latest pip release lacks the new UDP driver).
   ⚠️ Clone into `~/cs2_ws/cflib-src` (NOT `/tmp/cflib-src`) — `/tmp` is wiped on reboot, which leaves the editable install pointing at a missing path and crazyflie_server crashes with `ModuleNotFoundError: No module named 'cflib'`:
   ```bash
   cd ~/cs2_ws
   git clone --depth=1 https://github.com/bitcraze/crazyflie-lib-python.git cflib-src
   cd cflib-src
   <conda-or-system>/bin/pip uninstall -y cflib
   SETUPTOOLS_SCM_PRETEND_VERSION=0.1.31 <conda-or-system>/bin/pip install -e .
   ```
   ⚠️ Install into **system Python** (or `--break-system-packages` on 24.04, which is
   externally-managed): the ROS `crazyflie_server` imports `cflib` from system Python,
   not from a conda env. The `extpos-packed` patch (BVC) must be present — verify with
   `python3 -c "from cflib.crazyflie.localization import Localization; print(hasattr(Localization,'send_extpos_packed'))"` → `True`.
6. **Keep colcon out of the root forks.** `crazyflie-firmware/` and `cflib-src/` sit at
   the workspace root (siblings of `src/`), so a bare `colcon build` recurses into them
   and aborts with `Duplicate package names not supported: CMSISDSP` (from the firmware's
   `vendor/CMSIS`). They're built outside colcon (cmake in step 4, pip in step 5), so mark
   them ignored — one-time:
   ```bash
   touch ~/cs2_ws/crazyflie-firmware/COLCON_IGNORE ~/cs2_ws/cflib-src/COLCON_IGNORE
   ```
7. **Install ROS dependencies** (`rosdep`). The `src/` packages pull a handful of system
   packages not in a base Jazzy install; without them `colcon` fails (e.g. `crazyflie`
   errors `Could not find … "motion_capture_tracking_interfaces"`):
   ```bash
   source /opt/ros/jazzy/setup.bash
   rosdep update                                              # user-level, no sudo
   rosdep install --from-paths src --ignore-src -r -y         # runs sudo apt under the hood
   ```
   On this box that resolves to 5 packages — install them directly if `rosdep`'s sudo
   step is unavailable:
   ```bash
   sudo apt install -y \
     ros-jazzy-motion-capture-tracking-interfaces \
     ros-jazzy-grid-map-msgs ros-jazzy-grid-map-rviz-plugin \
     ros-jazzy-tf-transformations ros-jazzy-joint-state-publisher-gui
   ```
8. **Build the workspace:**
   ```bash
   cd ~/cs2_ws
   source /opt/ros/jazzy/setup.bash      # gz vendor + ament on CMAKE_PREFIX_PATH
   colcon build --symlink-install
   source install/setup.bash
   ```
   Expect `Summary: 17 packages finished`. For day-to-day sourcing use
   [`env.sh`](#env-sourcing-runtime) instead of the two `source` lines.

   > **Gotcha — empty `baseline_optimizers` executables.** If `ros2 run baseline_optimizers
   > constant_leash_node` later reports `No executable found` (its
   > `install/lib/baseline_optimizers/` is empty), that package's console-scripts didn't get
   > installed — rebuild just it: `colcon build --packages-select baseline_optimizers`. For
   > standalone planner tests you can skip it entirely and publish the leash directly:
   > `ros2 topic pub /coverage/leash coverage_optimizer_interfaces/msg/Leash "{drone_names: ['cf1'], radii: [1.0], gate_state: 1}"`.

<a id="env-sourcing-runtime"></a>
### Sourcing the environment to run (`env.sh`)

Every run terminal needs both the ROS underlay and this workspace's overlay. Instead of
typing two `source` lines per shell, source the bundled `~/cs2_ws/env.sh`:

```bash
source ~/cs2_ws/env.sh        # = source /opt/ros/jazzy/setup.bash + source install/setup.bash
ros2 pkg list | grep crazyflie   # sanity: crazyflie / crazyflie_interfaces / crazyflie_sim …
```

⚠️ **Do NOT add this to `~/.bashrc`** (or any global auto-source). Auto-sourcing ROS in
every shell collides with the conda env we install later, and global env edits have bitten
this box before (the GPU-env / GNOME-login incident). Source `env.sh` explicitly per
terminal. GPU is likewise never set globally — it's applied narrowly via the `gz-gpu`
wrapper only at sim-run time.

---

## How to run

Four flows. Paths below use `$CS2_WS` — set it once per shell to wherever you cloned the
workspace (a variable expands anywhere on the line, unlike `~`; see the note under the
single-drone flow):

```bash
export CS2_WS=$HOME/cs2_ws   # set to wherever you cloned cs2_ws
```

### Thermal mapping demo (one-command, recommended for the multi-drone path)

```bash
$CS2_WS/scripts/thermal_demo.sh up        # full bringup: sim + server + takeoff + thermal mapping + RViz
$CS2_WS/scripts/thermal_demo.sh map       # subscribe /thermal_map, print finite cells + temp range
$CS2_WS/scripts/thermal_demo.sh status    # what's running
$CS2_WS/scripts/thermal_demo.sh down      # tear everything down

# flags:  --no-takeoff   --no-rviz   --no-thermal   --num-drones N
```

The script does explicit pre-flight (docker, cflib, ROS env, image, YAML) then runs T1→T2→T3→T4 with waits at each step, log files in `/tmp/thermal_demo/`. See [scripts/THERMAL_DEMO.md](scripts/THERMAL_DEMO.md) for details and gotchas.

### Single-drone, free-flying

```bash
# Terminal 1 — Gazebo + cf2-0 docker container
source $CS2_WS/env.sh          # gz is a ROS vendor pkg → only on PATH after sourcing
cd $CS2_WS/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh \
    -m crazyflie -x 0 -y 0

# Terminal 2 — crazyswarm2 server (cflib backend)
source $CS2_WS/env.sh
ros2 launch crazyflie launch.py \
    backend:=cflib \
    crazyflies_yaml_file:=$CS2_WS/config/crazyflies_sitl.yaml \
    mocap:=False \
    gui:=false

# Terminal 3 — fly
ros2 service call /cf1/takeoff crazyflie_interfaces/srv/Takeoff \
    "{group_mask: 0, height: 1.0, duration: {sec: 3, nanosec: 0}}"
```

**Why `mocap:=False`:** sim doesn't use motion capture (`crazyflies_sitl.yaml` has
`tracking: off`), but the launch default is `mocap:=True`. With `backend:=cflib` the
`motion_capture_tracking` node's condition (`backend != 'sim' and mocap == 'True'`,
[crazyswarm2 `crazyflie/launch/launch.py`](src/crazyswarm2/crazyflie/launch/launch.py))
evaluates true and launch tries to spawn it — but that package isn't built in this
workspace, so the whole launch aborts with `package 'motion_capture_tracking' not found`.
`mocap:=False` skips the node. Only bites the `backend:=cflib` + sim combo (the `sim`
backend already short-circuits the condition; HW builds the package).

**Why `$CS2_WS` not `~` in `crazyflies_yaml_file`:** a `~` mid-word after `:=` is *not*
tilde-expanded by bash — it's passed literally and launch dies with
`No such file or directory: '~/cs2_ws/config/...'`. A variable like `$CS2_WS` expands
anywhere on the line, so it sidesteps the trap (an absolute path works too). Same applies
to the multi-drone command below.

**Heads-up — planner-driven flights (coverage / figure-8):** the policy planners
(`coverage_planner_node`, `quadrant_figure8_node`, …) only begin publishing
`/cfN/policy_target` once BOTH `/coverage/leash` AND `/cfN/odom` are live. No leash ⇒ the
drone just hovers at its takeoff point — no error, no warning (a quiet failure). Supply a
leash first (an optimizer, or `ros2 topic pub /coverage/leash …`).

### Multi-drone, free-flying (4 drones in a square)

```bash
# Terminal 1 — Gazebo + 4 cf2 containers
source $CS2_WS/env.sh          # gz is a ROS vendor pkg → only on PATH after sourcing
cd $CS2_WS/crazyflie-firmware
bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_multiagent_square.sh \
    -n 4 -m crazyflie

# Terminal 2 — crazyswarm2 server
source $CS2_WS/env.sh
ros2 launch crazyflie launch.py \
    backend:=cflib \
    crazyflies_yaml_file:=$CS2_WS/config/crazyflies_sitl_multi.yaml \
    gui:=false

# Terminal 3
ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff \
    "{group_mask: 0, height: 1.0, duration: {sec: 3, nanosec: 0}}"
```

### Payload world (4 drones rod-coupled to a payload box)

```bash
# Terminal 1 — full stack: Gazebo + 4 containers + crazyswarm2 server
ros2 launch cf_payload_world payload_hover_crazysim.launch.py
# Optional: config:=tilted

# Terminal 2 — coordinated takeoff
ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff \
    "{group_mask: 0, height: 1.25, duration: {sec: 4, nanosec: 0}}"
```

---

## File map (what changed where, on this branch)

| Path | Purpose |
|---|---|
| `cs2_ws/crazyflie-firmware/` | **NEW** clone of `llanesc/crazyflie-firmware` on `crazysim` branch — provides cf2 SITL + Gazebo plugin sources |
| `cs2_ws/crazyflie-firmware/Dockerfile.cf2-sitl` | **NEW** — Ubuntu 22.04 image that builds cf2 |
| `cs2_ws/crazyflie-firmware/tools/.../sitl_singleagent.sh` | **MODIFIED** — wraps cf2 launch in `docker run --network=host`; cleanup also stops containers |
| `cs2_ws/crazyflie-firmware/tools/.../sitl_multiagent_square.sh` | **MODIFIED** — same docker pattern as singleagent |
| `cs2_ws/src/crazyswarm2/` | **REPLACED** with `llanesc/crazyswarm2` `crazysim` branch (UDP backend support); old upstream stashed at `cs2_ws/.crazyswarm2-upstream-backup/` |
| `cs2_ws/config/crazyflies_sitl.yaml` | **NEW** — 1-drone SITL config (UDP `19850`) |
| `cs2_ws/config/crazyflies_sitl_multi.yaml` | **NEW** — 4-drone SITL config (UDP `19850..19853`, but cf_id 0..3) |
| `cs2_ws/config/crazyflies_payload.yaml` | **NEW** — 4-drone payload config (UDP `19851..19854`, cf_id 1..4) |
| `cs2_ws/src/cf_payload_world/worlds/payload_world_crazysim.sdf` | **NEW** — same geometry as `payload_world.sdf` but with `gz_crazysim_plugin` per drone, IMU+baro sensors, `commandSubTopic` on motor models |
| `cs2_ws/src/cf_payload_world/launch/payload_world.launch.py` | **MODIFIED** — added `world_file` launch arg (defaults to legacy `payload_world.sdf`) |
| `cs2_ws/src/cf_payload_world/launch/payload_hover_crazysim.launch.py` | **NEW** — full payload stack: Gazebo + 4 cf2 containers + crazyswarm2 |
| `cs2_ws/src/cf_fw_controller/` | **DELETED** (was the broken direct-cffirmware approach; CrazySim supersedes) |
| `cs2_ws/src/cf_payload_world/worlds/payload_world_fw.sdf` | **DELETED** — cruft from cf_fw_controller era |
| `cs2_ws/src/cf_payload_world/launch/fw_payload_hover.launch.py` | **DELETED** — same |
| `cs2_ws/src/cf_payload_world/scripts/test_tier{1,2}.py` | **DELETED** — were tied to cf_fw_controller |
| `cs2_ws/CRAZYSIM_MIGRATION.md` | this doc |

The `payload_hover.launch.py` (legacy MulticopterVelocityControl path) is **kept** so you can still run the old setup for comparison: `ros2 launch cf_payload_world payload_hover.launch.py`.

---

## Known issues & gotchas

1. **`cf2` doesn't run on Ubuntu 24.04 host directly.** Always go through the `cf2-sitl:22.04` container. CrazySim issue #23.
2. **Docker socket ACL is non-persistent.** If you reboot or restart docker, run `sudo setfacl -m u:$USER:rw /var/run/docker.sock` again.
3. **ROS2 daemon caches stale state.** After killing/restarting `crazyflie_server.py`, run `ros2 daemon stop && ros2 daemon start` so `ros2 service list` and `ros2 node list` show fresh data.
4. **The Gazebo plugin's `socketInit` latches on the first 0xF3 from a cf2.** If you restart cf2 mid-run without restarting Gazebo, the plugin keeps pointing at the old (now dead) ephemeral port — you'll see `cflib echoes 0xFF` work but full handshakes don't. **Always restart the whole stack when in doubt** (kill Gazebo + all cf2 containers, then re-launch).
5. **Each cflib connection holds resources.** If you open and close cflib repeatedly without restarting cf2, the firmware's TOC subsystems can get into a stuck state where new connections hang at "TOC for port [2] found in cache". Workaround: restart cf2 (`docker restart cf2-N`) — though for clean state the full-stack restart is more reliable.
6. **`docker run` from `ExecuteProcess` in launch files needs explicit cleanup.** The `payload_hover_crazysim.launch.py` already has an `OnShutdown` handler that runs `docker stop cf2-*`. If you bypass the launch (e.g. start containers manually), remember to clean them yourself.
7. **Gazebo GUI may show empty / black screen on this machine** due to EGL warnings (NVIDIA driver / DRI2). The simulation runs fine on the server side; you can verify state via `gz topic -e /world/.../stats` or `ros2 topic echo /cfN/odom`.
8. **Single + multi launches use separate-terminal ordering for timing.** Bash scripts (`sitl_singleagent.sh`, `sitl_multiagent_square.sh`) start Gazebo + cf2; you launch crazyswarm2 manually in a second terminal afterwards. The human gate provides the timing. The payload `payload_hover_crazysim.launch.py` combines everything in one `ros2 launch`; it uses a 15-s `TimerAction` to delay the crazyswarm2 server, otherwise the server sends UDP before cf2's done booting and the kernel returns ICMP-unreachable → server crashes with `ConnectionRefusedError`. If you ever build an "all-in-one" single-drone launch, copy that `TimerAction` pattern.
9. **`/tmp` is wiped on reboot** — anything stored there (cflib editable install at `/tmp/cflib-src`, scratch configs) is gone after restart. The cflib editable install in particular silently breaks crazyflie_server with `ModuleNotFoundError: No module named 'cflib'`. Setup step 5 has been updated to clone into `~/cs2_ws/cflib-src` instead.
10. **NEVER use apt's `~n` pattern with `--only-upgrade` on `ros-jazzy-`.** Verified on this machine: `sudo apt install --only-upgrade '~nros-jazzy-'` REMOVED ~250 packages instead of upgrading. Use the explicit form: `sudo apt install -y --only-upgrade $(dpkg -l | awk '/^ii  ros-jazzy-/ {print $2}')`. If you hit this, recover with `sudo apt install -y ros-jazzy-desktop ros-jazzy-grid-map ros-jazzy-grid-map-rviz-plugin ros-jazzy-crazyflie ros-jazzy-crazyflie-py ros-jazzy-crazyflie-examples ros-jazzy-crazyflie-sim ros-jazzy-crazyflie-interfaces ros-jazzy-motion-capture-tracking`.

---

## Payload world: state at handoff & pickup guide

### What was tested at handoff (2026-05-05)

Smoke-test result: **plumbing complete, tuning unfinished.**

Confirmed working:
- All 4 cf2 containers boot inside Ubuntu 22.04, complete the 0xF3 handshake with the
  Gazebo plugin, and reach `Software-in-the-Loop Simulator is up and running!`.
- crazyswarm2 server (with the 15-s `TimerAction` delay) connects to all 4 drones via
  `udp://127.0.0.1:1985N`, registers `/cf1..cf4/takeoff`, `/all/takeoff`, etc.
- World launches **paused** (drones at z=1.25 m suspended); `/all/takeoff` accepted by
  firmware; Gazebo unpauses on `gz service /world/.../control --req 'pause: false'`.

Failed — and why:
- Once unpaused, drones fall. The rod-coupled system has an effective mass of ~40 g per
  drone (28 g body + 12.5 g share of the 50 g payload). Stock CF2.1 firmware tuning was
  calibrated for the free-flying 28 g case. Two compounding failures:
  1. **Transient at unpause:** the firmware integrators were not pre-loaded for the
     extra constant downward force — they need a few seconds to wind up to the new
     equilibrium thrust. During that window gravity wins.
  2. **Solver overload after fall:** once a few drones lose altitude, the ball-jointed
     rods enter ill-conditioned configurations. Gazebo's DART solver hits 360%+ CPU and
     real-time factor collapses — at which point cf2's wall-clock-driven controller
     can't keep up either.

So the integration layer is sound; what remains is **physics/controller tuning**, which
is a research task.

### Likely pickup tasks (in priority order)

1. **Survive the unpause transient.** Two complementary directions:
   - **Pre-arm via setpoint.** Before unpause, send each drone a `/cfN/notify_setpoints_stop`
     followed by a manual setpoint at its spawn pose (`commander.send_position_setpoint`)
     for ~2 seconds. This lets the firmware spin motors up to hover thrust against the
     paused world; once unpaused, gravity is matched immediately. The legacy
     MulticopterVelocityControl path got this for free because cmd_vel→thrust is direct.
   - **Spawn on the ground, not in the air.** Edit `CONFIGS['level']` in
     `payload_world.launch.py` to e.g. z=0.05 (drones touching ground) and have the
     payload rest on the ground too. Then unpause first, takeoff after — no transient.
     Caveat: 4 drones + payload + ground-collision + ball-joint solver may have its
     own startup pain; needs a try.

2. **Tune the firmware PID gains for the coupled-payload dynamics.** Symptoms to expect:
   - Drones lift but oscillate vertically by 5–20 cm.
   - Yaw drift across the payload.
   - One or two drones overshoot while others undershoot, tilting the payload.

   Knobs to try first (in `cs2_ws/config/crazyflies_payload.yaml`, under `all.firmware_params`):
   ```yaml
   posCtlPid:    # outer position loop
     xKp: 1.0   # default ~2.0 — halve to start
     xKi: 0.0
     yKp: 1.0
     zKp: 1.5   # default ~2.0 — slight reduction
   velCtlPid:    # velocity loop
     vxKFF: 0.0 # try disabling feed-forward first
     vyKFF: 0.0
     vzKFF: 0.0
   ```
   Per-drone overrides go under `robots.cfN.firmware_params`. (See
   `crazyflie-firmware/src/modules/src/controller_pid.c` for the param names.)

3. **Physics tweaks.** If solver still struggles after tuning:
   - Reduce IMU update rate from 1000 → 250 Hz (per drone, in
     `payload_world_crazysim.sdf`). Saves CPU.
   - Add joint damping on the rod ball joints (`<damping>` inside `<axis>`).
   - Increase the `<max_step_size>` for physics from 0.001 to 0.002 (look for it in
     `payload_world_crazysim.sdf`'s physics block — currently inherited from default).

2. **Validate `go_to` with rod-constrained motion.** Once stable hover is achieved, command all drones to the same horizontal displacement (`Δx = +0.3 m`). Verify the payload follows without rod bind. Also try a small yaw-only setpoint and a small tilt setpoint.

3. **Re-do the `test_tier1.py`/`test_tier2.py` style benchmarks** using the new pipeline. The old test scripts were deleted because they were specific to `cf_fw_controller`; you'll want a fresh script that calls `/cfN/takeoff`, `/cfN/go_to`, then samples `ros2 topic echo /cfN/odom` for metrics. The patterns from `cs2_ws/src/cf_fw_controller/scripts/test_controller.py` (deleted but visible in `git log`) can be a starting point.

4. **Optionally port `fed_dcsa` and `rl_demo` to point at this payload setup.** Their service interfaces (`/cfN/takeoff`, `/cfN/go_to`, `/cfN/upload_trajectory`) are 100 % identical between the legacy MulticopterVelocityControl backend and the CrazySim backend, so the migration should be a no-op — just point those projects at the new launch file.

### Verification block (run on next session, fill in result)

```text
[ ] Stack starts: `ros2 launch cf_payload_world payload_hover_crazysim.launch.py`
    → all 4 containers Up, /cf1..cf4/takeoff services available within 90 s.
[ ] Coordinated takeoff: `/all/takeoff` to 1.25 m → all 4 drones reach 1.25 m ± ?
[ ] 30-second hover: max(z) - min(z) over 30 s = ? cm  (target < 10 cm)
[ ] Payload z stable: payload z = 1.21 m ± ? cm  (target < 5 cm)
[ ] Per-drone go_to: cf1 to (Δx=+0.1, Δy=0, z=1.25) → followed by others, no rod NaN.
```

---

## Rollback

If anything explodes:

```bash
cd ~/cs2_ws
git stash      # any uncommitted work
git checkout main
# Done. crazyflie-firmware/ and crazyswarm2/ are back on upstream;
# cf_fw_controller restored from main; payload world reverts.
```

The CrazySim work survives on the `crazysim-migration` branch — `git checkout crazysim-migration` brings it back.
