# cs2_ws — Multi-Project Crazyflie ROS 2 Workspace

A ROS 2 (Jazzy) workspace for multi-drone research using Bitcraze Crazyflie 2.1
quadrotors with Gazebo Harmonic simulation, built on **CrazySim** (full Crazyflie
firmware running as SITL in Docker, bridged to Gazebo) + Crazyswarm2. The same node
code runs in sim and on real hardware unchanged.

For workspace architecture, sim/HW invariants, hardware bringup, and conventions see
**[CRAZYSIM_MIGRATION.md](CRAZYSIM_MIGRATION.md)**. For the doc-system protocol see
**[CLAUDE.md](CLAUDE.md)**.

---

## Active projects

### 1. Federated Coverage (CoRL paper)
**Doc:** [docs/federated_coverage.md](docs/federated_coverage.md) ·
**Packages:** `fed_dcsa`, `coverage_optimizer_interfaces`, `cf_coverage_planner`,
`baseline_optimizers` (uses `thermal_mapping` as data plane)

Distributed resource allocation with persistent safety guarantees. A federated
optimizer publishes per-drone leashes $r_i^k$ at every round; per-drone planners
turn the leash into Crazyswarm2 position commands via a polar coverage policy
(being swapped for a learned RL policy — a work in progress, see below). The aggregate constraint
$\sum_i c_i r_i^2 \le B$ holds at every algorithmic iterate.

```bash
~/cs2_ws/scripts/coverage_restart.sh fed_dcsa
```

### 2. Thermal Mapping Demo
**Doc:** [docs/thermal_mapping_demo.md](docs/thermal_mapping_demo.md) ·
**Packages:** `thermal_mapping`, `thermal_mapping_interfaces`

Multi-drone shared occupancy/heatmap demo. Each drone synthesises a 16×16 thermal
sensor frame over a configured ground-truth field; a central mapper accumulates per-cell
mean and publishes a unified `grid_map_msgs/GridMap`. Serves both as a standalone demo
and as the data plane for the Federated Coverage project.

```bash
~/cs2_ws/scripts/thermal_demo.sh up
```

### 3. Payload Coupling
**Doc:** [docs/payload.md](docs/payload.md) ·
**Package:** `cf_payload_world`

4 Crazyflies cooperatively suspend a 200 g rectangular payload via ball-jointed rods.
Plumbing complete; stable hover tuning open. Validates persistent-safety constraint
enforcement against coupled multi-body physics.

```bash
ros2 launch cf_payload_world payload_hover_crazysim.launch.py
```

---

## In progress / placeholder

- **MultiNash bridge** (`src/multinash_cs2_bridge`) — bridge between Crazyswarm2 and an
  external Multi-Nash game-theoretic planner. No doc yet.
- **RL policy** (`src/rl_demo`) — a learned policy that replaces the hand-coded
  lawnmower/figure-8 planner in `cf_coverage_planner` (deployed through
  `rl_planner_node`). **Work in progress** — a semi-working PPO policy trains and
  runs in sim today; we're actively improving it.

Archived/abandoned projects (kept for git history) live under
[`docs/archive/`](docs/archive/).

---

## Workspace structure

```
cs2_ws/
├── CLAUDE.md                       # Auto-loaded doc protocol for any chat in this workspace
├── CRAZYSIM_MIGRATION.md           # Workspace + infrastructure reference (start at §0)
├── README.md                       # This file (GitHub-facing intro)
├── docs/                           # Per-project docs
│   ├── federated_coverage.md
│   ├── thermal_mapping_demo.md
│   ├── payload.md
│   └── archive/                    # Abandoned/legacy docs
├── src/                            # ROS 2 packages (see CRAZYSIM_MIGRATION.md §1.6)
├── config/                         # Crazyflie YAML configs (sim + hardware)
├── scripts/
│   ├── thermal_demo.sh             # Thermal mapping demo bringup
│   ├── coverage_restart.sh         # Federated coverage demo bringup
│   └── THERMAL_DEMO.md             # Thermal demo skill doc
├── crazyflie-firmware/             # CrazySim fork (cf2 SITL + Gazebo plugin)
└── cflib-src/                      # Editable cflib install
```

---

## Setup

### Prerequisites
- ROS 2 Jazzy
- Gazebo Harmonic
- Docker (for cf2-sitl:22.04 container — see CRAZYSIM_MIGRATION.md §4 "One-time setup")
- Python 3.10+

### Build

```bash
git clone git@github.com:ramank3/cs2_ws.git
cd cs2_ws

# External dependencies fetched by vcstool (crazyswarm2 = llanesc fork, crazyflie-simulation)
vcs import < deps.repos
cd src/crazyswarm2 && git submodule update --init --recursive && cd ../..

# IMPORTANT: the crazyflie-firmware and cflib forks are NOT fetched by vcs, and the
# crazyswarm2 fork has a local edit. Clone all three at their pinned commits and apply
# the patches per  vendor/DEPENDENCIES.md  before continuing. That recipe also copies
# Dockerfile.cf2-sitl into crazyflie-firmware/ (it lives in vendor/ because it is
# gitignored inside the fork — a fresh clone will NOT contain it).

# ROS deps
rosdep install --from-paths src --ignore-src -r -y

# Build cf2 SITL Docker image (one-time) — needs Dockerfile.cf2-sitl copied in per vendor/DEPENDENCIES.md
docker build --network=host -f crazyflie-firmware/Dockerfile.cf2-sitl -t cf2-sitl:22.04 crazyflie-firmware/

# Build the workspace
colcon build --symlink-install
source install/setup.bash
```

See [CRAZYSIM_MIGRATION.md §4 "One-time setup"](CRAZYSIM_MIGRATION.md) for the full
recipe (Docker socket ACL, cflib editable install, etc.).

---

## External dependencies

| Package | Source | Purpose |
|---|---|---|
| [crazyswarm2 (CrazySim fork)](https://github.com/llanesc/crazyswarm2/tree/crazysim) | llanesc | ROS 2 Crazyflie driver with UDP backend for CrazySim |
| [crazyflie-simulation](https://github.com/bitcraze/crazyflie-simulation) | Bitcraze | URDF/mesh assets |
| [crazyflie-firmware (CrazySim fork)](https://github.com/llanesc/crazyflie-firmware/tree/crazysim) | llanesc | Crazyflie firmware compiled as SITL Linux binary + Gazebo plugin |
