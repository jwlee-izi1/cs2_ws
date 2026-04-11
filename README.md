# cs2_ws — Multi-Project Crazyflie ROS2 Workspace

A ROS2 (Jazzy) workspace for multi-drone research using Bitcraze Crazyflie 2.1 quadrotors with Gazebo Harmonic simulation. This repo contains three research projects sharing a common Crazyflie simulation and control stack.

---

## Projects

### 1. Distributed Surveillance — Fed-DCSA
**Package:** [`src/fed_dcsa/`](src/fed_dcsa/)

Federated Distributed Constraint-Satisfaction Algorithm for multi-drone surveillance coverage. Drones are assigned to surveillance stations with battery-aware scheduling and autonomous swapping.

- **Core algorithm:** `fed_dcsa/algorithm.py` — server-coordinated gate, projected gradient descent
- **Coordinator node:** `fed_dcsa/coordinator_node.py` — orchestrates 4 stations, each with active+standby drone pair
- **Config:** `config/fed_dcsa_params.yaml` — algorithm parameters (K, T, tau, energy rates)
- **Experiments:** `scripts/run_numerical.py`, `run_drone_simulation.py`
- **Docs:** `docs/parameter_tuning_guide.md` — comprehensive parameter relationships and tuning

```bash
# Numerical analysis (no simulation needed)
python3 src/fed_dcsa/scripts/run_numerical.py

# Full Gazebo simulation
ros2 launch fed_dcsa fed_dcsa_experiment.launch.py
```

### 2. Game-Theoretic Planning — MultiNash
**Package:** [`src/multinash_cs2_bridge/`](src/multinash_cs2_bridge/)

Bridge between Crazyswarm2 and an external Multi-Nash game-theoretic planner for cooperative multi-drone motion planning via Nash equilibrium seeking.

- **Bridge node:** `multinash_cs2_bridge/multinash_bridge.py` — fetches drone poses via TF, calls external planner
- **Execution modes:**
  - `multinash_exec_cf1.py` — single drone
  - `multinash_exec_two_cf_mnash.py` — two drones with MultiNash
  - `multinash_exec_N_cf_mnash.py` — N drones with MultiNash
  - `multinash_exec_N_cf_traj.py` — N drones, trajectory mode

> **Note:** Requires external planner repo (`DronePotentialGame`) and a separate virtualenv (`.venv_multinash`).

### 3. Cooperative Payload Transport
**Package:** [`src/cf_payload_world/`](src/cf_payload_world/)

Gazebo world with 4 Crazyflies carrying a rigid rectangular payload connected via ball-jointed rods. Used to validate coordinated hovering and tilt correction algorithms.

- **World:** `worlds/payload_world.sdf` — 4 drones, 4 rods, 1 payload (50g)
- **Launch:** `launch/payload_hover.launch.py` — Gazebo + bridge + 4 control nodes
- **Configurations:** level (all drones same height) and tilted (~20 deg tilt)
- **Test script:** `scripts/test_payload_hover.py` — sends go_to commands, records and plots z-evolution
- **Tuning log:** `TUNING_LOG.md` — SDF parameters and ROS controller gains

```bash
# Launch payload hover (level configuration)
ros2 launch cf_payload_world payload_hover.launch.py mode:=level

# Launch payload hover (tilted configuration)
ros2 launch cf_payload_world payload_hover.launch.py mode:=tilted
```

---

## Workspace Structure

```
cs2_ws/
├── src/
│   ├── fed_dcsa/                  # Project 1: Surveillance / resource allocation
│   ├── multinash_cs2_bridge/      # Project 2: Game-theoretic planning
│   ├── cf_payload_world/          # Project 3: Payload transport
│   ├── crazyswarm2/               # [external] ROS2 Crazyflie driver stack
│   ├── ros_gz_crazyflie/          # Modified fork — Gazebo integration + payload control tuning
│   └── crazyflie-simulation/      # [external] Meshes and URDF descriptions
├── config/                        # Crazyflie YAML configs (hw + sim)
├── exp1/                          # Experiment 1 results (CSVs, GIFs)
├── exp2/                          # Experiment 2 results (CSVs)
├── deps.repos                     # External dependency manifest
└── README.md
```

---

## Setup

### Prerequisites
- ROS2 Jazzy
- Gazebo Harmonic
- Python 3.10+
- [`vcs` tool](https://github.com/dirk-thomas/vcstool) (`pip install vcstool`)

### Clone and build

```bash
# Clone the repo
git clone git@github.com:ramank3/cs2_ws.git
cd cs2_ws

# Fetch external dependencies
vcs import < deps.repos

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

---

## ros_gz_crazyflie (Modified Fork)

This repo includes a modified version of [knmcguire/ros_gz_crazyflie](https://github.com/knmcguire/ros_gz_crazyflie) with the following additions to `control_services.py`:

- `kd_z` / `kd_xy` derivative damping parameters (stock only has proportional)
- Auto-hover on startup (holds spawn position on first odom message)
- `max_vel_z` clamping (2.0 m/s)
- Runtime-tunable gains via `ros2 param set`

These changes are required for stable payload hovering. See [`src/cf_payload_world/TUNING_LOG.md`](src/cf_payload_world/TUNING_LOG.md) for details.

---

## External Dependencies

Fetched automatically via `deps.repos`:

| Package | Source | Purpose |
|---------|--------|---------|
| [crazyswarm2](https://github.com/IMRCLab/crazyswarm2) | IMRCLab | ROS2 Crazyflie driver, interfaces, Python API |
| [crazyflie-simulation](https://github.com/bitcraze/crazyflie-simulation) | Bitcraze | URDF/mesh assets |
