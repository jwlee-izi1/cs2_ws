# Fed-DCSA Orchestrator: Gazebo Control Layer Context

This document provides all the context an AI agent needs to build a Python orchestrator that replays Fed-DCSA algorithm output as a Gazebo drone demonstration.

---

## Mission

Build a **Python orchestrator script** that:

1. Reads pre-computed Fed-DCSA algorithm output (round-by-round sector time allocations and drone swap decisions)
2. Replays selected rounds in Gazebo via ROS 2 service calls
3. Demonstrates: drones flying from corner charging stations to assigned survey sectors, hovering for allocated time, returning, and optionally swapping with standby partners

The algorithm (Fed-DCSA) is a federated distributed constrained stochastic approximation that runs **offline first** (Option A). The orchestrator is a **replay layer** that drives the existing Gazebo simulation.

---

## Arena Geometry

```
4x4m arena with walls at +/-2m. Origin at center.

Sector centers (2x2m survey area divided into 4 quadrants):
  sector 1: (-0.5, -0.5)    sector 2: (+0.5, -0.5)
  sector 3: (-0.5, +0.5)    sector 4: (+0.5, +0.5)

Charging stations (orange pads at corners):
  station 1: (-1.5, -1.5)   station 2: (+1.5, -1.5)
  station 3: (-1.5, +1.5)   station 4: (+1.5, +1.5)
```

## Drone Assignments

| Station | Active Drone | Standby Drone | Station XY |
|---------|-------------|---------------|------------|
| 1 | cf1 | cf5 | (-1.5, -1.5) |
| 2 | cf2 | cf6 | (+1.5, -1.5) |
| 3 | cf3 | cf7 | (-1.5, +1.5) |
| 4 | cf4 | cf8 | (+1.5, +1.5) |

Active drones start at (station_x, station_y - 0.1, 0.03).
Standby drones start at (station_x, station_y + 0.1, 0.03).

Each station_i is associated with sector_i. Drone at station_i is the primary surveyor for sector_i, but the algorithm may assign any drone to any sector.

---

## ROS 2 API (What the Orchestrator Needs)

The simulation must be running before the orchestrator starts:
```bash
cd ~/cs2_ws && source install/setup.bash
ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py
```

Wait ~5 seconds for all nodes to receive odometry before calling services.

### Takeoff

```bash
ros2 service call /{drone}/takeoff crazyflie_interfaces/srv/Takeoff \
  "{group_mask: 0, height: <meters>, duration: {sec: <s>, nanosec: 0}}"
```

- `height`: target altitude (use staggered heights per drone, see below)
- `duration`: time to reach height (2-3s typical)
- After completion, drone enters HOVERING state at that altitude

### GoTo (primary navigation command)

```bash
ros2 service call /{drone}/go_to crazyflie_interfaces/srv/GoTo \
  "{group_mask: 0, relative: false, goal: {x: <x>, y: <y>, z: <z>}, yaw: 0.0, duration: {sec: <s>, nanosec: 0}}"
```

- `relative: false` for absolute world coordinates (always use this)
- `goal`: target position in meters
- `yaw`: target heading in **degrees** (use 0.0 for no rotation)
- `duration`: transit time in seconds
- After completion, drone enters HOVERING state at target
- Target is **geofence-clamped** to [-1.9, -1.9, 0.0] to [1.9, 1.9, 2.0]

### Land

```bash
ros2 service call /{drone}/land crazyflie_interfaces/srv/Land \
  "{group_mask: 0, height: 0.0, duration: {sec: 3, nanosec: 0}}"
```

- After completion, drone enters IDLE state (motors off, on ground)
- Only call this when a drone is being swapped out or the demo ends

### Hover (hold position)

```bash
ros2 service call /{drone}/hover std_srvs/srv/Empty
```

- Immediately freezes drone at current position
- Useful as a safety stop

### NotifySetpointsStop (cancel trajectory)

```bash
ros2 service call /{drone}/notify_setpoints_stop \
  crazyflie_interfaces/srv/NotifySetpointsStop \
  "{group_mask: 0, remain_valid_millisecs: 0}"
```

- Cancels any in-progress trajectory, transitions to HOVERING at current position

### Status Topic (monitor drone state)

```bash
ros2 topic echo /{drone}/status --once
```

Returns a string: `"state:HOVERING remaining:0.000 traj_id:-1"`

**States:**
- `IDLE` — on ground, not flying
- `TRAJECTORY` — executing a movement command (takeoff/go_to/land)
- `HOVERING` — stationary in air at a setpoint

**To detect arrival:** After sending `go_to`, poll status until `state:HOVERING`.

---

## Duration Guidance

The control node clamps horizontal velocity to `max_vel_xy = 0.3 m/s`. If the requested duration is too short for the distance, the drone lags behind and takes extra time to converge.

**Formula:**
```
distance = sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)
duration_seconds = distance / 0.3 + 2.0   # 2s buffer for smoothstep ramp
```

**Common distances in this arena:**

| Route | Distance | Recommended Duration |
|-------|----------|---------------------|
| Station to own sector center | ~1.4m | 7s |
| Station to diagonal sector center | ~2.8m | 12s |
| Station to adjacent sector center | ~2.1m | 9s |
| Sector center to sector center | ~1.0m | 6s |

---

## Staggered Heights (CRITICAL)

Multiple drones sharing airspace MUST fly at different altitudes to avoid physics collisions in Gazebo.

**Recommended assignment:**

| Drone | Flight Height |
|-------|--------------|
| cf1 / cf5 | 0.5m |
| cf2 / cf6 | 0.7m |
| cf3 / cf7 | 0.9m |
| cf4 / cf8 | 1.1m |

Use these heights for both takeoff and all go_to commands. When a standby replaces an active drone, it inherits the same flight height.

---

## Status Polling Pattern

```python
import subprocess
import time

def wait_for_hover(drone, timeout=30):
    """Block until drone reports HOVERING state."""
    start = time.time()
    while time.time() - start < timeout:
        result = subprocess.run(
            ["ros2", "topic", "echo", f"/{drone}/status", "--once"],
            capture_output=True, text=True, timeout=10
        )
        for line in result.stdout.split("\n"):
            if "state:HOVERING" in line:
                return True
        time.sleep(0.5)
    return False

def wait_for_idle(drone, timeout=15):
    """Block until drone reports IDLE state (landed)."""
    start = time.time()
    while time.time() - start < timeout:
        result = subprocess.run(
            ["ros2", "topic", "echo", f"/{drone}/status", "--once"],
            capture_output=True, text=True, timeout=10
        )
        for line in result.stdout.split("\n"):
            if "state:IDLE" in line:
                return True
        time.sleep(0.5)
    return False

def call_service(drone, service, srv_type, args=""):
    """Call a ROS 2 service via CLI."""
    cmd = f'ros2 service call /{drone}/{service} {srv_type} "{args}"'
    subprocess.run(cmd, shell=True, capture_output=True, timeout=15)
```

---

## Orchestrator Workflow (Pseudocode)

```python
# 1. Load algorithm output
rounds = load_algorithm_output("fed_dcsa_output.json")

# 2. Initial state: active drones at stations
active = {"station_1": "cf1", "station_2": "cf2", "station_3": "cf3", "station_4": "cf4"}
standby = {"station_1": "cf5", "station_2": "cf6", "station_3": "cf7", "station_4": "cf8"}
heights = {"cf1": 0.5, "cf2": 0.7, "cf3": 0.9, "cf4": 1.1,
           "cf5": 0.5, "cf6": 0.7, "cf7": 0.9, "cf8": 1.1}

# 3. For each round to demonstrate
for round_data in rounds:
    print(f"=== Round {round_data['round']} ===")

    # 3a. Takeoff all active drones
    for station_id, drone in active.items():
        h = heights[drone]
        call_service(drone, "takeoff", "crazyflie_interfaces/srv/Takeoff",
                     f"{{group_mask: 0, height: {h}, duration: {{sec: 3, nanosec: 0}}}}")
    for drone in active.values():
        wait_for_hover(drone)

    # 3b. Fly each active drone to its assigned sector
    for station_id, drone in active.items():
        allocation = round_data["allocations"][station_id]  # {sector: time_minutes}
        # Fly to the sector with highest allocation (or iterate)
        sector_id = max(allocation, key=allocation.get)
        sx, sy = SECTOR_CENTERS[sector_id]
        h = heights[drone]
        dur = compute_duration(STATION_POSITIONS[station_id], (sx, sy))
        call_service(drone, "go_to", "crazyflie_interfaces/srv/GoTo",
                     f"{{group_mask: 0, relative: false, goal: {{x: {sx}, y: {sy}, z: {h}}}, "
                     f"yaw: 0.0, duration: {{sec: {dur}, nanosec: 0}}}}")
    for drone in active.values():
        wait_for_hover(drone)

    # 3c. Hover for allocated survey time (scaled for demo)
    max_time = max(
        max(alloc.values()) for alloc in round_data["allocations"].values()
    )
    demo_seconds = max_time * DEMO_TIME_SCALE  # e.g., 1 minute -> 5 seconds
    print(f"  Surveying for {demo_seconds:.0f}s (scaled)")
    time.sleep(demo_seconds)

    # 3d. Return all active drones to their stations
    for station_id, drone in active.items():
        px, py = STATION_POSITIONS[station_id]
        h = heights[drone]
        dur = compute_duration(SECTOR_CENTERS[sector_id], (px, py))
        call_service(drone, "go_to", "crazyflie_interfaces/srv/GoTo",
                     f"{{group_mask: 0, relative: false, goal: {{x: {px}, y: {py}, z: {h}}}, "
                     f"yaw: 0.0, duration: {{sec: {dur}, nanosec: 0}}}}")
    for drone in active.values():
        wait_for_hover(drone)

    # 3e. Handle swaps (if any)
    for station_id in round_data.get("swaps", []):
        old_drone = active[station_id]
        new_drone = standby[station_id]
        h = heights[old_drone]  # same height for the station

        # Land the depleted drone
        call_service(old_drone, "land", "crazyflie_interfaces/srv/Land",
                     "{group_mask: 0, height: 0.0, duration: {sec: 3, nanosec: 0}}")
        wait_for_idle(old_drone)

        # Takeoff the standby
        call_service(new_drone, "takeoff", "crazyflie_interfaces/srv/Takeoff",
                     f"{{group_mask: 0, height: {h}, duration: {{sec: 3, nanosec: 0}}}}")
        wait_for_hover(new_drone)

        # Swap roles
        active[station_id] = new_drone
        standby[station_id] = old_drone
        print(f"  Swapped {old_drone} -> {new_drone} at {station_id}")

# 4. Land all active drones at the end
for drone in active.values():
    call_service(drone, "land", "crazyflie_interfaces/srv/Land",
                 "{group_mask: 0, height: 0.0, duration: {sec: 3, nanosec: 0}}")
print("Demo complete.")
```

---

## Swap Procedure (Detailed)

When the algorithm determines a drone's SoC < e_safe, a swap occurs:

1. Active drone returns to its station (go_to station position)
2. Wait for HOVERING (arrived at station)
3. Active drone lands (land service, height=0.0)
4. Wait for IDLE (landed)
5. Standby drone takes off to the same flight height
6. Wait for HOVERING (airborne)
7. Update active/standby tracking

**Important:** Do NOT takeoff the standby while the active is still landing at the same station. Wait for IDLE first to avoid collision.

---

## Constraints

- **Geofence bounds:** All targets clamped to [-1.9, -1.9, 0.0] to [1.9, 1.9, 2.0]
- **Max horizontal velocity:** 0.3 m/s (configurable via `ros2 param set`)
- **Max vertical velocity:** 0.5 m/s
- **Minimum go_to duration:** Use the formula `distance / 0.3 + 2.0` seconds
- **No diagonal path collision avoidance:** The control layer does NOT have collision avoidance. Use staggered heights to prevent mid-air collisions.
- **Wall-clock vs sim-time:** External `time.sleep()` uses wall-clock time. If Gazebo sim factor < 1.0, trajectories take longer in wall time. For a simple orchestrator using subprocess calls, wall-clock sleep is fine since it adds buffer naturally.

---

## Expected Algorithm Output Format

The orchestrator expects a JSON file with the following structure:

```json
{
  "params": {
    "N": 4,
    "J": 4,
    "K": 50,
    "e_safe": 20.0,
    "demo_time_scale": 0.1
  },
  "rounds": [
    {
      "round": 1,
      "allocations": {
        "station_1": {"sector_1": 5.2, "sector_2": 1.0, "sector_3": 0.5, "sector_4": 0.3},
        "station_2": {"sector_1": 0.8, "sector_2": 4.1, "sector_3": 0.6, "sector_4": 1.5},
        "station_3": {"sector_1": 0.3, "sector_2": 0.7, "sector_3": 4.8, "sector_4": 1.2},
        "station_4": {"sector_1": 0.6, "sector_2": 1.2, "sector_3": 0.4, "sector_4": 4.8}
      },
      "soc": {
        "cf1": 85.0, "cf2": 90.0, "cf3": 78.0, "cf4": 82.0,
        "cf5": 100.0, "cf6": 100.0, "cf7": 100.0, "cf8": 100.0
      },
      "swaps": []
    },
    {
      "round": 15,
      "allocations": { "..." : "..." },
      "soc": { "..." : "..." },
      "swaps": ["station_3"]
    }
  ]
}
```

**Field definitions:**

| Field | Description |
|-------|-------------|
| `params.N` | Number of active drones (4) |
| `params.J` | Number of sectors (4) |
| `params.K` | Total algorithm rounds |
| `params.e_safe` | SoC threshold for swap (%) |
| `params.demo_time_scale` | Multiplier to convert algorithm minutes to demo seconds |
| `rounds[].round` | Round number (only include rounds to demo, not all K) |
| `rounds[].allocations` | Per-station dict of sector_id -> time in minutes allocated |
| `rounds[].soc` | State of charge (%) for each drone after this round |
| `rounds[].swaps` | List of station_ids where a swap occurs after this round |

**Simplification for demo:** The orchestrator only needs to demo a few key rounds (e.g., round 1, a mid-round, and the round where a swap happens). Include only those rounds in the JSON.

---

## Constants for the Orchestrator

```python
SECTOR_CENTERS = {
    "sector_1": (-0.5, -0.5),
    "sector_2": ( 0.5, -0.5),
    "sector_3": (-0.5,  0.5),
    "sector_4": ( 0.5,  0.5),
}

STATION_POSITIONS = {
    "station_1": (-1.5, -1.5),
    "station_2": ( 1.5, -1.5),
    "station_3": (-1.5,  1.5),
    "station_4": ( 1.5,  1.5),
}

# Active drone initial XY (slightly offset from station center)
ACTIVE_START = {
    "station_1": (-1.5, -1.6),
    "station_2": ( 1.5, -1.6),
    "station_3": (-1.5,  1.4),
    "station_4": ( 1.5,  1.4),
}

STANDBY_START = {
    "station_1": (-1.5, -1.4),
    "station_2": ( 1.5, -1.4),
    "station_3": (-1.5,  1.6),
    "station_4": ( 1.5,  1.6),
}

FLIGHT_HEIGHTS = {
    "station_1": 0.5,
    "station_2": 0.7,
    "station_3": 0.9,
    "station_4": 1.1,
}

INITIAL_ACTIVE = {
    "station_1": "cf1", "station_2": "cf2",
    "station_3": "cf3", "station_4": "cf4",
}

INITIAL_STANDBY = {
    "station_1": "cf5", "station_2": "cf6",
    "station_3": "cf7", "station_4": "cf8",
}

MAX_VEL_XY = 0.3  # m/s

def compute_duration(pos_from, pos_to):
    """Compute safe go_to duration for a given distance."""
    dx = pos_to[0] - pos_from[0]
    dy = pos_to[1] - pos_from[1]
    dist = (dx**2 + dy**2) ** 0.5
    return int(dist / MAX_VEL_XY + 3.0)  # integer seconds with buffer
```

---

## Launch Checklist (Before Running Orchestrator)

1. Build: `cd ~/cs2_ws && colcon build && source install/setup.bash`
2. Launch sim: `ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py`
3. Wait 5 seconds for all 8 drones to report odometry
4. Verify: `ros2 topic echo /cf1/status --once` should show `state:IDLE`
5. Run orchestrator: `python3 orchestrator.py fed_dcsa_output.json`
