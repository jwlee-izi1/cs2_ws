# Crazyflie Gazebo Control Layer Guide

A complete reference for the ROS 2 control layer that drives Crazyflie drones in Gazebo simulation. The layer provides two control modes:

- **High-level services** (Crazyswarm2-compatible): takeoff, land, go_to, hover, trajectory execution, emergency stop
- **Low-level teleop passthrough**: direct Twist velocity commands from a joystick or keyboard node

Both modes operate on the same per-drone control node and share the velocity output to Gazebo's MulticopterVelocityControl plugin.

---

## 1. Quick Start

```bash
# Terminal 1: Build and launch
cd ~/cs2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py

# Terminal 2: Fly a drone (high-level services)
source /opt/ros/jazzy/setup.bash && source ~/cs2_ws/install/setup.bash

# Takeoff cf1 to 0.5m over 3 seconds
ros2 service call /cf1/takeoff crazyflie_interfaces/srv/Takeoff \
  "{group_mask: 0, height: 0.5, duration: {sec: 3, nanosec: 0}}"

# Wait ~4s, then fly to (0.0, 0.0, 0.5) over 5 seconds
ros2 service call /cf1/go_to crazyflie_interfaces/srv/GoTo \
  "{group_mask: 0, relative: false, goal: {x: 0.0, y: 0.0, z: 0.5}, yaw: 0.0, duration: {sec: 5, nanosec: 0}}"

# Wait ~6s, then land
ros2 service call /cf1/land crazyflie_interfaces/srv/Land \
  "{group_mask: 0, height: 0.0, duration: {sec: 3, nanosec: 0}}"
```

---

## 2. Architecture Overview

Each enabled drone in `config/crazyflies_sim.yaml` gets:

- **One control node** (`control_services_{name}`) — runs the state machine, P controller, and exposes all services
- **ROS-Gazebo bridge entries** — connects `/{name}/cmd_vel` (ROS) to `/{name}/gazebo/command/twist` (Gazebo), plus odometry, TF, lidar, and clock

The launch file reads the YAML config, spawns drone models into the Gazebo world, and wires everything up automatically. You only need to edit `config/crazyflies_sim.yaml` to add/remove/reposition drones.

### Control modes

| Mode | Active when | Input | Output |
|------|------------|-------|--------|
| **High-level services** | Any service has been called (`service_active = True`) | Service calls (takeoff, land, go_to, etc.) | P controller -> `cmd_vel` |
| **Teleop passthrough** | No service called yet (`service_active = False`) | Twist on `/{name}/cmd_vel_teleop` | Forwarded to `cmd_vel` with height hold |

Once any high-level service is called, the node stays in service mode. The teleop path is disabled until the node is restarted.

---

## 3. Customizing Your Setup

### 3.1 Adding/removing drones

Edit `config/crazyflies_sim.yaml`:

```yaml
robots:
  my_drone_1:
    enabled: true
    type: cf21
    initial_position: [0.0, 0.0, 0.03]    # [x, y, z] in meters
  my_drone_2:
    enabled: true
    type: cf21
    initial_position: [1.0, 0.0, 0.03]
  # disabled drones are ignored by the launch file
  spare:
    enabled: false
    type: cf21
    initial_position: [2.0, 0.0, 0.03]
```

The drone names become the ROS namespace: services at `/my_drone_1/takeoff`, topics at `/my_drone_1/odom`, etc.

### 3.2 Modifying the world

Edit `src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/worlds/crazyflie_world.sdf`. Add static models (walls, obstacles, markers) using standard SDF syntax. The launch file injects `<include>` blocks for the drones, so keep the `<world name="demo">` wrapper intact.

### 3.3 Adjusting the geofence

The default geofence bounds are `[-1.9, -1.9, 0.0]` to `[1.9, 1.9, 2.0]`. If your environment is larger or smaller, change the defaults in `control_services.py` or override per-drone at runtime:

```bash
ros2 param set /control_services_my_drone_1 bounds_min "[-5.0, -5.0, 0.0]"
ros2 param set /control_services_my_drone_1 bounds_max "[5.0, 5.0, 3.0]"
```

### 3.4 Custom launch arguments

```bash
# Use a different config file
ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py \
  crazyflies_yaml_file:=/path/to/my_config.yaml

# Skip launching Gazebo (if already running)
ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py \
  gazebo_launch:=false
```

---

## 4. High-Level Services Reference

All services follow the topic pattern `/{drone_name}/{service_name}` (e.g., `/cf1/takeoff`).

### 4.1 Takeoff

| | |
|---|---|
| **Type** | `crazyflie_interfaces/srv/Takeoff` |
| **Topic** | `/{name}/takeoff` |

**Request fields:**

| Field | Type | Description |
|-------|------|-------------|
| `group_mask` | `uint8` | Unused (Crazyswarm2 compat). Set to 0. |
| `height` | `float32` | Target altitude in meters. If <= 0, defaults to `hover_height` param (0.5m). |
| `duration` | `Duration` | Time to reach target height. If <= 0, defaults to 2.0s. |

**Behavior:** Smoothstep trajectory from current position to (current_x, current_y, height). On completion, transitions to HOVERING state at the target height.

```bash
ros2 service call /cf1/takeoff crazyflie_interfaces/srv/Takeoff \
  "{group_mask: 0, height: 1.0, duration: {sec: 3, nanosec: 0}}"
```

### 4.2 Land

| | |
|---|---|
| **Type** | `crazyflie_interfaces/srv/Land` |
| **Topic** | `/{name}/land` |

**Request fields:**

| Field | Type | Description |
|-------|------|-------------|
| `group_mask` | `uint8` | Unused. Set to 0. |
| `height` | `float32` | Final altitude (typically 0.0 for ground). |
| `duration` | `Duration` | Time to descend. Defaults to 2.0s if <= 0. |

**Behavior:** Smoothstep from current position to landing height. On completion, transitions to **IDLE** (motors stop, zero velocity). The drone will drop to the ground under gravity.

```bash
ros2 service call /cf1/land crazyflie_interfaces/srv/Land \
  "{group_mask: 0, height: 0.0, duration: {sec: 3, nanosec: 0}}"
```

### 4.3 GoTo

| | |
|---|---|
| **Type** | `crazyflie_interfaces/srv/GoTo` |
| **Topic** | `/{name}/go_to` |

**Request fields:**

| Field | Type | Description |
|-------|------|-------------|
| `group_mask` | `uint8` | Unused. Set to 0. |
| `relative` | `bool` | If true, goal is offset from current position. If false, absolute world coordinates. |
| `goal` | `geometry_msgs/Point` | Target position (x, y, z in meters). |
| `yaw` | `float32` | Target heading in **degrees**. If relative, added to current yaw. |
| `duration` | `Duration` | Transit time. Defaults to 3.0s if <= 0. |

**Behavior:** Smoothstep to target. Target is **geofence-clamped** before execution. On completion, transitions to HOVERING. Warns if called while IDLE (takeoff first).

```bash
# Absolute position
ros2 service call /cf1/go_to crazyflie_interfaces/srv/GoTo \
  "{group_mask: 0, relative: false, goal: {x: 0.5, y: -0.5, z: 0.7}, yaw: 0.0, duration: {sec: 4, nanosec: 0}}"

# Relative: move 1m forward in X
ros2 service call /cf1/go_to crazyflie_interfaces/srv/GoTo \
  "{group_mask: 0, relative: true, goal: {x: 1.0, y: 0.0, z: 0.0}, yaw: 0.0, duration: {sec: 3, nanosec: 0}}"
```

### 4.4 Hover

| | |
|---|---|
| **Type** | `std_srvs/srv/Empty` |
| **Topic** | `/{name}/hover` |

**Behavior:** Immediately captures current position as hover setpoint and transitions to HOVERING. Useful to cancel a trajectory and hold position.

```bash
ros2 service call /cf1/hover std_srvs/srv/Empty
```

### 4.5 Emergency

| | |
|---|---|
| **Type** | `std_srvs/srv/Empty` |
| **Topic** | `/{name}/emergency` |

**Behavior:** Immediately sets state to IDLE, sends zero velocity, cuts all control. Also re-enables teleop passthrough mode (`service_active = False`). The drone will fall under gravity.

```bash
ros2 service call /cf1/emergency std_srvs/srv/Empty
```

### 4.6 Upload Trajectory

| | |
|---|---|
| **Type** | `crazyflie_interfaces/srv/UploadTrajectory` |
| **Topic** | `/{name}/upload_trajectory` |

**Request fields:**

| Field | Type | Description |
|-------|------|-------------|
| `trajectory_id` | `uint8` | Unique ID (0-255) to store this trajectory under. |
| `piece_offset` | `uint32` | Insertion index for piecewise append. Usually 0. |
| `pieces` | `TrajectoryPolynomialPiece[]` | Array of polynomial pieces. |

**TrajectoryPolynomialPiece fields:**

| Field | Type | Description |
|-------|------|-------------|
| `poly_x` | `float32[]` | X polynomial coefficients in **ascending power order**: `[a0, a1, a2, ...]` meaning `x(t) = a0 + a1*t + a2*t^2 + ...` |
| `poly_y` | `float32[]` | Y polynomial coefficients (same format). |
| `poly_z` | `float32[]` | Z polynomial coefficients. |
| `poly_yaw` | `float32[]` | Yaw polynomial coefficients. |
| `duration` | `Duration` | Duration of this piece in seconds. |

Does NOT start execution. Call `start_trajectory` after uploading.

### 4.7 Start Trajectory

| | |
|---|---|
| **Type** | `crazyflie_interfaces/srv/StartTrajectory` |
| **Topic** | `/{name}/start_trajectory` |

**Request fields:**

| Field | Type | Description |
|-------|------|-------------|
| `group_mask` | `uint8` | Unused. Set to 0. |
| `trajectory_id` | `uint8` | ID of a previously uploaded trajectory. |
| `timescale` | `float32` | Time scaling factor (1.0 = normal, 2.0 = half speed). Min 0.01. |
| `reversed` | `bool` | If true, traverse trajectory backwards. |
| `relative` | `bool` | If true, offset trajectory to start from current position. |

**Behavior:** Begins polynomial trajectory execution. Transitions to TRAJECTORY state. On completion, transitions to HOVERING at the endpoint.

```bash
ros2 service call /cf1/start_trajectory crazyflie_interfaces/srv/StartTrajectory \
  "{group_mask: 0, trajectory_id: 0, timescale: 1.0, reversed: false, relative: true}"
```

### 4.8 Notify Setpoints Stop

| | |
|---|---|
| **Type** | `crazyflie_interfaces/srv/NotifySetpointsStop` |
| **Topic** | `/{name}/notify_setpoints_stop` |

**Request fields:**

| Field | Type | Description |
|-------|------|-------------|
| `group_mask` | `uint8` | Unused. Set to 0. |
| `remain_valid_millisecs` | `uint32` | Unused in simulation. |

**Behavior:** If currently executing a trajectory, cancels it and transitions to HOVERING at the current position. If already hovering or idle, no-op.

```bash
ros2 service call /cf1/notify_setpoints_stop crazyflie_interfaces/srv/NotifySetpointsStop \
  "{group_mask: 0, remain_valid_millisecs: 0}"
```

---

## 5. Low-Level Teleop Mode

When no high-level service has been called, the control node operates in **teleop passthrough mode**. It subscribes to a Twist topic and forwards it to `cmd_vel` with built-in height hold and yaw rate clamping.

### Input topic

| | |
|---|---|
| **Topic** | `/{name}{incoming_twist_topic}` (default: `/{name}/cmd_vel_teleop`) |
| **Type** | `geometry_msgs/msg/Twist` |

### Behavior

| `linear.z` value | Effect |
|-------------------|--------|
| `> 0` (while not flying) | Auto-takeoff: commands 0.5 m/s upward until `hover_height` reached, then switches to `is_flying = True` |
| `< 0` (while flying) | Auto-land: descends until `z < 0.1m`, then sets `is_flying = False` |
| `~0` (while flying) | Height hold: locks current altitude and applies proportional correction |
| Any (while flying) | `linear.x`, `linear.y`, `angular.z` passed through directly |

### Yaw rate clamping

`angular.z` is clamped to `+/- max_ang_z_rate` (default 0.4 rad/s).

### Switching to service mode

Calling any high-level service (takeoff, go_to, etc.) sets `service_active = True`, which **permanently disables teleop** for that node until:
- The node is restarted, OR
- `emergency` is called (which resets `service_active = False`)

### Example: teleop with keyboard

```bash
# In one terminal, publish teleop commands manually:
ros2 topic pub /cf1/cmd_vel_teleop geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.5}}" --once
# This triggers auto-takeoff. Wait until the drone reaches hover_height.

# Move forward:
ros2 topic pub /cf1/cmd_vel_teleop geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}}" -r 10

# Land (negative z):
ros2 topic pub /cf1/cmd_vel_teleop geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: -0.3}}" -r 10
```

---

## 6. Topics Reference

### 6.1 Odometry (input)

| | |
|---|---|
| **Topic** | `/{name}/odom` |
| **Type** | `nav_msgs/msg/Odometry` |
| **Direction** | Gazebo -> ROS (via bridge) |

Provides current position (`pose.pose.position.x/y/z`) and orientation (quaternion). Published by Gazebo at physics rate. The control node does not produce any output until the first odom message arrives.

```bash
ros2 topic echo /cf1/odom --once
```

### 6.2 Status (output)

| | |
|---|---|
| **Topic** | `/{name}/status` |
| **Type** | `std_msgs/msg/String` |
| **Rate** | 50 Hz (matches control rate) |

**Format:** `"state:<STATE> remaining:<seconds> traj_id:<id>"`

| State | Meaning |
|-------|---------|
| `IDLE` | On ground, no control active |
| `TRAJECTORY` | Executing a movement (takeoff / land / go_to / polynomial) |
| `HOVERING` | Holding position at a setpoint |

**Examples:**
```
state:IDLE remaining:0.000 traj_id:-1
state:TRAJECTORY remaining:1.234 traj_id:5
state:HOVERING remaining:0.000 traj_id:-1
```

**Parsing in Python:**
```python
msg = "state:HOVERING remaining:0.000 traj_id:-1"
parts = dict(p.split(":") for p in msg.split())
state = parts["state"]                # "HOVERING"
remaining = float(parts["remaining"])  # 0.0
traj_id = int(parts["traj_id"])        # -1
```

### 6.3 Velocity Command (internal output)

| | |
|---|---|
| **Topic** | `/{name}/cmd_vel` |
| **Type** | `geometry_msgs/msg/Twist` |
| **Direction** | ROS -> Gazebo (via bridge) |

Published at 50 Hz by the control node. Contains `linear.x/y/z` (m/s) and `angular.z` (rad/s). Consumed by Gazebo's `MulticopterVelocityControl` plugin. You normally don't interact with this topic directly.

### 6.4 Lidar / LaserScan

| | |
|---|---|
| **Topic** | `/{name}/scan` |
| **Type** | `sensor_msgs/msg/LaserScan` |
| **Direction** | Gazebo -> ROS (via bridge) |

If the Crazyflie model includes a lidar sensor, range data is bridged to this topic. Useful for obstacle avoidance algorithms.

---

## 7. Parameters

All parameters are declared on the node `control_services_{name}` (e.g., `control_services_cf1`).

### Launch-time parameters (set in launch file)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `hover_height` | 0.5 | Default takeoff height (m) when service request height <= 0 |
| `robot_prefix` | `/crazyflie` | Topic/service namespace prefix. Set per-drone automatically by the launch file. |
| `incoming_twist_topic` | `/cmd_vel_teleop` | Teleop input topic suffix |
| `control_rate_hz` | 50.0 | Control loop frequency (Hz) |

### Runtime-tunable parameters

These can be changed live with `ros2 param set`. Changes take effect immediately — no restart needed:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kp_xy` | 1.0 | Proportional gain for X/Y position error |
| `kp_z` | 2.0 | Proportional gain for Z position error |
| `kp_yaw` | 1.0 | Proportional gain for yaw error |
| `max_vel_xy` | 0.3 | Max horizontal velocity magnitude (m/s) |
| `max_vel_z` | 0.5 | Max vertical velocity (m/s) |
| `max_ang_z_rate` | 0.4 | Max yaw rate (rad/s) |
| `bounds_min` | [-1.9, -1.9, 0.0] | Geofence lower bound [x, y, z] in meters |
| `bounds_max` | [1.9, 1.9, 2.0] | Geofence upper bound [x, y, z] in meters |

```bash
# List all params for a drone
ros2 param list /control_services_cf1

# Read a value
ros2 param get /control_services_cf1 max_vel_xy

# Change a value (takes effect immediately)
ros2 param set /control_services_cf1 max_vel_xy 0.5

# Change geofence for a larger environment
ros2 param set /control_services_cf1 bounds_min "[-5.0, -5.0, 0.0]"
ros2 param set /control_services_cf1 bounds_max "[5.0, 5.0, 3.0]"
```

---

## 8. Geofence

The geofence clamps all target positions to a bounding box before execution. Any `go_to`, `hover`, or trajectory target outside the bounds is silently clamped to the nearest edge.

**Default bounds** (set for the current 4x4m arena):
- Min: (-1.9, -1.9, 0.0)
- Max: (1.9, 1.9, 2.0)

**Change these** if your environment is different. Either:
1. Edit the defaults in `control_services.py` lines 58-59 (applies to all drones at launch)
2. Override per-drone at runtime via `ros2 param set` (see above)

**When clamping occurs:**
- `go_to` target (before trajectory starts)
- `hover` capture position
- Trajectory endpoint (on completion)
- Position controller target (every control cycle)
- `notify_setpoints_stop` hover capture

**How to know:** The control node logs a warning the first few times clamping occurs. If a drone doesn't reach the requested position, check the node's log output.

---

## 9. Position Controller Internals

The high-level services all feed into a P controller with feedforward velocity. Understanding this helps with tuning.

### Control law

```
# Horizontal (XY)
error_xy = target_pos - current_pos
vel_xy   = kp_xy * error_xy + feedforward_vel_xy
vel_xy   = clamp_magnitude(vel_xy, max_vel_xy)

# Vertical (Z)
error_z = target_z - current_z
vel_z   = clamp(kp_z * error_z + feedforward_vel_z, +/-max_vel_z)

# Yaw
yaw_error = wrap_to_pi(target_yaw - current_yaw)
yaw_rate  = clamp(kp_yaw * yaw_error, +/-max_ang_z_rate)
```

### Feedforward sources

| Trajectory mode | Feedforward |
|----------------|-------------|
| Smoothstep (takeoff, land, go_to) | Derivative of quintic smoothstep: `s'(t) / duration * (end - start)` |
| Polynomial (start_trajectory) | Polynomial derivative evaluated at local time, scaled by `1/timescale` |

### Trajectory interpolation

**Smoothstep** (used by takeoff, land, go_to): `s(t) = 6t^5 - 15t^4 + 10t^3` for `t in [0,1]`. Zero velocity and acceleration at both endpoints — smooth starts and stops.

**Polynomial** (used by start_trajectory): Piecewise polynomials with coefficients in ascending power order. Evaluated via Horner's method.

### Tuning tips

| Symptom | Fix |
|---------|-----|
| Drone overshoots target | Reduce `kp_xy` or `kp_z` |
| Drone approaches target too slowly | Increase `kp_xy` or `max_vel_xy` |
| Drone oscillates at hover | Reduce `kp_xy` (too aggressive for the velocity plugin) |
| Drone loses altitude during fast horizontal moves | Reduce `max_vel_xy` (excessive tilt) |
| Duration too short → drone lags | Use longer duration or increase `max_vel_xy` |

---

## 10. Sample Scripts

### 10.1 Single drone: takeoff, navigate, land

```python
#!/usr/bin/env python3
"""Basic single-drone flight using subprocess calls."""
import subprocess
import time

DRONE = "cf1"

def call_service(drone, service, srv_type, args=""):
    cmd = f'ros2 service call /{drone}/{service} {srv_type} "{args}"'
    subprocess.run(cmd, shell=True, capture_output=True, timeout=15)

def wait_for_hover(drone, timeout=30):
    """Block until drone reports HOVERING state."""
    start = time.time()
    while time.time() - start < timeout:
        result = subprocess.run(
            ["ros2", "topic", "echo", f"/{drone}/status", "--once"],
            capture_output=True, text=True, timeout=10
        )
        if "state:HOVERING" in result.stdout:
            return True
        time.sleep(0.5)
    return False

# Takeoff
call_service(DRONE, "takeoff", "crazyflie_interfaces/srv/Takeoff",
             "{group_mask: 0, height: 0.5, duration: {sec: 3, nanosec: 0}}")
wait_for_hover(DRONE)
print("Hovering!")

# Fly to a waypoint
call_service(DRONE, "go_to", "crazyflie_interfaces/srv/GoTo",
             "{group_mask: 0, relative: false, goal: {x: 1.0, y: 0.0, z: 0.5}, "
             "yaw: 0.0, duration: {sec: 5, nanosec: 0}}")
wait_for_hover(DRONE)
print("At waypoint!")

# Hover for 3 seconds
time.sleep(3)

# Land
call_service(DRONE, "land", "crazyflie_interfaces/srv/Land",
             "{group_mask: 0, height: 0.0, duration: {sec: 3, nanosec: 0}}")
time.sleep(5)
print("Landed.")
```

### 10.2 Multi-drone with staggered heights

```python
#!/usr/bin/env python3
"""Fly multiple drones to different waypoints without collision."""
import subprocess
import time

def call_service(drone, service, srv_type, args=""):
    cmd = f'ros2 service call /{drone}/{service} {srv_type} "{args}"'
    subprocess.run(cmd, shell=True, capture_output=True, timeout=15)

def wait_for_hover(drone, timeout=30):
    start = time.time()
    while time.time() - start < timeout:
        result = subprocess.run(
            ["ros2", "topic", "echo", f"/{drone}/status", "--once"],
            capture_output=True, text=True, timeout=10
        )
        if "state:HOVERING" in result.stdout:
            return True
        time.sleep(0.5)
    return False

# Define your drones and their waypoints
# IMPORTANT: use different heights to avoid mid-air collisions
flights = {
    "cf1": {"height": 0.5, "waypoint": (1.0,  0.0, 0.5)},
    "cf2": {"height": 0.8, "waypoint": (0.0,  1.0, 0.8)},
    "cf3": {"height": 1.1, "waypoint": (-1.0, 0.0, 1.1)},
}

# Takeoff all
for drone, cfg in flights.items():
    call_service(drone, "takeoff", "crazyflie_interfaces/srv/Takeoff",
                 f"{{group_mask: 0, height: {cfg['height']}, duration: {{sec: 3, nanosec: 0}}}}")

for drone in flights:
    wait_for_hover(drone)
print("All hovering.")

# Fly to waypoints
for drone, cfg in flights.items():
    x, y, z = cfg["waypoint"]
    call_service(drone, "go_to", "crazyflie_interfaces/srv/GoTo",
                 f"{{group_mask: 0, relative: false, goal: {{x: {x}, y: {y}, z: {z}}}, "
                 f"yaw: 0.0, duration: {{sec: 6, nanosec: 0}}}}")

for drone in flights:
    wait_for_hover(drone)
print("All at waypoints.")

# Land all
for drone in flights:
    call_service(drone, "land", "crazyflie_interfaces/srv/Land",
                 "{group_mask: 0, height: 0.0, duration: {sec: 3, nanosec: 0}}")
time.sleep(5)
print("Done.")
```

### 10.3 Polynomial trajectory (figure-8 from CSV)

```python
#!/usr/bin/env python3
"""Upload a piecewise polynomial trajectory from CSV and execute it."""
import csv
import subprocess
import time

DRONE = "cf1"
CSV_PATH = "src/crazyswarm2/crazyflie_examples/crazyflie_examples/data/figure8.csv"

def load_csv(path):
    """Load trajectory CSV. Format: duration, x^0..x^7, y^0..y^7, z^0..z^7, yaw^0..yaw^7"""
    pieces = []
    with open(path) as f:
        reader = csv.reader(f)
        next(reader)  # skip header
        for row in reader:
            vals = [float(v) for v in row if v.strip()]
            if len(vals) < 33:
                continue
            pieces.append({
                "poly_x": vals[1:9], "poly_y": vals[9:17],
                "poly_z": vals[17:25], "poly_yaw": vals[25:33],
                "duration": vals[0],
            })
    return pieces

pieces = load_csv(CSV_PATH)

# Takeoff first
subprocess.run(
    f'ros2 service call /{DRONE}/takeoff crazyflie_interfaces/srv/Takeoff '
    '"{group_mask: 0, height: 0.5, duration: {sec: 3, nanosec: 0}}"',
    shell=True, capture_output=True, timeout=15)
time.sleep(5)

# Build and upload
pieces_yaml = "["
for p in pieces:
    sec = int(p["duration"])
    nsec = int((p["duration"] - sec) * 1e9)
    pieces_yaml += (
        f"{{poly_x: {p['poly_x']}, poly_y: {p['poly_y']}, "
        f"poly_z: {p['poly_z']}, poly_yaw: {p['poly_yaw']}, "
        f"duration: {{sec: {sec}, nanosec: {nsec}}}}}, ")
pieces_yaml += "]"

subprocess.run(
    f'ros2 service call /{DRONE}/upload_trajectory '
    f'crazyflie_interfaces/srv/UploadTrajectory '
    f'"{{trajectory_id: 0, piece_offset: 0, pieces: {pieces_yaml}}}"',
    shell=True, capture_output=True, timeout=15)

# Start (relative=true starts from current position)
subprocess.run(
    f'ros2 service call /{DRONE}/start_trajectory '
    f'crazyflie_interfaces/srv/StartTrajectory '
    f'"{{group_mask: 0, trajectory_id: 0, timescale: 1.0, reversed: false, relative: true}}"',
    shell=True, capture_output=True, timeout=15)

total_dur = sum(p["duration"] for p in pieces)
print(f"Running trajectory ({total_dur:.1f}s)...")
time.sleep(total_dur + 2)

# Land
subprocess.run(
    f'ros2 service call /{DRONE}/land crazyflie_interfaces/srv/Land '
    '"{group_mask: 0, height: 0.0, duration: {sec: 3, nanosec: 0}}"',
    shell=True, capture_output=True, timeout=15)
print("Done.")
```

### 10.4 Status monitoring utility

```python
#!/usr/bin/env python3
"""Reusable helpers for monitoring drone state."""
import subprocess
import time

def get_status(drone):
    """Return (state, remaining, traj_id) tuple."""
    result = subprocess.run(
        ["ros2", "topic", "echo", f"/{drone}/status", "--once"],
        capture_output=True, text=True, timeout=10)
    for line in result.stdout.split("\n"):
        if "state:" in line:
            parts = dict(p.split(":") for p in line.strip().strip("'\"").split())
            return (parts.get("state", ""),
                    float(parts.get("remaining", 0)),
                    int(parts.get("traj_id", -1)))
    return ("", 0.0, -1)

def wait_for_state(drone, target_state, timeout=30):
    """Block until drone reaches target state."""
    start = time.time()
    while time.time() - start < timeout:
        state, _, _ = get_status(drone)
        if state == target_state:
            return True
        time.sleep(0.5)
    return False

def get_position(drone):
    """Return dict with x, y, z from odometry."""
    result = subprocess.run(
        ["ros2", "topic", "echo", f"/{drone}/odom", "--once", "--no-arr"],
        capture_output=True, text=True, timeout=8)
    lines = result.stdout.split("\n")
    pos = {}
    for i, line in enumerate(lines):
        if "position:" in line:
            for j in range(1, 4):
                parts = lines[i + j].strip().split(": ")
                if len(parts) == 2:
                    pos[parts[0]] = float(parts[1])
            break
    return pos if "x" in pos else {"x": 0.0, "y": 0.0, "z": 0.0}

# Usage:
# wait_for_state("cf1", "HOVERING")
# pos = get_position("cf1")
# print(f"cf1 at ({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})")
```

---

## 11. Pitfalls and Gotchas

### Odom must arrive before services work
The control node publishes zero velocity until the first odometry message from Gazebo arrives. If you call takeoff immediately after launch, it may not respond for 1-2 seconds. **Fix:** Wait 3-5 seconds after launch, or poll `/{name}/odom` until messages appear.

### Duration too short for distance causes lag
The `max_vel_xy` parameter (default 0.3 m/s) clamps output velocity. If you set a short `go_to` duration for a long distance, the smoothstep trajectory will "finish" but the drone will still be catching up. It will converge eventually (the hover P controller keeps pushing it to the target), but slowly.

**Rule of thumb:** `duration >= distance / max_vel_xy + 2 seconds`

### Multi-drone collisions at same height
If multiple drones cross paths at the same altitude, Gazebo physics will cause them to collide and tumble. After a collision the drone's physics state is corrupted — it typically cannot recover without restarting the sim. **Fix:** Always use staggered flight heights (e.g., 0.5, 0.8, 1.1m).

### go_to while IDLE: takeoff first
Calling `go_to` when the drone is on the ground (IDLE state) logs a warning. The drone may not have enough thrust to translate horizontally on the ground. **Fix:** Always call `takeoff` before `go_to`.

### Landing transitions to IDLE, not HOVERING
After `land` completes, the state becomes IDLE and zero velocity is published. The drone drops under gravity and sits on the ground. If you want to hold a low altitude without landing, use `go_to` to a low height instead.

### Stale ros2 topic pub processes
If you use `ros2 topic pub` for manual testing and hit Ctrl+C, the process may linger and keep publishing commands that override the control node. **Fix:** `pkill -f "ros2 topic pub"` after each manual test.

### Geofence silently clamps targets
If a `go_to` target exceeds the geofence bounds, it is clamped to the nearest edge. The drone flies to the clamped position, not the requested one. **Diagnosis:** Check the control node logs — it warns on the first few clamp events.

### GoTo yaw is in degrees (not radians)
The `yaw` field in GoTo is in **degrees**. The node converts to radians internally. This matches the Crazyswarm2 convention.

### Status topic is a raw string
The `/{name}/status` topic publishes `std_msgs/String`, not a structured message. Parse it by splitting on spaces, then on colons (see section 6.2).

### Polynomial coefficients are ascending power order
`poly_x = [a0, a1, a2, ...]` means `x(t) = a0 + a1*t + a2*t^2 + ...`. This is the **opposite** of numpy's `polyval` convention (descending order). Be careful when converting.

### use_sim_time is enabled
All control nodes use `use_sim_time: True`. ROS timers and durations track Gazebo physics time, but `time.sleep()` in external Python scripts uses wall-clock time. If Gazebo runs slower than real-time (common with 8 drones), trajectories take longer in wall-clock time. For precise synchronization, use `rclpy` timers or poll `/clock`.

### Emergency re-enables teleop
Unlike other services, `emergency` resets `service_active = False`. This means the node returns to teleop passthrough mode. If you want to resume using high-level services after emergency, just call any service (e.g., takeoff) — it will set `service_active = True` again.

---

## 12. State Machine Summary

```
                    takeoff / go_to / start_trajectory
  IDLE ──────────────────────────────────────────────> TRAJECTORY
   ^                                                       |
   |  land completes                    trajectory completes|
   |  emergency                                            v
   +<──────────────────── IDLE <──(land)──── HOVERING <────+
                                               ^     |
                                   hover /     |     | go_to /
                                   notify_stop +─────+ start_trajectory
                                                  via TRAJECTORY
```

| State | Meaning | cmd_vel output |
|-------|---------|---------------|
| IDLE | On ground, not flying | Zero Twist |
| TRAJECTORY | Executing time-based motion | P controller + feedforward |
| HOVERING | Holding position | P controller only (no feedforward) |

---

## 13. File Locations

| File | Path | Purpose |
|------|------|---------|
| Control node | `src/ros_gz_crazyflie/ros_gz_crazyflie_control/ros_gz_crazyflie_control/control_services.py` | State machine, services, P controller |
| Launch file | `src/ros_gz_crazyflie/ros_gz_crazyflie_bringup/launch/crazyflie_simulation.launch.py` | Spawns drones, bridge, control nodes |
| World SDF | `src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/worlds/crazyflie_world.sdf` | Gazebo environment (walls, ground, visuals) |
| Drone config | `config/crazyflies_sim.yaml` | Drone names, positions, enabled/disabled |
| Service defs | `src/crazyswarm2/crazyflie_interfaces/srv/` | Takeoff.srv, Land.srv, GoTo.srv, etc. |
| Message defs | `src/crazyswarm2/crazyflie_interfaces/msg/` | TrajectoryPolynomialPiece.msg |
| Figure-8 CSV | `src/crazyswarm2/crazyflie_examples/crazyflie_examples/data/figure8.csv` | Example polynomial trajectory |

---

## 14. Debugging Cheatsheet

```bash
# Check what drones are running
ros2 node list | grep control_services

# Check all services for a drone
ros2 service list | grep cf1

# Check all topics for a drone
ros2 topic list | grep cf1

# Monitor drone state in real-time
ros2 topic echo /cf1/status

# Read current position
ros2 topic echo /cf1/odom --once

# Watch velocity commands being sent
ros2 topic echo /cf1/cmd_vel

# Check a parameter
ros2 param get /control_services_cf1 bounds_max

# List all parameters
ros2 param list /control_services_cf1

# Kill stale background processes
pkill -f "ros2 topic pub"
pkill -f "ros2 service call"
```
