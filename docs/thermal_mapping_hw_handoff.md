# Thermal Mapping — Hardware Handoff

Written 2026-05-18. Hand-off note from the chat that prepared the thermal
mapping pipeline for hardware flights, to the chat handling the
crazyswarm2/mocap flight stack.

## What this doc is for

You (the flight chat) have a Crazyflie flying via crazyswarm2 + Vicon mocap
(any drone, any controller, any naming — currently cf3 with PID controller
because cf4 turned out unstable with single-marker mocap). You want to
**add thermal mapping on top** so the demo includes:

- Real drone flying autonomously
- A `/thermal_map` topic + RViz heatmap that fills in as the drone moves
  over synthetic hot spots in the lab

This doc explains what was changed in `src/thermal_mapping/` to make the
existing thermal demo (originally designed for the 4-drone CrazySim
fed_dcsa flow) work on hardware with **any subset of drones** — including
a single non-sequential one like cf3.

## What changed (and what didn't)

### Changed: `src/thermal_mapping/launch/thermal_mapping_demo.launch.py`

Added a new optional launch arg `drone_names` that lets you specify
exactly which drones to thermal-map by name. Backward-compatible: if you
don't pass it, behavior is unchanged (cf1..cfN derived from `num_drones`).

| Invocation | Drones thermal-mapped |
|---|---|
| `ros2 launch thermal_mapping thermal_mapping_demo.launch.py` | cf1..cf4 (default, sim) |
| `... num_drones:=2` | cf1, cf2 (sim) |
| `... drone_names:=cf3` | **cf3 only** ← hardware single-drone |
| `... drone_names:=cf1,cf3` | cf1, cf3 (any subset) |

Rebuilt with `colcon build --packages-select thermal_mapping --symlink-install`
so future edits to the launch file don't need another build.

### Not changed: anything else

- `thermal_sensor_node` source — unchanged. Takes `drone_name` as a
  parameter and subscribes to `/<drone_name>/odom`.
- `thermal_mapper_node` source — unchanged. Takes a list `drone_names`
  parameter and subscribes to all `/<name>/thermal/raw` topics.
- `config/thermal_field.yaml` — unchanged. Synthetic Gaussian hot spots in
  the **world (mocap) frame**. Current spots:
  - `(1.0, 1.0)` peak 28°C, radius 0.30 m
  - `(-1.0, -1.0)` peak 18°C, radius 0.50 m
  - `(1.5, -0.5)` peak 12°C, radius 0.20 m
  - Ambient: 22°C
- `config/thermal_mapping_params.yaml` — unchanged. 5×5 m map centered at
  origin, 1 cm cells, 2 Hz publish.

## What the thermal pipeline needs from the flight stack

This is the contract between the two stacks. The flight stack must
provide, and thermal_mapping subscribes to:

| Topic | Type | Frequency | Source |
|---|---|---|---|
| `/<drone_name>/odom` | `nav_msgs/Odometry` | ≥10 Hz | crazyswarm2 firmware logging (`firmware_logging.default_topics.odom`) |

That's it. No other dependency. The thermal pipeline doesn't care whether
the drone is flying via fed_dcsa, manual `go_to` service calls, joystick
teleop, or RL. Just needs `/<drone_name>/odom` to be publishing.

## How to bring up thermal mapping alongside the flight stack

Assumes flight stack (crazyswarm2 + motion_capture_tracking) is already
running and your drone is connected + EKF locked. To add thermal mapping
**in another terminal**:

```bash
source /opt/ros/jazzy/setup.bash
source ~/cs2_ws/install/setup.bash

# For cf3 (current active hardware drone), single drone:
ros2 launch thermal_mapping thermal_mapping_demo.launch.py drone_names:=cf3

# For multiple drones (e.g., cf1+cf3), once more are flying:
ros2 launch thermal_mapping thermal_mapping_demo.launch.py drone_names:=cf1,cf3

# For the eventual 4-drone setup, just use the default:
ros2 launch thermal_mapping thermal_mapping_demo.launch.py num_drones:=4
```

The launch spawns:
- One `thermal_sensor_node` per drone (synthesizes 16×16 thermal sensor
  readings from drone position at 5 Hz)
- One central `thermal_mapper_node` (aggregates → `/thermal_map` at 2 Hz)
- RViz with the GridMap plugin pre-configured to show `/thermal_map`

If the flight stack is using a different RViz config, you can skip the
thermal RViz with the existing default launch — or open a second RViz
window for the thermal view specifically.

## Demo flow (single drone, e.g. cf3)

```bash
# Terminal 1: flight stack (you already know how to do this)
# crazyswarm2 + motion_capture_tracking, drone connected, EKF locked

# Terminal 2: thermal mapping
ros2 launch thermal_mapping thermal_mapping_demo.launch.py drone_names:=cf3

# Terminal 3: actually fly the drone to a hot spot
# Arm + takeoff
ros2 service call /cf3/arm crazyflie_interfaces/srv/Arm "{arm: true}"
ros2 service call /cf3/takeoff crazyflie_interfaces/srv/Takeoff \
  "{group_mask: 0, height: 0.5, duration: {sec: 3, nanosec: 0}}"
sleep 5

# Navigate to a hot spot — (1.5, -0.5) is the closest one and within the
# Vicon volume. Adjust z to comfortable hover height.
ros2 service call /cf3/go_to crazyflie_interfaces/srv/GoTo \
  "{group_mask: 0, relative: false, goal: {x: 1.5, y: -0.5, z: 0.5}, yaw: 0.0, duration: {sec: 5, nanosec: 0}}"
sleep 8

# Hover over the hot spot for thermal readings to accumulate (the map
# updates at 2 Hz, sensor samples at 5 Hz, mapping cell-mean over time)
sleep 5

# Land + disarm
ros2 service call /cf3/land crazyflie_interfaces/srv/Land \
  "{group_mask: 0, height: 0.05, duration: {sec: 3, nanosec: 0}}"
sleep 4
ros2 service call /cf3/arm crazyflie_interfaces/srv/Arm "{arm: false}"
```

What RViz shows:
- Map starts mostly NaN (white/transparent)
- As cf3 hovers over `(1.5, -0.5)`, a hot blob (red/orange in the rainbow
  colormap) fills in at that location
- Areas the drone never visited stay NaN

## Verification (no flight needed)

Before flying, sanity-check the thermal pipeline:

```bash
# Confirm topics are being published while thermal_mapping is up:
ros2 topic hz /cf3/thermal/raw    # should be ~5 Hz once drone has /cf3/odom flowing
ros2 topic hz /thermal_map          # 2 Hz

# Confirm the thermal sensor sees the drone (with drone powered + EKF locked):
ros2 topic echo /cf3/thermal/raw --once   # should show 16x16 data array
ros2 topic echo /thermal_map --once         # GridMap message
```

If `/cf3/thermal/raw` doesn't publish:
- Check `/cf3/odom` is publishing (firmware logging issue, flight stack side)
- Check thermal_sensor's `min_altitude: 0.2` gate — sensor doesn't publish
  if drone is below 0.2 m (avoids spamming the map while drone is on the
  floor). Take off first.

## Known nuances

1. **Hot spot positions are in the WORLD (Vicon) frame.** If the Vicon
   origin is at the cage center (as our setup is), `(1.5, -0.5)` means
   1.5 m in +X (out of cage) and 0.5 m in -Y (operator's left).
   To pick a hot spot to fly toward, look at `config/thermal_field.yaml`
   and match against your Vicon coords.
2. **Map extent is 5×5 m centered at origin.** Drones flying outside that
   won't contribute to the map. Configurable in
   `config/thermal_mapping_params.yaml` (`length_x`, `length_y`, `center_x`,
   `center_y`) if your cage volume is shifted from origin.
3. **`min_altitude: 0.2 m`** gates per-drone publishing. Drone needs to be
   above 20 cm for sensor to emit frames. Pre-takeoff = silent.
4. **Marker placement bias** is invisible to thermal — thermal_sensor uses
   the firmware's `/cfN/odom` which is the EKF state (already biased by
   marker offset). So if the EKF position is slightly off, thermal map
   reads at slightly-wrong positions too. Cosmetic, not functional.

## Where things might trip you up

- **Wrong drone name**: if you pass `drone_names:=cf4` but the flight
  stack only has cf3 enabled, thermal_sensor_cf4 will sit waiting for
  `/cf4/odom` forever. Match drone names exactly to what the flight stack
  publishes.
- **RViz double-instance**: this launch starts its own RViz. If the flight
  stack also has RViz running, you'll get two windows. Either close one
  or skip RViz here by not launching it (you'd need to modify the launch
  or comment out the RViz Node — not done in this handoff).
- **Workspace not built/sourced**: if `ros2 launch thermal_mapping ...`
  returns "package not found", run
  `colcon build --packages-select thermal_mapping --symlink-install` from
  `~/cs2_ws/` and re-source `install/setup.bash`.

## What this handoff does NOT include

- fed_dcsa optimizer (not running for this demo — drone is hand-flown via
  manual `go_to`)
- cf_coverage_planner (same — no automated planner)
- The packet-loss / shared-channel feature (future enhancement, see
  `~/.claude/plans/i-want-to-start-quirky-adleman.md` "Future enhancement"
  section)
- Multi-drone scaling (works automatically once flight stack enables
  more drones — just add them to `drone_names:=cf1,cf2,...`)
