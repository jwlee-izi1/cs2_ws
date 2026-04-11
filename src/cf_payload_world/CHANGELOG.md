# Changelog: Payload-Coupled Hover (2026-04-07)

Everything done in the "payload hover tuning" conversation to get from
"Phase 1 Gazebo world exists but nothing flies" to "stable tilted hover
with z-corrections working."

---

## Files Created (new)

### 1. `src/cf_payload_world/launch/payload_hover.launch.py`
**Purpose**: Combined launch file — starts Gazebo + ros_gz_bridge + 4 control_services nodes in one command. The original `payload_world.launch.py` only started Gazebo.

**What it does**:
- Includes `payload_world.launch.py` with `paused=false` (Gazebo starts running immediately)
- Spawns 4 `control_services` nodes (one per drone, namespaced `/cf1`..`/cf4`)
- Creates a ros_gz_bridge with clock, cmd_vel (ROS→GZ), and odom (GZ→ROS) for each drone
- Bridge config written to `/tmp/cf_payload_bridge.yaml`

**Usage**:
```bash
ros2 launch cf_payload_world payload_hover.launch.py config:=level
ros2 launch cf_payload_world payload_hover.launch.py config:=tilted
```

### 2. `src/cf_payload_world/scripts/test_payload_hover.py`
**Purpose**: Standalone test script — sends go_to commands to all 4 drones, records odom z-traces, produces matplotlib plots.

**What it does**:
- Takes `--targets Z1 Z2 Z3 Z4` for per-drone target heights
- Phase 1: Holds current position for `--hold-time` seconds
- Phase 2: Sends go_to with real targets
- Phase 3: Records odom with wall-clock timestamps, prints live z values
- Saves CSV files and z-trace plot to `--output-dir`

**Usage**:
```bash
python3 src/cf_payload_world/scripts/test_payload_hover.py \
    --targets 1.25 1.25 1.25 1.25 --record-duration 30
```

**Note**: Run this AFTER `payload_hover.launch.py` is up. Not a ROS node — just run with `python3` directly.

### 3. `src/cf_payload_world/TUNING_LOG.md`
**Purpose**: Documents the final working parameters, key findings, and how to reproduce.

---

## Files Modified

### 4. `src/cf_payload_world/worlds/payload_world.sdf` (MAJOR REWRITE)
**What changed**: Complete restructure from flat model to nested models.

**Before**: One flat `<model name="cf4_payload_system">` containing all links (payload, rods, drone bodies, props) and all plugins at the same level. The `MulticopterVelocityControl` plugin summed the entire model mass (~0.17 kg), making hover impossible.

**After**: Each drone (cf1..cf4) is a nested `<model>` inside the parent model:
```
cf4_payload_system (parent)
  ├── payload link (0.050 kg)
  ├── rod_1..rod_4 links + ball joints to payload
  ├── <model name="cf1"> (nested — plugin sees only 0.028 kg)
  │     ├── body link
  │     ├── m1_prop..m4_prop links (relative poses, hardcoded)
  │     ├── m1_joint..m4_joint revolute joints
  │     ├── 4× MulticopterMotorModel plugins
  │     ├── 1× MulticopterVelocityControl plugin
  │     └── 1× OdometryPublisher plugin
  ├── rod1_cf1 ball joint (parent-level, child=cf1::body)
  ├── <model name="cf2"> ... (same structure)
  ├── <model name="cf3"> ...
  └── <model name="cf4"> ...
```

**Specific parameter changes in the SDF**:
- Payload mass: 0.200 → **0.050 kg** (motor budget constraint)
- Payload inertia: scaled proportionally
- maxRotorVelocity: 2618 → **3700 rad/s** (doubles thrust, TWR ~1.7 loaded)
- velocityGain z: 0.2425 → **10.0** (compensates payload mass not in feedforward)
- attitudeGain: **0.02 0.02 0.02** (restored to stock CF values)
- angularRateGain: **0.005 0.005 0.005** (restored to stock CF values)
- Removed `<mass>0.042</mass>` tags (silently ignored by plugin)
- Prop link names: `cf1/m1_prop` → `m1_prop` (scoped inside nested model)
- Joint names: `cf1/m1_joint` → `m1_joint` (scoped inside nested model)
- comLinkName: `cf1_body` → `body`
- Prop poses: absolute `{CF1_M1_POSE}` placeholders → hardcoded relative offsets
- Rod-to-drone joints: `<child>cf1_body</child>` → `<child>cf1::body</child>` (scoped name)

**Placeholders reduced**: 25 → 9 (removed 16 prop pose placeholders)

### 5. `src/cf_payload_world/launch/payload_world.launch.py`
**What changed**:
- Added `paused` launch argument (default `true`; `payload_hover.launch.py` passes `false`)
- Removed propeller absolute pose computation loop (lines 161-166 of original)
- Removed 16 `CF*_M*_POSE` keys from `compute_poses()` — now returns 9 poses
- Tilted config heights: `1.181/1.318` → **`1.21/1.29`** (8cm spread instead of 14cm)

### 6. `src/cf_payload_world/package.xml`
**What changed**: Added exec dependencies:
- `ros_gz_crazyflie_control`
- `ros_gz_bridge`

### 7. `src/ros_gz_crazyflie/ros_gz_crazyflie_control/ros_gz_crazyflie_control/control_services.py`
**What changed** (4 changes):

**(a) Added `kd_z` and `kd_xy` parameters** (lines 54, 56):
```python
self.declare_parameter('kd_xy', 0.0)  # velocity damping, default off
self.declare_parameter('kd_z', 0.0)   # velocity damping, default off
```
Both are runtime-tunable via `ros2 param set`. `kd_xy` exists but is left at 0.0.

**(b) Stored odom twist for velocity damping** (line 88, 724):
```python
self.current_twist = Odometry().twist.twist  # init
self.current_twist = msg.twist.twist         # in _odom_cb
```

**(c) Updated `_position_controller` with PD formula** (lines 647-661):
```python
vx = kp_xy * ex - kd_xy * current_twist.linear.x + ff_vel[0]
vy = kp_xy * ey - kd_xy * current_twist.linear.y + ff_vel[1]
vz = clamp(kp_z * ez - kd_z * current_twist.linear.z + ff_vel[2], max_vel_z)
```
Note the minus sign: `- kd_z * current_vz` damps actual velocity directly.

**(d) Auto-hover on first odom** (in `_odom_cb`):
```python
if not self.odom_received:
    self.odom_received = True
    self.service_active = True
    self.hover_pos = [x, y, z]  # from first odom message
    self.state = self.STATE_HOVERING
```
Prevents freefall between Gazebo start and first go_to command.

**(e) Default `max_vel_z` changed**: 0.5 → **2.0** (allows enough velocity command for payload compensation)

### 8. `src/cf_payload_world/CMakeLists.txt`
**Not changed** — scripts/ directory is not installed. Run test script directly with `python3`.

---

## Files NOT Modified

- `src/ros_gz_crazyflie/ros_gz_crazyflie_bringup/launch/crazyflie_simulation.launch.py` — untouched
- `src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models/crazyflie/model.sdf` — untouched
- `src/fed_dcsa/` — untouched (next step: optimizer integration)
- Bridge topic names — unchanged (robotNamespace stays cf1..cf4)

---

## How to Use

### Build
```bash
cd ~/cs2_ws
colcon build --packages-select cf_payload_world ros_gz_crazyflie_control
source install/setup.bash
```

### Launch
```bash
# Symmetric hover (all drones at z=1.25)
ros2 launch cf_payload_world payload_hover.launch.py config:=level

# Tilted hover (cf1/cf4 at 1.21, cf2/cf3 at 1.29)
ros2 launch cf_payload_world payload_hover.launch.py config:=tilted
```

### Set Tuned Gains (after launch)
```bash
for cf in cf1 cf2 cf3 cf4; do
    ros2 param set /control_services_${cf} kp_z 5.0
    ros2 param set /control_services_${cf} kd_z 0.3
done
```

### Send Z Corrections (optimizer-style)
Always use `relative: true` with `x: 0, y: 0` — only change z:
```bash
# Example: raise cf1 by 3cm over 10 seconds
ros2 service call /cf1/go_to crazyflie_interfaces/srv/GoTo \
    "{group_mask: 0, relative: true, goal: {x: 0.0, y: 0.0, z: 0.03}, yaw: 0.0, duration: {sec: 10, nanosec: 0}}"
```

### Key Rules
- **Never** use `relative: false` with `x: 0, y: 0` — that commands drones to world origin
- **Always** use `relative: true` for z corrections — preserves xy position
- All drones at same z = level payload (regardless of rod angles or xy positions)
- Gains are runtime-tunable: `ros2 param set /control_services_cfN kp_z <value>`
