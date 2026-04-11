# Payload-Coupled Hover Tuning Log

## Date: 2026-04-07

## Final Working Configuration

### SDF Parameters (`payload_world.sdf`)
- **Model structure**: Nested `<model>` per drone (cf1..cf4) inside parent model
  - Fixes MulticopterVelocityControl mass computation (sees ~0.028 kg per drone, not full model)
- **Payload mass**: 0.050 kg (payload/drone ratio ~1.8x)
- **Rod mass**: 0.002 kg each
- **maxRotorVelocity**: 3700 rad/s (2x stock CF, gives TWR ~1.7 loaded)
- **velocityGain**: `1.25 1.25 10.0` (z=10.0 compensates for payload mass not in feedforward)
- **attitudeGain**: `0.02 0.02 0.02` (stock CF values)
- **angularRateGain**: `0.005 0.005 0.005` (stock CF values)

### ROS Controller Parameters (`control_services.py`)
- **kp_z**: 5.0 (set at runtime via `ros2 param set`)
- **kd_z**: 0.3 (velocity damping, set at runtime)
- **kp_xy**: 1.0 (default, untouched)
- **kd_xy**: 0.0 (available but not needed)
- **max_vel_z**: 2.0 (increased from 0.5 to allow payload compensation)
- **max_vel_xy**: 0.3 (default)
- **Auto-hover on startup**: Enabled (drones hold spawn position on first odom)

### Launch Configuration
- **payload_hover.launch.py**: Starts Gazebo with `-r` (running, not paused) + bridge + 4 control nodes
- **Level config**: All drones at z=1.25
- **Tilted config**: cf1/cf4 at z=1.21, cf2/cf3 at z=1.29 (8cm spread)

## Key Findings

### Problem 1: MulticopterVelocityControl mass computation
- In a flat (non-nested) SDF model, the plugin sums ALL link masses (~0.17 kg)
- Each drone's gravity feedforward was 6x too high, motors saturated
- The `<mass>` XML tag is **silently ignored** — not a supported parameter
- **Fix**: Nested `<model>` elements — plugin only sees its own drone's ~0.028 kg

### Problem 2: Startup freefall
- Drones fall before control commands arrive (bridge startup delay)
- **Fix**: Auto-hover at spawn position on first odom message + launch with `-r`

### Problem 3: Steady-state altitude droop
- Plugin feedforward only compensates drone mass, not payload share
- Deficit: ~0.137 N per drone must come from velocity controller
- P-only controller produces droop = v_needed / kp_z
- **Fix**: High velocityGain_z (10.0) + high kp_z (5.0) reduces droop to ~1cm
- kd_z = 0.3 damps oscillation during transitions

### Problem 4: xy drift from go_to commands
- Absolute go_to with x=0, y=0 commanded all drones to world origin — crashed
- **Fix**: Use `relative: true` with `goal: {x: 0, y: 0, z: delta}` for z-only corrections

## Verified Behaviors

1. **Symmetric hover** (level config): All 4 drones hold z=1.243 ± 0.2mm for 30+ seconds
2. **Asymmetric hover** (tilted config): Drones hold different z heights stably
3. **Coordinated z corrections**: Relative z offsets drive tilted payload toward level
4. **Leveling verified**: After corrections, all drones within <1cm of each other, payload visually level

## How to Run

```bash
# Build
colcon build --packages-select cf_payload_world ros_gz_crazyflie_control

# Launch (level or tilted)
ros2 launch cf_payload_world payload_hover.launch.py config:=level
ros2 launch cf_payload_world payload_hover.launch.py config:=tilted

# Set tuned gains (after launch)
for cf in cf1 cf2 cf3 cf4; do
    ros2 param set /control_services_${cf} kp_z 5.0
    ros2 param set /control_services_${cf} kd_z 0.3
done

# Send z-only corrections (example: cf1 up 3cm)
ros2 service call /cf1/go_to crazyflie_interfaces/srv/GoTo \
    "{group_mask: 0, relative: true, goal: {x: 0.0, y: 0.0, z: 0.03}, yaw: 0.0, duration: {sec: 10, nanosec: 0}}"
```

## Files Modified

| File | Changes |
|------|---------|
| `src/cf_payload_world/worlds/payload_world.sdf` | Nested models, 50g payload, 3700 RPM motors, velocityGain_z=10 |
| `src/cf_payload_world/launch/payload_world.launch.py` | Removed prop pose placeholders, added `paused` arg, tilted config 8cm |
| `src/cf_payload_world/launch/payload_hover.launch.py` | Combined launch (Gazebo -r + bridge + control), paused=false |
| `src/ros_gz_crazyflie/ros_gz_crazyflie_control/ros_gz_crazyflie_control/control_services.py` | Added kd_z/kd_xy params, auto-hover on startup, max_vel_z=2.0 |
| `src/cf_payload_world/scripts/test_payload_hover.py` | Test script with odom recording and plotting |
