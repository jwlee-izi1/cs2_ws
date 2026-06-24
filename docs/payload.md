# Payload Coupling

## Goal

4 Crazyflies cooperatively suspend a 200 g rectangular payload via rigid rods
(ball-jointed at the drone end). The setup is a coupled multi-body system: each drone
sees the payload's weight as a constant downward force; tilt of the payload depends on
which drone is too high/low. The demo validates persistent-safety constraint
enforcement against a *physical* coupled-dynamics scenario for the CoRL paper.

Two paths exist in the workspace:

- **Legacy** — `payload_hover.launch.py` uses Gazebo's `MulticopterVelocityControl`
  directly. Kept for comparison.
- **CrazySim** — `payload_hover_crazysim.launch.py` runs the full CF firmware as SITL
  in Docker containers, talking to Gazebo via `gz_crazysim_plugin` + crazyswarm2.
  This is the path used by all other projects in the workspace.

## Packages

| Package | Role |
|---|---|
| `src/cf_payload_world` | World SDF (`payload_world.sdf`, `payload_world_crazysim.sdf`), launch files (`payload_hover.launch.py`, `payload_hover_crazysim.launch.py`), payload coupling geometry. |

**Shared (workspace-level):** uses crazyswarm2 + cf2 docker image from
[CRAZYSIM_MIGRATION.md](../CRAZYSIM_MIGRATION.md).

## Architecture / topic contracts

Standard crazyswarm2 control plane (`/cfN/takeoff`, `/cfN/go_to`, `/cfN/odom`, etc.) —
no project-specific contracts. The payload is a passive rigid body in Gazebo; drones
each hold one ball-jointed rod attached to a corner of the payload.

Physical geometry (from `payload_world_crazysim.sdf`):
- Payload: ~200 g rectangular box at $z = 1.21$ m (when drones at $z = 1.25$ m).
- Rods: 4 ball-jointed rigid links, 0.04 m above each drone's base, to the 4 payload
  corners.
- Effective mass per drone: ~40 g = 28 g CF body + 12.5 g share of 50 g payload tension.

## Status

| Feature | State | Notes |
|---|---|---|
| Plumbing (cf2 ↔ Gazebo ↔ crazyswarm2) | ✅ | All 4 cf2 containers handshake, services register, world launches paused with drones at z=1.25 m. |
| Coordinated takeoff sequence | ✅ | `/all/takeoff` accepted; drones spool motors. |
| **Stable hover under coupled physics** | ⏳ | Drones tumble after unpause. Two compounding failures: (1) firmware integrators not pre-loaded for extra constant downward force from payload — they need a few seconds to wind up, gravity wins during that window; (2) once 1-2 drones lose altitude, ball-jointed rods enter ill-conditioned configurations, DART solver hits 360%+ CPU, real-time factor collapses, controller can't keep up. |
| `go_to` under rod constraint | ❌ | Untested — depends on stable hover first. |
| `fed_dcsa` integration on payload | ❌ | Possible once stable hover lands; service interfaces (`/cfN/takeoff`, `/cfN/go_to`) are 100% identical between this and free-flying setups. |

## Next steps (priority order)

1. **Survive the unpause transient.** Two complementary approaches:
   - **Pre-arm via setpoint.** Before unpause, send each drone
     `/cfN/notify_setpoints_stop` followed by a manual position setpoint at its spawn
     pose (`commander.send_position_setpoint`) for ~2 s. Lets the firmware spin motors
     up to hover thrust against the paused world; once unpaused, gravity is matched
     immediately. The legacy MulticopterVelocityControl path got this for free.
   - **Spawn on the ground.** Edit `CONFIGS['level']` in `payload_world.launch.py` to
     $z = 0.05$ m (drones touching ground), payload resting on ground. Unpause first,
     takeoff after — no transient. Caveat: 4 drones + payload + ground collision +
     ball-joint solver may have its own startup pain.

2. **Tune firmware PID for coupled dynamics.** Knobs in
   `config/crazyflies_payload.yaml` under `all.firmware_params`:
   ```yaml
   posCtlPid:
     xKp: 1.0   # default ~2.0, halve to start
     zKp: 1.5   # default ~2.0
   velCtlPid:
     vxKFF: 0.0; vyKFF: 0.0; vzKFF: 0.0   # try disabling feed-forward first
   ```
   Per-drone overrides under `robots.cfN.firmware_params`. Param names in
   `crazyflie-firmware/src/modules/src/controller_pid.c`.

3. **Physics tweaks** (if solver still struggles after tuning):
   - Reduce IMU update rate 1000 → 250 Hz per drone in `payload_world_crazysim.sdf`.
   - Add joint damping (`<damping>` inside `<axis>` of ball joints).
   - Increase `<max_step_size>` from 0.001 to 0.002 in the physics block.

4. **Validate `go_to` with rod constraint** once stable hover works. Command all drones
   to $\Delta x = +0.3$ m; verify payload follows without rod binding. Try yaw-only and
   tilt-only setpoints.

5. **Re-do the deleted `test_tier{1,2}.py` benchmarks** — old scripts were specific to
   `cf_fw_controller`; write fresh ones that call `/cfN/takeoff`, `/cfN/go_to`, then
   sample `/cfN/odom` for metrics.

## How to run

```bash
# Terminal 1 — full stack: Gazebo + 4 cf2 containers + crazyswarm2 server
ros2 launch cf_payload_world payload_hover_crazysim.launch.py
# Optional: config:=tilted

# Terminal 2 — coordinated takeoff
ros2 service call /all/takeoff crazyflie_interfaces/srv/Takeoff \
    "{group_mask: 0, height: 1.25, duration: {sec: 4, nanosec: 0}}"
```

To still run the legacy MulticopterVelocityControl path for comparison:
```bash
ros2 launch cf_payload_world payload_hover.launch.py
```

## Verification block (run on next session, fill in result)

```text
[ ] Stack starts: `ros2 launch cf_payload_world payload_hover_crazysim.launch.py`
    → all 4 containers Up, /cf1..cf4/takeoff services available within 90 s.
[ ] Coordinated takeoff: `/all/takeoff` to 1.25 m → all 4 drones reach 1.25 m ± ?
[ ] 30-second hover: max(z) - min(z) over 30 s = ? cm  (target < 10 cm)
[ ] Payload z stable: payload z = 1.21 m ± ? cm  (target < 5 cm)
[ ] Per-drone go_to: cf1 to (Δx=+0.1, Δy=0, z=1.25) → followed by others, no rod NaN.
```

## Historical context

See [CRAZYSIM_MIGRATION.md §5](../CRAZYSIM_MIGRATION.md) for the original migration
history — including why `cf_fw_controller` (the direct-cffirmware approach) was
abandoned in favour of full SITL firmware in Docker, and the exact known-issues list
for the CrazySim setup (socket ACL non-persistence, daemon cache, plugin socketInit
latching, etc.).
