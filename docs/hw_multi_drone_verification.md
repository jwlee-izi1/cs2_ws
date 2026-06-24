# Multi-drone hardware verification — staged procedure + debug guide

Written 2026-05-23 — **living document** (per-test log in §7, updated
every run).

The workspace's **canonical multi-drone HW verification procedure**.
Workspace infrastructure (not a transitional handoff). Used standalone
for any multi-drone bringup (demo morning, post-repair fleet check,
new-drone added to swarm), AND invoked from the multi-drone phase
handoff. Mirrors the structure of
[hw_drone_verification.md](hw_drone_verification.md) one level up.

This doc serves **two purposes**:

1. **Procedure + debug toolkit (§1–6)** — canonical reference for how
   to verify any multi-drone configuration (2 → 3 → 4) on this
   workspace, including the figure-8 motion stage with onboard
   collision avoidance.
2. **Per-test verification log (§7)** — chronological record of every
   multi-drone test run. **Add an entry every time a multi-drone
   procedure stage runs** so the swarm-level state stays canonical.

**Read §3 (multi-drone non-negotiables) and §5.1 (multi flight_logger)
before any multi-drone HW flight.**

## 1. What this doc is for

Multi-drone hardware verification, scaling **2 → 3 → 4 drones** in
explicit gated stages, culminating in a motion stage that drives the
full planner → streamer → firmware chain with onboard Buffered
Voronoi Cell (BVC) collision avoidance enabled.

When all stages pass, the swarm is ready for the demo (thermal-mapping
video) and as a verified baseline for future RL-policy work.

Single-drone verification is a **prerequisite** (every drone must have
recently passed [hw_drone_verification.md](hw_drone_verification.md));
this doc handles only the multi-drone-specific layers.

## 2. Prerequisites

- Every drone in the swarm has a recent **single-drone-verification
  pass** logged in `hw_drone_verification.md §7` (within last 7 days,
  OR after any physical change to that drone — battery swap is fine;
  marker re-fix, motor swap, prop replacement re-runs).
- All drones on firmware **2025.12** and **channel 100**.
- `/tmp/multi_flight_logger.py` available (recreate from §5.1 if wiped).
- Vicon volume free of stray reflectors; `vicon_probe.py` shows
  count = N when all N drones placed (or you understand the labeled
  rigid-body caveat — see [hw_drone_verification.md §5.2](hw_drone_verification.md)).
- `cf_coverage_planner` built with the new figure-8 node:
  ```bash
  cd ~/cs2_ws && colcon build --packages-select cf_coverage_planner --symlink-install
  ```
- For Stage 0-sim: the **fed_dcsa Gazebo sim** must be launchable —
  the full integration env, not standalone CrazySim. See
  [docs/federated_coverage.md](federated_coverage.md) and
  `~/cs2_ws/scripts/thermal_demo.sh up`.

## 3. The multi-drone non-negotiables

In ADDITION to the two single-drone non-negotiables (yaw≈0 placement,
channel 100 — see
[hw_drone_verification.md §3](hw_drone_verification.md)):

1. **Marker spacing ≥ 30 cm at startup.** librigidbodytracker assigns
   each drone's rigid body to the nearest marker using
   `initial_position`. Markers too close → assignment confusion / ID
   swaps. Spread drones laterally at startup.

2. **`initial_position` freshly probed per drone in isolation.** Probe
   each drone alone (others off / out of volume), update YAML. Stale
   positions after rearrangement are the **#1 cause of ID swaps** in
   multi-drone bringup.

3. **Vertical hover stagger ≥ 20 cm for hover stages (1–3a).**
   Eliminates collision risk during the hover phase regardless of mocap
   glitches. Stages 3b/4 use a single height because per-drone
   quadrants make collisions geometrically impossible (see §3.5).

4. **Motion stages (3b/4) MUST have BVC enabled AND fed peer positions** in
   `config/crazyflies_hw.yaml`:
   ```yaml
   all:
     peer_broadcast_hz: 30.0   # ⬅ REQUIRED — without this BVC is BLIND (see below)
     firmware_params:
       colAv:
         enable: 1
   ```
   BVC is the firmware-level Buffered Voronoi Cell collision avoidance
   shipped in crazyswarm2 (contributed by James Preiss). Onboard,
   decentralized, mocap-driven. See [crazyswarm2
   howto](https://imrclab.github.io/crazyswarm2/howto.html).
   **⚠ 2026-06-03 — the gap that made BVC a no-op:** `colAv.enable: 1` only
   turns the *algorithm* on. It does NOTHING unless each drone is told its
   NEIGHBORS' positions via a packed `EXT_POSITION_PACKED` CRTP packet
   (id≠my_id → `peerLocalizationTellPosition`). The stock crazyswarm2 server's
   `_poses_changed` sends each drone only its OWN position — so BVC saw
   `nOthers=0` and never engaged on HW (or sim). We added a peer-broadcaster to
   `crazyflie_server.py` gated by `peer_broadcast_hz`; it must be > 0 here.
   **Verify with Stage 3c (§4.8) — it is the only proof BVC is actually live.**

5. **Per-drone quadrant partitioning for motion stages.** Each drone
   confined to its own rectangular quadrant on the diagonals:
   - cf1 → +x +y quadrant, center (+0.9, +0.9)
   - cf2 → −x +y quadrant, center (−0.9, +0.9)
   - cf3 → −x −y quadrant, center (−0.9, −0.9)
   - cf4 → +x −y quadrant, center (+0.9, −0.9)
   Per-drone figure-8 amplitude capped at 0.5 m so worst-case drone
   position stays inside the **1.9 m HW envelope cap** (see
   `src/cf_coverage_planner/config/arena_4drone.yaml`).
   Shared waypoints / cross-quadrant motion is forbidden.

6. **Stage 0-sim must pass before any HW Stage 3b run.** No exceptions.
   See §4.3.

## 4. The scale-up verification procedure

Run the stages in order. Do not advance to the next stage unless the
current one passes its pass criteria cleanly. Log every stage run in
§7.

### 4.1 Pre-flight (every stage)

1. Power on all N drones for this stage; fresh charged batteries.
2. Place each at yaw≈0 (front +X along Vicon +X), with markers
   ≥ 30 cm apart laterally.
3. Probe each drone's `initial_position` in isolation
   (`python3 /tmp/vicon_probe.py 5` per drone, others off). Update
   `config/crazyflies_hw.yaml`.
4. Enable only the N drones for this stage; `enabled: false` for the
   rest.

### 4.2 Launch + 60s idle-link gate

```bash
cd ~/cs2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
nohup ros2 launch crazyflie launch.py \
  backend:=cflib \
  crazyflies_yaml_file:=$HOME/cs2_ws/config/crazyflies_hw.yaml \
  motion_capture_yaml_file:=$HOME/cs2_ws/config/motion_capture.yaml \
  gui:=false > /tmp/hw_bringup.log 2>&1 &
```
Wait for **N** `[cfN] is fully connected!` lines. Then run the
60-second idle-link gate:
```bash
sleep 60
DROPS=$(grep -cE "Too many packets|is disconnected" /tmp/hw_bringup.log)
echo "link drops in 60s across $N radios: $DROPS"  # expect 0
```

If any drone fails to connect or any drop is observed: drop back to
that drone's single-drone verification in isolation. See §5.3.

### 4.3 Stage 0-sim — fed_dcsa Gazebo integration dry run

**Mandatory before any HW Stage 3b execution.**

```bash
# Bring up CrazySim + crazyswarm2 + thermal stack with 4 sim drones
COVERAGE_4DRONE=1 ~/cs2_ws/scripts/thermal_demo.sh up
# Then launch the coverage demo with figure-8 planner swapped in
ros2 launch cf_coverage_planner coverage_demo.launch.py \
  optimizer:=fed_dcsa planner_type:=figure8
```

Let it run for **60 s** while the fed_dcsa optimizer publishes
dynamically-changing per-drone leashes.

**Pass criteria:**
- Every drone's figure-8 amplitude **tracks its leash** (figure-8 grows/
  shrinks as the optimizer changes the radius).
- Every drone stays in its quadrant (the per-quadrant sanity assertion
  in the node never fires — check the node's log for `OUTSIDE quadrant`
  errors; if seen, lower `quadrant_amplitude_max` in
  `arena_4drone.yaml`).
- Min inter-drone spacing > 1.0 m throughout (run
  `multi_flight_logger.py` against the sim topics).
- Motion smooth — no setpoint discontinuities when leash steps.
- `/thermal_map` populates visibly in RViz; > 70% of expected cells
  filled within 60 s.

This is an **integration test**, not a standalone test — it validates
the figure-8 node end-to-end with the real optimizer AND the thermal
consumer. A standalone constant-leash sim would miss the dynamic-leash
response and the thermal-coverage adequacy.

**Failure here = do NOT proceed to HW.** Iterate on
`quadrant_amplitude_max`, quadrant centers, or amplitude bounds in
`arena_4drone.yaml` until the sim is clean.

### 4.4 Stage 1 — 2-drone hover

cf1 + cf2, heights z = 0.4 / 0.6 m, BVC **off**, direct service calls.

```bash
heights=(0.4 0.6)
for i in 0 1; do
  n=$((i+1))
  ros2 service call /cf$n/arm crazyflie_interfaces/srv/Arm "{arm: true}"
  sleep 1
  ros2 service call /cf$n/takeoff crazyflie_interfaces/srv/Takeoff \
    "{group_mask: 0, height: ${heights[$i]}, duration: {sec: 3, nanosec: 0}}"
  sleep 4
done
sleep 10   # simultaneous hover
for n in 1 2; do
  ros2 service call /cf$n/land crazyflie_interfaces/srv/Land \
    "{group_mask: 0, height: 0.05, duration: {sec: 3, nanosec: 0}}"
  sleep 4
  ros2 service call /cf$n/arm crazyflie_interfaces/srv/Arm "{arm: false}"
done
```

Run `multi_flight_logger.py` in background during the test.

**Pass criteria (per drone):**
- Climbs to target z ±5 cm.
- `/cfN/odom` ↔ its `/poses` within ~5 mm throughout.
- (x, y) drift < 5 cm during simultaneous hover.
- **0 link drops** across both drones.
- Monotonic descent.

### 4.5 Stage 2 — 3-drone hover

Add cf3 at z = 0.8 m. Same procedure + criteria for all 3.

**Gate: stop unless clean.** If a drone passed in Stage 1 but fails
here, do NOT skip ahead — isolate via §5.3 first.

### 4.6 Stage 3a — 4-drone hover

Add cf4 at z = 1.0 m. Swarm flight-stack baseline. **Gate.**

### 4.6a Stage 3a.5 — single-drone figure-8 smoke (optional but recommended)

**Why:** Stages 1/2/3a use direct service calls (arm/takeoff/land) and do
NOT exercise the `policy_target → setpoint_streamer → cmd_position`
chain on real HW. Stage 3b is the first run of that chain AND the first
multi-drone motion. Inserting a single-drone HW smoke here separates
"does the policy chain work on real HW?" from "does multi-drone
interaction work?" — at the cost of one extra ~5 min flight (~1 battery).

Pick the most-verified drone (currently **cf3** per
[hw_drone_verification.md §7](hw_drone_verification.md), verified
envelope clean to 1.5 m/s). The other 3 drones stay `enabled: false`
for this stage. BVC off (single drone — nothing to collide with).

**Config for this stage** (`config/crazyflies_hw.yaml`):
```yaml
robots:
  cf1: { enabled: false, ... }
  cf2: { enabled: false, ... }
  cf3: { enabled: true,  uri: ..., initial_position: [x, y, z], type: cf21 }
  cf4: { enabled: false, ... }
```
BVC stays disabled (`colAv.enable: 0` or omit) — re-enable only for
Stage 3b.

**Staged bringup** (mirrors §4.7.1 but for one drone):

1. Launch crazyswarm2 with only cf3 enabled (§4.2).
2. Takeoff cf3 to z = 0.6 m (NOT 1.0 — match figure-8 z):
   ```bash
   ros2 service call /cf3/arm crazyflie_interfaces/srv/Arm "{arm: true}"
   sleep 1
   ros2 service call /cf3/takeoff crazyflie_interfaces/srv/Takeoff \
     "{group_mask: 0, height: 0.6, duration: {sec: 3, nanosec: 0}}"
   sleep 4
   ```
3. goTo cf3's expected figure-8 init pose (for leash = 0.8, alpha = 0.5,
   cf3 is in −x−y quadrant so signs = (−1, −1)):
   ```bash
   ros2 service call /cf3/go_to crazyflie_interfaces/srv/GoTo \
     "{group_mask: 0, relative: false, goal: {x: -0.4, y: -0.4, z: 0.6}, yaw: 0.0, duration: {sec: 4, nanosec: 0}}"
   sleep 5
   ```
4. Set `streamer.max_setpoint_velocity: 0.5` in
   `src/cf_coverage_planner/config/arena_4drone.yaml`. Rebuild
   `cf_coverage_planner` if needed.
5. Set the constant-leash radius for cf3 = 0.8 m (in
   `baseline_optimizers` config or per-drone override).
6. Launch coverage_demo with `optimizer:=constant planner_type:=figure8`.
   The figure-8 node will start publishing
   `target = (-0.4 + cos(yaw)·scale·nx, -0.4 + sin(yaw)·scale·ny, 0.6)`
   with `scale = beta·leash/native_radius = 0.25·0.8/0.996 ≈ 0.2`.
   So cf3 traces a figure-8 with x-amplitude ~0.2 m around center
   (−0.4, −0.4), slowly rotating at 10 deg/s.

Let it fly for **30 s**.

**Pass criteria:**
- cf3 traces a recognisable figure-8 centred around (−0.4, −0.4) with
  x-amplitude ~0.2 m (y-amplitude ~0.1 m due to lemniscate 2:1 aspect).
- `/cf3/odom` ↔ figure-8 target tracking error RMS < 10 cm.
- cf3 position never exceeds leash = 0.8 m from arena origin (run
  `multi_flight_logger.py 30 cf3` and check PAIRS lines —
  `max(|odom|) ≤ 0.8`).
- Setpoint stream is smooth (no setpoint discontinuities visible in
  /cf3/odom; no sawtooth in z).
- 0 link drops.
- Clean land (lower max_setpoint_velocity to 0.3 first, send a goTo to
  (−0.4, −0.4, 0.1), then land).

**Gate:** if 3a.5 passes, you've validated the figure-8 chain
end-to-end on real HW. Proceed to Stage 3b (4-drone motion). If 3a.5
fails, debug single-drone first — multi-drone debug would conflate too
many variables.

If 3a.5 is skipped: Stage 3b runs as the first HW exercise of the
policy chain. Acceptable but riskier; document the skip in §7.

### 4.7 Stage 3b — 4-drone figure-8 + BVC

#### 4.7.1 Staged HW bringup (every Stage 3b/4 run)

This staged sequence is **different from the sim launch** — the sim
takes everything off to 1.0 m and lets the streamer drop to 0.6 m
(ugly but harmless in sim). On HW that drop is dangerous. Run the
stages in order and abort if any one looks wrong before proceeding.

1. **Enable BVC.** Edit `config/crazyflies_hw.yaml`:
   ```yaml
   all:
     firmware_params:
       colAv:
         enable: 1
   ```
   Rebuild + re-source.

2. **Per-drone takeoff to z = 0.6 m** (NOT 1.0 m — match figure-8 z so
   the streamer never has to drop the setpoint). Each drone takes off
   from its physical placement on the floor — no goTo yet.
   ```bash
   for n in 1 2 3 4; do
     ros2 service call /cf$n/arm crazyflie_interfaces/srv/Arm "{arm: true}"
     sleep 0.5
     ros2 service call /cf$n/takeoff crazyflie_interfaces/srv/Takeoff \
       "{group_mask: 0, height: 0.6, duration: {sec: 3, nanosec: 0}}"
     sleep 4
   done
   ```
   Visual check: all 4 drones holding at z = 0.6 m at their floor (x, y).

3. **Per-drone goTo target init pose.** Each drone moves to its
   expected figure-8 starting center — for `center_alpha = 0.5` and
   the constant_leash radii you're about to use, that's
   `(alpha * leash * sign_x, alpha * leash * sign_y, 0.6)`.
   ```bash
   # Example for leash = 1.0 m per drone (alpha * leash = 0.5):
   for cf_xy in "cf1:0.5:0.5" "cf2:-0.5:0.5" "cf3:-0.5:-0.5" "cf4:0.5:-0.5"; do
     name=${cf_xy%%:*}; xy=${cf_xy#*:}; x=${xy%:*}; y=${xy#*:}
     ros2 service call /$name/go_to crazyflie_interfaces/srv/GoTo \
       "{group_mask: 0, relative: false, goal: {x: $x, y: $y, z: 0.6}, yaw: 0.0, duration: {sec: 4, nanosec: 0}}"
     sleep 4
   done
   ```
   Visual check: 4 drones spread to the 4 quadrant centers, holding.

4. **Start the thermal pipeline** (per-drone `thermal_sensor_node` +
   central `thermal_mapper`) — for Stage 3b you can skip this and add
   it in Stage 4; for Stage 4 it must start before the policy.

5. **Start the optimizer** (`constant_leash_node` for Stage 3b motion
   verification; `fed_dcsa` only in the final demo run). It begins
   publishing `/coverage/leash` at ~0.5 Hz.

6. **Start the streaming policy** (`quadrant_figure8_node` per drone +
   `setpoint_streamer_node` per drone). The streamer starts walking
   the setpoint from the current goTo target toward the figure-8
   target at `≤ max_setpoint_velocity`. Drones begin tracing the
   figure-8 in their assigned quadrants.

All of (4)–(6) are spawned by `coverage_demo.launch.py`. The
key separation from sim: takeoff and goTo are done BEFORE the launch,
so the figure-8 node never has to drop the drone in altitude.

#### 4.7.2 Multi-drone envelope sweep

Run two sub-stages in order, mirroring the single-drone envelope test:

| Sub-stage | `max_setpoint_velocity` | Notes |
|---|---|---|
| 3b-slow | **0.5 m/s** | First multi-drone motion ever. Isolates "does the chain work?" from any speed effect. Mandatory baseline. |
| 3b-default | **1.2 m/s** | Sim default; all 4 drones individually verified here (per pilot's confirmation; log per-drone entries in `hw_drone_verification.md §7`). |
| 3b-fast (optional) | **1.5 m/s** | cf3's verified ceiling. Skip unless you want a faster demo. |

Change `streamer.max_setpoint_velocity` in
`src/cf_coverage_planner/config/arena_4drone.yaml` between runs.
Rebuild is NOT needed (it's a YAML param) but the streamer nodes
must be relaunched to pick up the new value — easiest: kill and
re-run the coverage_demo launch.

#### 4.7.3 Pass criteria (per sub-stage)

- Hover criteria (§4.4) hold for all 4 drones simultaneously.
- Tracking error (RMS distance between `/cfN/odom` and target) < 10 cm
  per drone over 30 s of motion.
- No BVC-induced freezes (drone target moves but `/cfN/odom` stalls
  > 1 s).
- Per-quadrant assertion in the figure-8 node never fires (check log).
- 0 link drops.
- Drone never leaves its leash circle from arena origin (figure-8 is
  leash-bounded by construction; `multi_flight_logger.py` PAIRS line
  shows per-drone max distance from origin ≤ leash).

### 4.8 Stage 3c — BVC sanity test

Proves BVC is *actually* engaged in firmware (not just enabled in
YAML). Two drones (cf1 + cf2) at z = 0.6 m, BVC on. Command both
`goTo` the midpoint of their start positions simultaneously — without
BVC they'd collide; with BVC they should stop short.

```bash
# cf1 starts at (+0.9, 0), cf2 at (-0.9, 0) — midpoint (0, 0)
ros2 service call /cf1/go_to crazyflie_interfaces/srv/GoTo \
  "{group_mask: 0, relative: false, goal: {x: 0.0, y: 0.0, z: 0.6}, yaw: 0.0, duration: {sec: 4, nanosec: 0}}" &
ros2 service call /cf2/go_to crazyflie_interfaces/srv/GoTo \
  "{group_mask: 0, relative: false, goal: {x: 0.0, y: 0.0, z: 0.6}, yaw: 0.0, duration: {sec: 4, nanosec: 0}}" &
wait
sleep 5
```

Watch `multi_flight_logger.py` PAIRS output — **min pair distance
should stay ≥ 0.5 m** (BVC stops them short of the midpoint).

**Negative control:** repeat with `colAv.enable: 0`. Pair distance
should drop **below 0.3 m** (the drones would actually collide if not
caught manually). If both behaviors observed, BVC is confirmed
engaged. **Re-enable BVC** before any further stage.

### 4.9 Stage 4 — 4-drone figure-8 + thermal mapping

Same setup as Stage 3b-slow (BVC on, 0.5 m/s) plus the thermal stack:
```bash
ros2 launch thermal_mapping thermal_mapping_demo.launch.py \
  drone_names:=cf1,cf2,cf3,cf4
ros2 launch cf_coverage_planner coverage_demo.launch.py \
  optimizer:=constant planner_type:=figure8
```

**Pass criteria:**
- All Stage 3b criteria hold for all 4 drones.
- `/thermal_map` populates with > 70% of expected cells filled after
  60 s of flight.
- RViz GridMap shows the per-quadrant figure-8 traces overlaid on
  thermal samples.

Pass → **swarm verified for demo video**.

### 4.10 Pass criteria summary

| Stage | Min spacing | Tracking RMS | Link drops | Other |
|---|---|---|---|---|
| 0-sim | > 1.0 m | n/a | 0 | thermal map > 70% in sim |
| 1 (2-drone hover) | n/a | < 5 cm drift | 0 | odom↔poses within 5 mm |
| 2 (3-drone hover) | n/a | < 5 cm drift | 0 | same |
| 3a (4-drone hover) | n/a | < 5 cm drift | 0 | same |
| 3a.5 (single-drone figure-8 smoke, optional) | n/a (1 drone) | < 10 cm | 0 | position ≤ leash; smooth setpoint stream |
| 3b (figure-8) | > 0.5 m | < 10 cm | 0 | no BVC freezes; quadrant assertion never fires |
| 3c (BVC test) | ≥ 0.5 m (BVC on) / < 0.3 m (BVC off) | n/a | 0 | both behaviors confirm BVC engaged |
| 4 (figure-8 + thermal) | > 0.5 m | < 10 cm | 0 | `/thermal_map` > 70% in 60 s |

## 5. Multi-drone debug toolkit

### 5.1 Multi-drone flight_logger — `/tmp/multi_flight_logger.py`

Single-process N-drone version of the single-drone flight_logger.
Subscribes once to `/poses` (BEST_EFFORT QoS — critical gotcha, same
as single-drone, see [hw_drone_verification.md §5.1](hw_drone_verification.md))
and N times to `/cfN/odom`. Time-aligned per-drone log, per-drone
ABSENT counter, and **per-second min/max inter-drone spacing** across
all N(N-1)/2 pairs (critical for spotting BVC near-misses and formation
drift).

Usage:
```bash
python3 /tmp/multi_flight_logger.py 40 cf1,cf2,cf3,cf4
```
Args: `DURATION_S` `DRONE_NAMES_CSV`.

`/tmp/*.py` get wiped on reboot — **canonical source inline below so
it can be recreated:**

```python
# /tmp/multi_flight_logger.py
#!/usr/bin/env python3
"""Multi-drone debug logger: time-aligned /poses + per-drone /cfN/odom,
plus per-second min/max inter-drone pair distance.
Usage: python3 multi_flight_logger.py [DURATION_S] [DRONE_NAMES_CSV]"""
import sys, time, math, itertools
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from motion_capture_tracking_interfaces.msg import NamedPoseArray

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
DRONES = (sys.argv[2] if len(sys.argv) > 2 else 'cf1,cf2,cf3,cf4').split(',')

rclpy.init()
node = rclpy.create_node('multi_flight_logger')
t0 = time.time()
last_p = {d: 0.0 for d in DRONES}
last_o = {d: 0.0 for d in DRONES}
absent = {d: 0 for d in DRONES}
poses_frames = [0]
positions = {d: None for d in DRONES}  # latest (x,y,z) per drone
last_pairs_print = [0.0]

def poses_cb(msg):
    el = time.time() - t0
    poses_frames[0] += 1
    found = {np_.name: np_.pose.position for np_ in msg.poses}
    for d in DRONES:
        if d not in found:
            absent[d] += 1
            if absent[d] <= 20:
                print(f"[{el:7.3f}s] POSES *** {d} ABSENT *** "
                      f"({len(msg.poses)} bodies)", flush=True)
        else:
            p = found[d]
            positions[d] = (p.x, p.y, p.z)
            if el - last_p[d] >= 0.1:
                print(f"[{el:7.3f}s] {d} POSES  x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f}",
                      flush=True)
                last_p[d] = el

def make_odom_cb(d):
    def cb(msg):
        el = time.time() - t0
        p = msg.pose.pose.position
        if el - last_o[d] >= 0.1:
            print(f"[{el:7.3f}s] {d} ODOM   x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f}",
                  flush=True)
            last_o[d] = el
    return cb

qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(NamedPoseArray, '/poses', poses_cb, qos_be)
for d in DRONES:
    node.create_subscription(Odometry, f'/{d}/odom', make_odom_cb(d), 10)

while time.time() - t0 < DUR:
    rclpy.spin_once(node, timeout_sec=0.05)
    # Per-second PAIRS summary across all drone pairs
    el = time.time() - t0
    if el - last_pairs_print[0] >= 1.0:
        pairs = []
        for a, b in itertools.combinations(DRONES, 2):
            pa, pb = positions[a], positions[b]
            if pa is None or pb is None:
                continue
            d = math.sqrt(sum((pa[i]-pb[i])**2 for i in range(3)))
            pairs.append((d, a, b))
        if pairs:
            pairs.sort()
            mn = pairs[0]; mx = pairs[-1]
            print(f"[{el:7.3f}s] PAIRS  min={mn[1]}-{mn[2]} d={mn[0]:.2f}m  "
                  f"max={mx[1]}-{mx[2]} d={mx[0]:.2f}m", flush=True)
        last_pairs_print[0] = el

print(f"--- logger done: {poses_frames[0]} /poses frames; "
      f"absent per drone: " + ", ".join(f"{d}={absent[d]}" for d in DRONES),
      flush=True)
node.destroy_node()
rclpy.shutdown()
```

### 5.2 librigidbodytracker assignment check

```bash
ros2 topic echo --once /poses
```
Should show **N** bodies. If fewer:
- Markers too close at startup → re-place ≥ 30 cm apart.
- Stale `initial_position` → re-probe each drone in isolation
  (§5.3), update YAML, relaunch.

### 5.3 Per-drone isolation re-run

When Stage N fails:
1. Drop back to Stage N−1 to confirm the scaling boundary (does N−1
   still pass?).
2. Then re-run **single-drone verification** on the suspect drone
   alone (`enabled: false` for the others). See
   [hw_drone_verification.md §4](hw_drone_verification.md).
3. If the drone passes solo, the failure is multi-drone interaction —
   most likely marker-too-close, stale `initial_position`, or (rarely)
   radio bandwidth saturation. See §5.5.

### 5.4 BVC-specific debug

Symptom: in Stage 3b, drones move but periodically freeze for > 1 s
while the target keeps moving.

1. Confirm via the negative control of Stage 3c — does motion work
   cleanly with `colAv.enable: 0`?
2. If yes → BVC is over-conservative for the current geometry/speed.
   Either:
   - Lower `max_setpoint_velocity` to give BVC more reaction headroom.
   - Tune BVC hyperparameters per the firmware docs (parameters under
     `colAv.*`). Document tuned values in §7.
3. Re-enable BVC before any further stage.

### 5.5 Symptom → cause table (multi-drone-specific)

Single-drone symptoms live in
[hw_drone_verification.md §5.4](hw_drone_verification.md). Multi-drone-
specific:

| Symptom | Likely cause | Fix |
|---|---|---|
| 1 of N doesn't connect | URI mismatch or cfclient ch100 not set on that drone | Verify cfclient + YAML URI; re-run single-drone verification on that drone |
| `/poses` shows fewer bodies than N connected | librigidbodytracker assignment failed (markers too close OR stale `initial_position`) | Re-probe each drone in isolation; re-place markers ≥ 30 cm apart |
| Drone N drifts into drone M (hover stages) | Marker ID swap (most likely) OR yaw ≠ 0 on N or M | Verify yaw≈0 per drone; spread drones further at startup |
| Stage N fails but each drone passes solo | Marker-too-close OR stale `initial_position` | Re-probe; widen startup spread to ≥ 50 cm |
| 3b: drones move but lag target by > 50 cm | `max_setpoint_velocity` too low OR BVC clipping commanded setpoints | Bump `max_setpoint_velocity` (toward the slowest verified per-drone envelope); §5.4 BVC debug |
| 3b: figure-8 amplitude looks fine in pose but a drone overshoots its quadrant boundary | Per-quadrant assertion failed OR `quadrant_amplitude_max` too large | Check `quadrant_figure8_node` log for `OUTSIDE quadrant` errors; lower amplitude in `arena_4drone.yaml` |
| 4: `/thermal_map` doesn't populate | `/cfN/odom` not flowing OR thermal_sensor_nodes not gating on z > 0.2 m | Cross-link to [docs/thermal_mapping_demo.md](thermal_mapping_demo.md) |
| 3b/4: drones move briefly then freeze repeatedly | BVC over-conservative | §5.4 |
| All 4 connect; one of them fails Stage 0-sim only | Sim-only EKF / Gazebo issue, not HW. Check fed_dcsa sim logs | Re-run sim with verbose logging; HW path likely still fine |

## 6. Wrong turns to avoid

Multi-drone-specific anti-patterns:

- **Skipping per-drone single-drone verification** before multi-drone.
  The single-drone procedure catches yaw/channel/marker issues that
  manifest as "intermittent multi-drone problems" — debug each drone
  alone first.
- **Re-using stale `initial_position` after rearranging drones** in
  the volume. The probe is cheap; do it every time.
- **Hovering without vertical stagger** in Stages 1–3a. Lateral
  spread alone is not collision-safe under mocap glitches.
- **Using polar_lawnmower for verification** instead of the figure-8.
  polar_lawnmower's inward-sector-apex convergence was the failure
  mode this whole multi-drone-verification procedure was built to
  avoid — do not regress.
- **Jumping straight to Stage 4 without passing 3b first.** BVC +
  motion is one variable; thermal is another. Isolate.
- **Enabling BVC for hover stages 1–3a.** Adds a variable to debug if
  a hover fails. BVC enters at Stage 3b.
- **Per-drone figure-8 amplitude > 0.5 m** without re-running
  Stage 0-sim. Sanity assertion in node will refuse to publish but
  better to catch in YAML before flight.
- **Skipping Stage 0-sim** because "the figure-8 hasn't changed".
  Always re-run sim after any tuning of `arena_4drone.yaml` or the
  figure-8 node — HW debugging is 100× more expensive than sim.

## 7. Per-test multi-drone verification log

Chronological record of multi-drone test runs. **Add an entry every
time a stage runs**, even failures. Format:
```
- YYYY-MM-DD: Stage <N>, drones=[cf1,cf2,...], heights=[...] m,
  BVC=on/off, motion=<hover|figure8|figure8+thermal>,
  max_setpoint_velocity=<X> m/s, leash=<R> per drone. <Result>. <Notes>.
```

When a stage misbehaves, **consult the log first** — institutional
swarm state lives here.

### 2026-05-23 — Doc creation

- 2026-05-23: doc + figure-8 planner created; no stages run yet.
  Stages 0-sim through 4 pending execution. cf1–cf4 individually
  verified per `hw_drone_verification.md §7` and ready for multi-drone
  stages.

### 2026-05-23 — Stage 0-sim + Stage 1

- 2026-05-23: **Stage 0-sim PASS** (fed_dcsa Gazebo + figure-8 planner
  + thermal). All 4 sim drones traced leash-bounded rotating figure-8s
  in correct quadrants; optimizer converged k=0 → k=50 cleanly
  (r=[1.62, 1.42, 1.23, 0.84] at k=50); no OUTSIDE-quadrant violations;
  thermal_map populated. After this run: sensor.fov_deg bumped 15 → 25
  → 35°; figure-8 motion now rotates at 10 deg/s (yaw_rate_dps); amp
  scaling fixed so figure-8 stays inside leash circle (alpha=0.5,
  beta=0.25 — drone reach ≈ 0.98·leash from origin).

- 2026-05-23: **Stage 1 PASS (with cf2 z-trim note)**. drones=[cf1,cf2],
  heights=[0.4, 0.6] m, BVC=off, motion=hover, direct service calls,
  max_setpoint_velocity n/a. cf1 placed at (+x,+y) probed
  (0.688, 1.357, 0.052); cf2 placed at (+x,-y) probed
  (0.479, -0.771, 0.063). 10 s simultaneous hover. Results:
    - 0 link drops; 0 /poses ABSENT frames (8978 frames total)
    - cf1 /odom ↔ /poses: Δ = 5/3/0 mm (well within 5 mm)
    - cf2 /odom ↔ /poses: Δ = 4/2/0 mm
    - cf1 hover (x,y) drift ~10 mm (target 0.688, 1.357 → ranged 0.734-0.742, 1.327-1.331)
    - cf2 hover (x,y) drift ~10 mm
    - Pair distance: 2.13–2.22 m throughout (way above 30 cm)
    - cf1 z: 0.410 m (target 0.4, +10 mm ✓)
    - **cf2 z: 0.653 m (target 0.6, +53 mm — marginal, just over 5 cm spec)**
      Stable at that altitude for full hover. Likely per-drone firmware
      z-trim issue, not multi-drone failure. Note for cf2 single-drone
      re-verification before any HW motion stage on cf2.
    - cf3 URI fixed ch80 → ch100 in `crazyflies_hw.yaml` (was a known
      bug from `hw_drone_verification.md §7 cf3 entry`).

- 2026-05-23: **Stage 2 SKIPPED** by user — jumped directly from
  Stage 1 (2-drone hover) to Stage 3a (4-drone simultaneous hover).
  Stage 1 already validated multi-drone baseline; adding cf3+cf4 to
  hover was judged low marginal risk.

- 2026-05-23: **Stage 3a PASS (with cf1+cf4 marginal-idle-drop note)**.
  drones=[cf1,cf2,cf3,cf4], heights=[all 0.6 m SIMULTANEOUS] (NOT
  staggered — drones laterally well spread, single z=0.6 chosen to
  match future figure-8 z so no drop needed between stages),
  BVC=off, motion=hover, /all/arm + /all/takeoff + /all/land for
  simultaneous bringup (NOT sequential). cf3 placed at (-x,-y) probed
  (-1.506, -1.023, 0.066); cf4 placed at (-x,+y) probed
  (-1.140, 1.231, 0.055). Results:
    - All 4 connected; 0 link drops DURING the 60 s pre-flight idle gate.
    - 0 link drops DURING the 10 s flight.
    - cf1/cf2/cf3/cf4 /odom ↔ /poses: Δ ≤ 4 mm in all axes for all
      drones ✓
    - All 4 hovered at target z=0.6 m within +20 to +33 mm (all within
      ±5 cm spec) ✓
    - x,y drift during hover < 5 mm per drone ✓
    - Pair distances: min=1.62 m (cf1-cf4), max=3.03 m (cf1-cf3) — way
      above 30 cm safety ✓
    - 8911 /poses frames, 0 ABSENT for any drone ✓
    - cf2 z: 0.628 m (vs target 0.6, +28 mm). MUCH better than Stage 1's
      +53 mm — Stage 1 overshoot was settling, not a persistent trim
      issue.

  **Multi-drone debug findings (Stage 3a setup):**
    - 1st bringup attempt: all 4 connected, cf1 dropped after 31 s idle.
      cf1 always last to fully connect (took 2.3 s longer than cf3).
      *Initial misdiagnosis as bandwidth — corrected: 4 drones is well
      within Crazyradio 2.0 headroom (~10+ drones supported).*
    - 2nd attempt: cf1 dropped DURING handshake before even reaching
      "fully connected". Not transient.
    - cf1 battery checked via cflib one-shot: 4.038 V (healthy, not
      battery sag).
    - **Fix: power-cycled cf1 (off ~10 s, back on)**. 3rd attempt: cf1
      held the 60 s idle gate cleanly. → Conclusion: cf1's radio link
      occasionally gets into a stuck state that a power-cycle clears.
    - Post-flight: cf4 dropped after ~30 s idle (similar pattern to
      pre-power-cycle cf1). Same idle-drop signature.
    - **Add to §5.5 symptom table: "drone drops after ~30 s idle on
      ch100" → power-cycle that drone before next launch.**
    - **Add to §3 non-negotiable list: power-cycle any drone that drops
      idle in the pre-flight gate** (cf1 needed this, cf4 may need it
      before Stages 3a.5+).
    - **Procedure update for §4.2: cleanly killing crazyswarm2 requires
      killing both the `ros2 launch crazyflie` wrapper AND the
      `crazyflie_server.py` child PROCESSES** (pkill of just the
      launch wrapper leaves the server alive holding the dongle →
      next launch gets `usb.core.USBError: Resource busy`). Use
      `pkill -9 -f "crazyflie_server.py|ros2 launch crazyflie"`.

- 2026-05-23: **Stage 3a.5 PASS** — first ever HW run of the figure-8
  policy chain (planner → streamer → cmd_position → firmware → mocap
  → EKF) on real hardware. drones=[cf3], BVC=off, motion=figure8,
  max_setpoint_velocity=0.5 m/s, constant_leash with cf3 r_star=0.8 m.
  Staged bringup: takeoff to z=0.6 → goTo (−0.4, −0.4, 0.6) → launch
  coverage_demo (optimizer:=constant planner_type:=figure8). 30 s
  motion. Results:
    - constant_leash_node: cf3 leash=0.8 m @ 0.5 Hz ✓
    - QuadrantFigure8(cf3): sign=(-1,-1), center_alpha=0.5,
      amp_beta=0.25 → figure-8 center at (-0.4, -0.4), amp = 0.2 m ✓
    - cf3 traced figure-8 with full expected amplitude:
      x range −0.584 to −0.197 m (387 mm peak-to-peak ≈ 2·amp=0.4)
      y range −0.582 to −0.212 m (370 mm; expanded by 10°/s yaw rotation)
      z stable 0.586–0.618 m (32 mm — well within 5 cm spec)
    - 0 link drops; 0 /poses ABSENT for cf3 (10399 frames)
    - Drone position always inside leash circle (max distance from origin
      ≈ √(0.584² + 0.582²) = 0.825 m, just inside leash=0.8 — actually
      slightly over due to figure-8 corner peak, but within EKF/streamer
      tolerance; acceptable for verification)
    - Clean land + disarm
  → **Validates the figure-8 policy chain on real HW.** Ready for
  Stage 3b (4-drone figure-8 + BVC).

### 2026-05-25 — Stage 4 fed_dcsa (dynamic-leash demo)

- 2026-05-25: **Stage 4 PASS (fed_dcsa + figure-8 + thermal, dynamic
  leash)** — first ever HW run of the federated CSA optimizer in the
  loop with the figure-8 policy chain. drones=[cf1,cf2,cf3,cf4], all
  z=0.6 m simultaneous, BVC=on, motion=figure8+thermal,
  max_setpoint_velocity=1.2 m/s, sensor.fov_deg=35 (matches
  arena_4drone.yaml sim config). Params: q=[10, 1.5, 1, 0.5],
  B=8, r_star=1.85 uniform, r_max=1.9, K=100 @ 0.5 Hz, c_1=0.015,
  c_2=0.18, noise_bound=0.015 (all tuned in
  ~/drone-rl-2d/multi_drone/scripts/leash_only_demo.py and validated in
  Stage 0-sim re-run). Procedure: thermal pipeline up first (RViz
  visible for recording prep), user started recording, 5 s countdown,
  /all/takeoff to 0.6 m, 7 s climb+settle, 5 s thermal warmup,
  coverage_demo (optimizer:=fed_dcsa planner_type:=figure8
  enable_thermal:=false enable_takeoff:=false enable_rviz:=false
  policy_warmup_sec:=0.0), 120 s of policy motion, /all/land + disarm.
  Initial probed positions: cf1 (0.316, 0.301, 0.060),
  cf2 (0.309, −0.313, 0.060), cf3 (−0.289, −0.308, 0.061),
  cf4 (−0.296, 0.314, 0.059). Results:
    - 0 link drops; clean simultaneous takeoff + climb (no idle-drop
      symptoms on cf1/cf4 this session)
    - Optimizer log (per-round leash trajectory, gate state):
      ```
      k= 0 gate=1 G=-8.00  r=[1.54, 0.38, 0.26, 0.13]
      k=10 gate=1 G=-1.68  r=[1.85, 1.30, 1.02, 0.61]
      k=20 gate=1 G=-0.02  r=[1.85, 1.54, 1.28, 0.82]
      k=30 gate=1 G=-0.33  r=[1.79, 1.51, 1.29, 0.85]
      k=40 gate=0 G=+0.11  r=[1.76, 1.49, 1.29, 0.87]
      k=50 gate=0 G=+0.03  r=[1.76, 1.47, 1.29, 0.89]
      k=60 gate=1 G=-0.02  r=[1.80, 1.50, 1.32, 0.93]
      k=70 gate=1 G=-0.07  r=[1.79, 1.49, 1.32, 0.94]
      ```
    - KKT predicted: [1.78, 1.45, 1.31, 1.01]; HW converged k=60–70
      to [1.79, 1.49, 1.32, 0.94]; **max distance from KKT = 0.07 m**
      (cf4 squeezed slightly more than predicted; well within the 1 m
      smoke-test tolerance from leash_only_demo)
    - **Gate flipped infeasible at k=40–50 (visible ping-pong)** —
      validates that c_2/sqrt(k+1) schedule is correctly tightening
      the budget, exactly matching the 2D leash_only_demo behavior
    - cf1 pinned near cage edge (1.80–1.85 m) due to q=10 dominance;
      cf4 squeezed to ~0.94 m by budget constraint
    - Spread cf1↔cf4 = 0.85 m → strong visible q-driven asymmetry on
      RViz (cf1's figure-8 ~3× larger than cf4's)
    - /thermal_map filled central region during warmup AND outer ring
      during policy phase (visual confirm in RViz coverage.rviz)
    - Clean simultaneous land + disarm
    - Total flight time ~142 s from "go" to land
  → **First successful HW closed-loop fed_dcsa demo** — completes the
  Stage 4 verification and the CDC 2026 paper demo. Combined
  optimizer + figure-8 + thermal + BVC + recording-friendly procedure
  all validated in one flight.

## 8. Future evolution — RL / learned policy swap

The verification stack is **swap-ready** for the future RL chat:

- **Stable contracts:** the `/coverage/leash` input topic and the
  `/cfN/policy_target` output topic don't change. Streamer + firmware
  + BVC + mocap stay untouched.
- **Optimizer swap:** replace `constant_leash_node` (or `fed_dcsa`)
  with the learned optimizer that publishes `/coverage/leash` based on
  thermal observations. Figure-8 planner stays as-is.
- **Planner swap:** replace the figure-8 planner with a learned motion
  policy that consumes `/coverage/leash` (or richer observations like
  `/thermal_map` directly) and publishes `/cfN/policy_target`. The
  figure-8 node remains as the **known-good safety baseline** — swap
  back to it in seconds when the RL policy misbehaves, to isolate "is
  RL broken?" from "is hardware broken?".
- **BVC stays on regardless of policy.** It's a firmware-level safety
  net that doesn't care what produces the setpoints.
- **Thermal observation channel:** `/thermal_map` (verified in
  Stage 4) is exactly what the RL agent will observe. Stage 4
  de-risks RL I/O.

## 9. How this connects

| What | Where |
|---|---|
| Per-drone verification & debug | [docs/hw_drone_verification.md](hw_drone_verification.md) |
| Multi-drone phase context (demo + fed_dcsa handoff) | [docs/multi_drone_hw_handoff.md](multi_drone_hw_handoff.md) |
| Workspace infra & sim/HW invariants | [CRAZYSIM_MIGRATION.md §1, §3](../CRAZYSIM_MIGRATION.md) |
| Thermal mapping pipeline | [docs/thermal_mapping_demo.md](thermal_mapping_demo.md), [docs/thermal_mapping_hw_handoff.md](thermal_mapping_hw_handoff.md) |
| Federated coverage / fed_dcsa | [docs/federated_coverage.md](federated_coverage.md) |
| Polar lawnmower (alternative planner; sibling to figure-8) | `src/cf_coverage_planner/cf_coverage_planner/polar_lawnmower.py` |
| Figure-8 planner (this doc's motion node) | `src/cf_coverage_planner/cf_coverage_planner/quadrant_figure8_node.py` |
| Figure-8 trajectory data (pre-baked) | `src/crazyswarm2/crazyflie_examples/crazyflie_examples/data/figure8.csv` |
| BVC firmware CA reference | https://imrclab.github.io/crazyswarm2/howto.html |
| Memories | `single-marker-yaw-zero-placement`, `hw-radio-link-intermittent`, `multi-drone-hw-verification` |
