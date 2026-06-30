# Hardware drone verification — single-drone procedure + debug guide

Written 2026-05-23 — **living document** (per-drone log in §7, updated
every test run).

The workspace's **canonical per-drone HW verification procedure** for
the single-marker mocap method — **workspace infrastructure**, not a
transitional handoff. Used standalone for any drone bringup,
post-repair re-verification, or new-drone onboarding, AND invoked
per-drone by handoff docs that need N drones live (e.g.,
[docs/multi_drone_hw_handoff.md](multi_drone_hw_handoff.md)).

This doc serves **two purposes**:

1. **Procedure + debug toolkit (§1–6)** — canonical reference for how
   to verify and debug a single drone on this workspace.
2. **Per-drone verification log (§7)** — chronological record per
   drone of every verification / envelope test / post-repair check.
   **Add an entry every time you run a per-drone test on any drone.**
   When a drone misbehaves, check §7 first — "did it ever pass? when?
   what changed since?"

Encodes the lessons from the 2026-05-20 → 23 single-drone debug saga —
the two real bugs we fixed, the procedure that works, and the debug
techniques that cracked it. **Read §3 (the non-negotiables) and §5
(debug techniques) before any HW flight.**

## 1. What this doc is for

Single-drone hardware bringup and verification. Output: a drone known to
(a) hold a stable radio link, (b) have a correctly-fused mocap+EKF
position estimate, and (c) take off, hover stably, and land cleanly
under the firmware controller.

When this passes, the drone is ready for multi-drone simultaneous work,
thermal mapping, fed_dcsa, etc. When it doesn't, §5's debug toolkit
isolates whether the failure is mocap-side, firmware-side, or hardware.

## 2. Prerequisites

Hardware:
- Crazyflie 2.1+ on firmware **2025.12** (NOT 2026.04 — that release
  removed the V1 log/param protocol that crazyswarm2 v1.0.3 uses; see
  [CRAZYSIM_MIGRATION.md §3](../CRAZYSIM_MIGRATION.md))
- Crazyradio dongle (PA or 2.0) — `lsusb` VID `1915`
- Single retroreflective marker on top of the drone, centered over the
  IMU plate (Path Z / single-marker — see CRAZYSIM_MIGRATION.md §3 for
  why this marker config)
- Vicon mocap volume (DataStream on `192.168.0.62:801`)

Software:
- cs2_ws built with `colcon build --symlink-install`
- `config/motion_capture.yaml` configured for Vicon
- `config/crazyflies_hw.yaml` with the drone listed (URI +
  `initial_position`)
- `pyvicon_datastream` python package available (for the debug scripts
  in §5)

## 3. The two non-negotiables (real bugs + fixes)

Both must hold for ANY HW flight on this setup. Skipping either causes
predictable, deterministic failure — we proved this the hard way over
three days of crashes.

### 3.1 Wrong initial yaw → place at yaw≈0

**Mechanism.** Single-marker mocap gives the firmware *position only*,
no orientation. The Crazyflie EKF assumes yaw=0 at boot ("body +X
aligned with Vicon +X") and integrates the gyro from there — single-
marker never corrects it. If the drone is placed at a random
orientation, the firmware's yaw is wrong → the position controller's
world→body conversion is wrong → it cannot hold position on takeoff →
drifts → outruns the mocap dynamics check → `/poses` drops the drone →
EKF starves → diverges → runaway → crash.

**Signature.** Drone drifts away on takeoff while `/cfN/odom` is *still
matching* `/poses` (the EKF estimate is correct; the controller is
turning correct estimates into wrong commands because of yaw). Drift
direction varies between runs (depends on the random placement yaw).

**Fix.** Place each drone with its **front (+X)** pointing along the
**Vicon +X axis**, ~10–20° eyeball is plenty. Now the firmware's
assumed yaw=0 matches physical yaw=0 → frame conversion correct →
drone holds position on takeoff.

See memory: `single-marker-yaw-zero-placement`.

### 3.2 Channel-80 WiFi interference → channel 100

**Mechanism.** Crazyradio channel 80 = 2480 MHz sits inside the 2.4 GHz
WiFi band (2401–2483 MHz). In a busy lab, WiFi traffic stomps on the
Crazyradio link → intermittent `Too many packets lost` → drone drops off
the radio every ~30–120 s.

**Signature.** Bringup log shows `Got link error callback [Too many
packets lost]` → `[cfN] is disconnected!`. The drop window varies
(20–120 s) and appears even while the drone is sitting idle, disarmed.

**Fix.** Move every drone to **channel 100** (2500 MHz, above the
consumer WiFi band):
1. In cfclient: connect to the drone → Configure → set radio channel to
   100. Stored on the drone itself.
2. Update the YAML URI: `radio://0/100/2M/E7E7E7E7E[N]`.

See memory: `hw-radio-link-intermittent`.

## 4. The verification procedure

Run start-to-finish for any drone. Pass criteria at the end (§4.6).

### 4.1 Drone prep (user)

1. In cfclient: confirm the drone is on **firmware 2025.12** and
   **channel 100**. If not, flash / reconfigure.
2. Stick a fresh charged battery on. Verify props spin freely (no
   grinding, no bends — quick sanity if the drone has taken any hits).
3. Place in the arena with **front (+X) along Vicon +X** (~10–20° by
   eye). Marker upright on top.
4. Power on.

### 4.2 Config — `config/crazyflies_hw.yaml`

For the drone under test:
```yaml
cfN:
  enabled: true
  uri: radio://0/100/2M/E7E7E7E7E[N]
  initial_position: [x, y, z]   # from a Vicon probe — see §4.3
  type: cf21
```
Disable other drones (`enabled: false`) so the stack only tries this one.

### 4.3 Probe Vicon for `initial_position`

```bash
cd ~/cs2_ws && python3 /tmp/vicon_probe.py 5
```
(Script inline in §5.3.) Output should show **exactly 1 unlabeled
marker** at the drone's spot. Copy those `(x, y, z)` values into the
YAML's `initial_position`.

- 0 markers → drone not in the volume, not powered, or marker missing.
- ≥ 2 markers → another reflector in the volume; remove it (or
  identify which marker is the drone's and use that position).

### 4.4 Launch flight stack + idle-link gate

```bash
cd ~/cs2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
nohup ros2 launch crazyflie launch.py \
  backend:=cflib \
  crazyflies_yaml_file:=$HOME/cs2_ws/config/crazyflies_hw.yaml \
  motion_capture_yaml_file:=$HOME/cs2_ws/config/motion_capture.yaml \
  gui:=false > /tmp/hw_bringup.log 2>&1 &
```
Wait for `[cfN] is fully connected!` in `/tmp/hw_bringup.log`.

**60-second idle-link gate** — watch for radio drops *before* arming
(channel 100 should hold; this catches dongle/USB issues):
```bash
DROPPED=0; for i in $(seq 1 4); do
  sleep 15
  if grep -qE "Too many packets|is disconnected" /tmp/hw_bringup.log; then
    echo ">>> LINK DROPPED — do NOT fly. Re-seat dongle or change USB port."
    DROPPED=1; break
  fi
  echo "  $((i*15))s: link up"
done
[ "$DROPPED" -eq 0 ] && echo ">>> 60s idle clean — clear to fly"
```

On channel 100 the link is rock-solid in a normally-busy lab. If the
60s gate fails, the issue is **dongle-side** (re-seat in a different
USB port — short, good-quality cable; not a hub).

### 4.5 Instrumented takeoff-hover-land

Start the Vicon monitor (true position via raw markers) and the flight
logger (`/poses` + `/cfN/odom` time-aligned) in the background:
```bash
nohup python3 /tmp/vicon_monitor.py 40 > /tmp/vicon_test.log 2>&1 &
sleep 1
nohup python3 /tmp/flight_logger.py 40 cfN > /tmp/flight_logger.log 2>&1 &
sleep 2
```
(Scripts inline in §5.1 and §5.2. Replace `cfN` with the actual drone
name, e.g. `cf1`.)

Then arm, takeoff, hover, land, disarm:
```bash
ros2 service call /cfN/arm crazyflie_interfaces/srv/Arm "{arm: true}"
sleep 1
ros2 service call /cfN/takeoff crazyflie_interfaces/srv/Takeoff \
  "{group_mask: 0, height: 0.5, duration: {sec: 3, nanosec: 0}}"
sleep 6   # 3s climb + 3s settle
ros2 service call /cfN/land crazyflie_interfaces/srv/Land \
  "{group_mask: 0, height: 0.05, duration: {sec: 3, nanosec: 0}}"
sleep 5
ros2 service call /cfN/arm crazyflie_interfaces/srv/Arm "{arm: false}"
```

### 4.6 Pass criteria

After the flight, all of:
1. Monitor: **0 anomalies** —
   `grep -c ANOMALY /tmp/vicon_test.log`.
2. Bringup log: **0 link drops** —
   `grep -cE 'Too many packets|is disconnected' /tmp/hw_bringup.log`.
3. flight_logger: `/poses` and `/cfN/odom` **match within ~3–5 mm**
   throughout climb, hover, and descent.
4. Drone held its takeoff `(x, y)` **within a few cm** while hovering.
5. Clean descent: z = 0.5 → 0.05 monotonic.

All pass → drone verified.

Any fail → §5's debug toolkit. The breakpoint between `/poses` and
`/cfN/odom` (from the flight logger) is the diagnostic: where the chain
breaks tells you mocap-side vs firmware-side.

## 5. Debug techniques (institutional knowledge)

Read this section **first** when something goes wrong. The instrumented-
logger pattern (§5.1) is THE one tool that cracked the multi-day debug
saga — reach for it *before* blaming hardware.

### 5.1 Instrumented logger — `/tmp/flight_logger.py`

A small rclpy node that subscribes to `/poses` (mocap output) and
`/cfN/odom` (firmware EKF) and logs them time-aligned. The breakpoint
between the two is THE diagnostic:

- `/poses` drops the drone (no more frames carrying that drone) →
  **mocap-side** failure (dynamics check, marker, Vicon stream).
- `/poses` keeps the drone but `/cfN/odom` diverges → **firmware-side**
  (EKF didn't get / apply extpos; usually radio drop, OR wrong yaw at
  boot, OR commander timeout).

**CRITICAL GOTCHA:** `/poses` is published with `BEST_EFFORT` QoS. Any
subscriber MUST use `BEST_EFFORT` too — the rclpy default is `RELIABLE`
and silently drops the messages. We lost ~20 minutes to this.

`/tmp/*.py` get wiped on reboot — **canonical sources inline below so
they can be recreated.**

```python
# /tmp/flight_logger.py
#!/usr/bin/env python3
"""Debug logger: time-aligned /poses (mocap output) + /cfN/odom (firmware EKF).
Flags every frame where the drone is ABSENT from /poses (tracker dropped it).
Usage: python3 flight_logger.py [DURATION_S] [DRONE_NAME]"""
import sys, time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from motion_capture_tracking_interfaces.msg import NamedPoseArray

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 50.0
DRONE = sys.argv[2] if len(sys.argv) > 2 else 'cf2'

rclpy.init()
node = rclpy.create_node('flight_logger')
t0 = time.time()
last_p = [0.0]; last_o = [0.0]
absent = [0]; poses_frames = [0]

def poses_cb(msg):
    el = time.time() - t0
    poses_frames[0] += 1
    found = None
    for np_ in msg.poses:
        if np_.name == DRONE:
            found = np_.pose.position
            break
    if found is None:
        absent[0] += 1
        if absent[0] <= 40:
            print(f"[{el:7.3f}s] POSES *** {DRONE} ABSENT *** "
                  f"({len(msg.poses)} bodies in frame)", flush=True)
    elif el - last_p[0] >= 0.1:
        print(f"[{el:7.3f}s] POSES  x={found.x:+.3f} y={found.y:+.3f} "
              f"z={found.z:+.3f}", flush=True)
        last_p[0] = el

def odom_cb(msg):
    el = time.time() - t0
    p = msg.pose.pose.position
    if el - last_o[0] >= 0.1:
        print(f"[{el:7.3f}s] ODOM   x={p.x:+.3f} y={p.y:+.3f} "
              f"z={p.z:+.3f}", flush=True)
        last_o[0] = el

# CRITICAL: /poses is BEST_EFFORT — must match or no messages arrive
qos_be = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(NamedPoseArray, '/poses', poses_cb, qos_be)
node.create_subscription(Odometry, f'/{DRONE}/odom', odom_cb, 10)

while time.time() - t0 < DUR:
    rclpy.spin_once(node, timeout_sec=0.05)

print(f"--- logger done: {poses_frames[0]} /poses frames, "
      f"{absent[0]} with {DRONE} ABSENT ---", flush=True)
node.destroy_node()
rclpy.shutdown()
```

### 5.2 Vicon monitor — `/tmp/vicon_monitor.py`

Reads raw Vicon (unlabeled markers) directly via `pyvicon_datastream`.
Sanity-checks the mocap layer **independently of**
`motion_capture_tracking_node`: if a marker is in Vicon but missing
from `/poses`, the ROS tracker dropped it (not Vicon).

Logs marker count + xyz with a 1 Hz heartbeat; flags anomalies
(count ≠ 1) which usually mean occlusion or phantom reflections.

**CRITICAL CAVEAT — labeled rigid bodies in Vicon Tracker.** This
monitor reads ONLY the **unlabeled** marker stream. If your Vicon
admin has defined any drone as a Vicon-side rigid body (a "subject"
in Tracker), that drone's marker goes into the **labeled** stream and
is **invisible to this monitor**. In that case:

- The monitor's reported markers are **strays only** (reflective tape,
  unmasked stationary reflectors, etc.) — not the drones you're flying.
- The monitor may show `count=1` with a fixed position that looks like
  a drone "stuck on the ground" — that's a stray reflector at that
  location, NOT the drone.
- The flight_logger (§5.1) is the authoritative source in this case:
  `/poses` shows the actual drone position regardless of
  labeled/unlabeled split.

**Rule of thumb:** trust the flight_logger over the Vicon monitor when
they conflict. The monitor is useful for confirming "no stray
reflectors / clean volume" before flying (count=0 = clean) and for
catching marker dropouts when no Vicon subjects are defined. It is
*not* a substitute for `/poses` when Vicon has labeled rigid bodies.

(Discovered the hard way in cf1's dogfood 2026-05-23 — see §7.)

```python
# /tmp/vicon_monitor.py
#!/usr/bin/env python3
"""Monitor Vicon unlabeled markers during a flight."""
import sys, time
from pyvicon_datastream import PyViconDatastream, Result

HOST = "192.168.0.62:801"
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

c = PyViconDatastream()
if c.connect(HOST) != Result.Success:
    print("connect failed", flush=True); sys.exit(1)
c.enable_unlabeled_marker_data(); time.sleep(0.2)

t0 = time.time()
last_count = None; last_hb = -1.0
frames = anomalies = 0
zmin, zmax = 99.0, -99.0
while time.time() - t0 < DUR:
    if c.get_frame() != Result.Success:
        time.sleep(0.005); continue
    frames += 1
    n = c.get_unlabeled_marker_count()
    pts = []
    for j in range(n):
        t = c.get_unlabeled_marker_global_translation(j)
        if t is not None:
            pts.append(tuple(round(v / 1000.0, 3) for v in t[:3]))
    el = time.time() - t0
    if pts:
        z = pts[0][2]; zmin, zmax = min(zmin, z), max(zmax, z)
    if n != 1:
        anomalies += 1
        if anomalies <= 25:
            print(f"[{el:6.2f}s] *** ANOMALY count={n} pts={pts}",
                  flush=True)
        elif anomalies == 26:
            print(f"[{el:6.2f}s] *** (further anomalies suppressed)",
                  flush=True)
        last_count = n
    elif n != last_count:
        print(f"[{el:6.2f}s] count={n} pts={pts}", flush=True)
        last_count = n
    elif el - last_hb >= 1.0:
        x, y, zz = pts[0]
        print(f"[{el:6.2f}s] ok  x={x:+.2f} y={y:+.2f} z={zz:.2f}",
              flush=True)
        last_hb = el
print(f"--- done: {frames} frames, {anomalies} anomalies, "
      f"z range [{zmin:.3f}, {zmax:.3f}] m over {DUR}s ---", flush=True)
c.disconnect()
```

### 5.3 Vicon probe — `/tmp/vicon_probe.py`

Quick one-shot read of unlabeled markers — for confirming the drone is
in the volume and getting its `(x, y, z)` before launch.

```python
# /tmp/vicon_probe.py
#!/usr/bin/env python3
"""Probe Vicon unlabeled markers — report count + positions over N frames."""
import sys, time
from pyvicon_datastream import PyViconDatastream, Result

HOST = "192.168.0.62:801"
N = int(sys.argv[1]) if len(sys.argv) > 1 else 20

c = PyViconDatastream()
if c.connect(HOST) != Result.Success:
    print("connect failed"); sys.exit(1)
c.enable_unlabeled_marker_data(); time.sleep(0.3)

for i in range(N):
    if c.get_frame() != Result.Success:
        print(f"frame {i}: no frame"); time.sleep(0.05); continue
    nu = c.get_unlabeled_marker_count()
    pts = []
    for j in range(nu):
        t = c.get_unlabeled_marker_global_translation(j)
        if t is not None:
            pts.append(tuple(round(v / 1000.0, 3) for v in t[:3]))
    print(f"frame {i}: unlabeled={nu} pts={pts}")
    time.sleep(0.05)

c.disconnect()
```

### 5.4 Symptom → cause table

| Symptom | Likely cause | Fix |
|---|---|---|
| `Too many packets lost` / `is disconnected` | channel-80 WiFi interference | move to ch100 (§3.2) |
| Drone drifts on takeoff with `/cfN/odom` still matching `/poses` | wrong initial yaw | place at yaw≈0 (§3.1) |
| `/cfN/odom` ramps to tens of meters in a sawtooth | EKF starved (no extpos sustained) | look upstream — radio drop OR mocap loss; flight_logger tells you which |
| `all dynamic check failed` flood in mocap log | **usually a consequence** of a fast-moving marker (drone drifting), not the trigger | look upstream; don't blame mocap robustness |
| Drone won't arm after a crash | supervisor LOCKED (tumble state) | **power-cycle the drone** |
| RViz fails with `BadValue` GLX error | NVIDIA driver updated without reboot — kernel module / userspace mismatch | reboot the host |
| `Crazyradio not found` on launch | dongle re-seat needed / transient | retry the launch once; if persistent, different USB port |
| At-rest `/poses ≈ /cfN/odom` (gate check passes) but drone fails in flight | the at-rest gate proves *nothing* — EKF just sits at init position | use the move-test (slide drone by hand, check `/cfN/odom` follows) or fly to actually validate |

### 5.5 Wrong turns we made — DON'T repeat

This saga ate ~3 days of debugging. The wrong turns are documented so
they don't repeat:

- **Blaming the battery** for `Too many packets lost` — the battery was
  at 3.8 V (not critically low); the cause was channel-80 interference.
- **Declaring cf4 "physically damaged"** after wall crashes — cf4 flew
  clean after yaw≈0 + ch100; same bug, not damage.
- **Pivoting to "single-marker is fundamentally broken / need rigid
  body"** before isolating the yaw issue — cf3 was always flying clean
  on single-marker, which alone disproves "fundamentally broken."
- **Chasing firmware-version mismatch and re-flashing** before
  instrumenting `/poses` vs `/cfN/odom`.
- **Trusting the at-rest gate check** (`/poses ≈ /cfN/odom` while
  static) as proof that extpos works — it doesn't. The EKF sits at
  init; only motion reveals whether extpos is flowing.

**Meta-rule learned: instrument the data BEFORE blaming hardware.** The
instrumented-logger pattern (§5.1) is what cracked it; reach for it
*first* on any new HW symptom.

## 7. Per-drone verification log

Chronological record of verification / test runs per drone. **Add an
entry every time the procedure (or any per-drone test — envelope sweep,
post-repair check, simultaneous-flight participation, etc.) is run on
any drone.** Format:

```
- YYYY-MM-DD: <what was done> (<context>). <Result>. <Notes>.
```

Keep entries concise (1–2 lines) but include enough to be useful weeks
later. If a drone has an open concern (suspected damage, marker
re-fixed, battery degraded, marginal motor, etc.), note it.

**When a drone misbehaves, consult its entry below first** —
institutional fleet state lives here.

### cf1

- 2026-05-23: takeoff-hover-land clean (yaw≈0 + ch100, fresh battery,
  marker on); `/poses` and `/cf1/odom` matched within ~3 mm
  throughout hover at z = 0.5 m; clean descent. **Verified.**
  - *Dogfood note:* this run also surfaced the Vicon-monitor caveat
    documented in §5.2 — Vicon has rigid bodies defined, so the
    monitor saw only a stray reflector (not cf1). The flight_logger
    was the authoritative signal. Procedure updated to note this.
- 2026-05-23: envelope Pass 1 @ 1.0 m/s clean (cross pattern 1.9 m
  radius, full 4-arm: +X 1.94 / −X −2.02 / +Y 1.95 / −Y −1.98; 0
  anomalies, 0 link drops). **Verified to 1.0 m/s.**
- 2026-05-23: envelope Pass 2 @ 1.5 m/s, 1.85 m radius — **CLEAN**
  (cross pattern: +X 1.89 ✓, −X −1.96 ✓, +Y/−Y reached but monitor
  expired before logging — HLC land succeeded so they completed; 6
  brief phantom-count anomalies at t=74 that recovered; 0 link drops
  during flight). **Verified to 1.5 m/s.** HLC-land at end of pattern
  worked cleanly — no supervisor lock.
  - *Procedural lesson learned during this run:* the streamer-descent-
    to-0.10 → disarm-drop pattern (used in cf1 Pass 1, cf2 Pass 1, cf4
    earlier) triggers the firmware supervisor's tumble lock — the
    ~10 cm drop reads as a tumble; arm/takeoff then silently refuses
    until a full battery power-cycle clears it. cf1's first attempt at
    Pass 2 hit exactly this. **Workaround for future envelope passes:
    end the cross pattern with `kill streamer → immediate HLC land`**
    (HLC takes over within the firmware's ~0.5 s commander-watchdog,
    cleanly descends to ~5 cm, then disarm — no drop, no tumble flag).
    This pattern worked on Pass 2's second attempt.
  - *False alarm during diagnosis:* bringup log showed `Error no
    LogEntry to handle id=1` (firmware 2026.04 broken-protocol
    signature), but cf1's firmware was confirmed 2025.12 — the error
    is harmless on this firmware and not predictive of failure. The
    actual cause of the "won't take off" was the supervisor lock, not
    firmware.

### cf2

- 2026-05-21: takeoff-hover-land clean (yaw≈0 + ch100); EKF tracked
  `/poses` within mm. **Verified.**
- 2026-05-21: envelope Pass 1 @ 1.0 m/s clean (cross pattern 1.9 m
  radius, hover + thermal mapping live). **Verified to 1.0 m/s.**
- 2026-06-29: takeoff-hover-land clean (yaw≈0 + ch100, single-drone
  test on a freshly-reprovisioned box). z=0.5 m takeoff; `/poses` and
  `/cf2/odom` both peaked at 0.578 m and tracked together throughout;
  held (x,y) within a few cm of init (0.02, −0.282); monotonic descent;
  0 mocap drops, 0 link drops. **Verified.**
  - *Box reprovision note:* this run started from a bare box — udev
    rule `99-bitcraze.rules`, `pyvicon_datastream`, and the apt pkg
    `ros-jazzy-motion-capture-tracking` (mocap **node**, distinct from
    the apt-present `-interfaces`) were all missing and had to be
    reinstalled before bringup. Wired `enp12s0`→Vicon switch had to be
    re-cabled (box was WiFi-only). Worth a preflight check after any
    reimage.
  - *vicon_monitor noise:* monitor logged 476 raw-marker anomalies
    (count≠1) but flight_logger showed 0 `/poses` drops — the §5.2
    "trust flight_logger over monitor" case again.

### cf3

- 2026-05-20: envelope passes 1–4 (0.5 / 1.0 / 1.2 / 1.5 m/s) clean —
  **max verified speed: 1.5 m/s** (Pass 5 @ 2.0 m/s deferred when
  batteries needed recharge).
- 2026-05-21: takeoff-hover-land clean (post channel-100 change).
  **Verified.**

### cf4

- 2026-05-20: 4 clean takeoff-hover-land cycles (escalation test after
  marker re-fix). Single-marker mocap held throughout.
- 2026-05-20: −Y wall crash during envelope Pass 1 cross pattern at
  **2 m radius** (geometry overshoot — wall is at ~y=−2.3 m; later
  runs used **1.9 m radius**). NOT physical damage; misdiagnosed as
  such at the time.
- 2026-05-21: takeoff-hover-land clean (yaw≈0 + ch100). **Confirmed
  NOT damaged — prior "physical damage" diagnosis revised.** Verified.

### (template — copy this block for any new drone, e.g. cf5)

```
### cfN
- YYYY-MM-DD: onboarded — cfclient firmware 2025.12 / channel 100
  set, marker attached. First verification: <result>.
```

## 8. How this connects

This doc is the single source of truth for per-drone HW verification.
It's invoked:
- **Per-drone in multi-drone bringup** — see
  [docs/multi_drone_hw_handoff.md](multi_drone_hw_handoff.md) §3.
- **Standalone for single-drone debugging** — when a drone misbehaves,
  re-run this procedure.
- **For new drones joining the fleet** — same procedure, same
  preconditions.

| What | Where |
|---|---|
| Multi-drone phase (1→4 + thermal) | [docs/multi_drone_hw_handoff.md](multi_drone_hw_handoff.md) |
| Workspace infra + sim/HW invariants + history | [CRAZYSIM_MIGRATION.md §3](../CRAZYSIM_MIGRATION.md) |
| Thermal mapping pipeline | [docs/thermal_mapping_demo.md](thermal_mapping_demo.md), [docs/thermal_mapping_hw_handoff.md](thermal_mapping_hw_handoff.md) |
| Federated coverage / fed_dcsa | [docs/federated_coverage.md](federated_coverage.md) |
| Memories | `single-marker-yaw-zero-placement`, `hw-radio-link-intermittent` |
