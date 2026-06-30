# Hardware test runbook — CBF head-on avoidance (first real flight)

Step-by-step procedure for taking the **CBF head-on avoidance** (validated in sim, Tests 1–5,
[sim_validation_log.md](sim_validation_log.md)) onto **real Crazyflies in the Vicon volume**.
This is the document to follow **at the bench / in the flight space** — commands, "go / no-go"
criteria, and safety rules in order.

> **Scope & honesty rules.** Written from code + config only (the author cannot run real
> hardware). Every claim cites `file:line`. Anything the repo does not pin down is marked
> **⚠️ NO REPO BASIS — confirm on HW** instead of guessed. The CBF *control logic* is identical
> sim↔HW (one control plane, [CRAZYSIM_MIGRATION.md:109-149](../CRAZYSIM_MIGRATION.md)); what
> changes is the backend (radio + Vicon) and the *quality* of the data the CBF runs on.
>
> Infrastructure procedures (Crazyradio, mocap, arm/firmware) are **not duplicated** here —
> this doc links to [CRAZYSIM_MIGRATION.md §3](../CRAZYSIM_MIGRATION.md) (hardware bringup) and
> [hw_localization_path.md](hw_localization_path.md) (the data path). Read those first.

---

## 0. The sim→HW mapping at a glance

Our sim head-on test was a 3-layer stack. What each layer becomes on hardware:

| Sim layer | Sim command | Hardware equivalent |
|---|---|---|
| **(a) Gazebo + SITL** | `scripts/sitl_2drone_headon.sh` (spawns `gz sim -s` + 2 `cf2-sitl` docker containers) | **Deleted.** Real drones replace the simulator. Instead you need: **physical drones placed in the Vicon volume**, **Vicon Tracker running**, **Crazyradio dongle plugged in**. No Gazebo, no docker. |
| **(b) crazyswarm2 server** | `ros2 launch crazyflie launch.py backend:=cflib crazyflies_yaml_file:=…sitl_2drone_nobvc.yaml mocap:=False gui:=false` | **Same node, swapped config**: `crazyflies_yaml_file:=…_hw.yaml` (radio URIs), `mocap:=True` (Vicon backend on). Only the YAML + `mocap` flag differ ([CRAZYSIM_MIGRATION.md:204-222](../CRAZYSIM_MIGRATION.md)). |
| **(c) CBF driver** | `python3 scripts/cbf_headon_test.py …` | **Same script, different flags.** The control plane (subscribes `/cfN/odom`, publishes `/cfN/cmd_position`, calls takeoff) is transport-agnostic and works as-is. But it has **sim assumptions to undo** (no arm step; the `PeerFeed` degradation layer double-counts on HW) — see §1c + §6. |

**Why (a) just disappears:** the SITL containers + Gazebo plugin exist only to *fake* what a real
drone + Vicon provide — firmware on real silicon and ground-truth pose
([hw_localization_path.md:60-71](hw_localization_path.md), [:137-153](hw_localization_path.md)).
On HW the firmware runs on the STM32 and pose comes from Vicon→EKF. Nothing replaces the launcher;
you simply power on drones and start Vicon.

---

## 1. What the CBF driver assumes, and what survives on HW

`scripts/cbf_headon_test.py` analysed line-by-line for sim-only coupling:

### (a) What is transport-agnostic (works on HW unchanged)
- **Odom subscriptions** `/cf1/odom`, `/cf2/odom` ([cbf_headon_test.py:257-258](../scripts/cbf_headon_test.py#L257)) — published by the same server on HW (`firmware_logging.odom`, [crazyflies_hw.yaml:59-64](../config/crazyflies_hw.yaml#L59)). ✅
- **Setpoint publish** `/cfN/cmd_position` ([:259-262](../scripts/cbf_headon_test.py#L259)) — identical service/topic sim↔HW ([CRAZYSIM_MIGRATION.md:151-165](../CRAZYSIM_MIGRATION.md)). ✅
- **Goal geometry is derived at runtime** from the drones' *actual* start positions ([:539-544](../scripts/cbf_headon_test.py#L539): cf1 goal = cf2 start, cf2 goal = cf1 start + `--lat`). So it adapts to wherever the drones really are — **no hard-coded (−1,0)/(+1,0)**. ✅ (But you must physically place them in a genuine head-on, see §2.)
- **CBF / bias / QP math** ([cbf_filter](../scripts/cbf_headon_test.py#L69), [right_bias](../scripts/cbf_headon_test.py#L105)) — pure math, no sim dependency. ✅

### (b) What has NO sim/HW coupling but you must be aware of
- The driver **does not use UDP, docker, or container ports**. (Those live only in `sitl_2drone_headon.sh`.) The driver talks ROS topics/services, which are backend-agnostic.

### (c) Sim assumptions that MUST be undone for HW
These are the "things to change" — **listed, not changed** (no code edits in this task):

1. **No arm step.** [takeoff_both()](../scripts/cbf_headon_test.py#L360) calls only `/cfN/takeoff` —
   it never calls `/cfN/arm`. On HW, firmware 2024+ **requires `/cfN/arm` (arm:true) BEFORE takeoff**
   ([CRAZYSIM_MIGRATION.md:403](../CRAZYSIM_MIGRATION.md)); the server exposes it at
   [crazyflie_server.py:315](../src/crazyswarm2/crazyflie/scripts/crazyflie_server.py#L315) (`/cfN/arm`)
   and [:380](../src/crazyswarm2/crazyflie/scripts/crazyflie_server.py#L380) (`all/arm`). In sim arming
   is implicit. **Fix for HW (choose one): arm manually from a 4th terminal right before launching the
   driver** (recommended, zero code change — see §3 step 6), or add an arm call to the driver.
2. **The `PeerFeed` degradation layer double-counts on HW.** `PeerFeed`
   ([cbf_headon_test.py:140-231](../scripts/cbf_headon_test.py#L140)) *artificially* degrades cf2's
   position (rate / delay / noise) to *simulate* a rough HW feed on top of perfect sim state. On HW the
   feed is **already real** — `/cf2/odom` carries the real Vicon+EKF estimate. Running the default
   degradation on top would degrade twice. **Fix: see §6** (peer-feed flags for HW). This is the single
   most important flag change.
3. **`crazyflies_hw.yaml` is a 4-drone quadrant config, not a 2-drone head-on.** It enables cf1–cf4
   ([crazyflies_hw.yaml:14-33](../config/crazyflies_hw.yaml#L14)) at placeholder quadrant positions
   ([:8-10](../config/crazyflies_hw.yaml#L8)), not a head-on layout. The driver only commands cf1/cf2,
   so cf3/cf4 would connect and arm but sit on the ground. **Fix: see §2** (make/adjust a 2-drone HW yaml).

---

## 2. Pre-flight preparation checklist (do at the bench, before powering rotors)

| # | Item | How / where | Go criterion |
|---|---|---|---|
| 2.1 | **Vicon up & reachable** | Vicon Tracker 3.10 on `192.168.0.62:801` ([motion_capture.yaml:4-6](../config/motion_capture.yaml#L4)). | `nc -zv 192.168.0.62 801` succeeds (the comment at [motion_capture.yaml:5](../config/motion_capture.yaml#L5) records this check). |
| 2.2 | **Rigid bodies / markers** | `librigidbodytracker` + `default_single_marker` = 1 retro ball per drone, centered on top ([crazyflies_hw.yaml:37-40](../config/crazyflies_hw.yaml#L37), [motion_capture.yaml:25-28](../config/motion_capture.yaml#L25)). Rigid bodies auto-generated by launch ([motion_capture.yaml:59-60](../config/motion_capture.yaml#L59)). | Each drone shows as a tracked single marker in Vicon Tracker with no ghost/extra markers in the volume. |
| 2.3 | **`initial_position` matches physical spawn** | Drones must be within **~50 cm** of their yaml `initial_position` for librigidbodytracker to assign IDs correctly ([crazyflies_hw.yaml:7-10](../config/crazyflies_hw.yaml#L7)). Current values are quadrant placeholders re-probed 2026-06-04 ([:17-32](../config/crazyflies_hw.yaml#L17)) — **not a head-on layout.** | For a head-on: physically place cf1 and cf2 **facing each other across the volume** (e.g. cf1 at one side, cf2 opposite, same y, clear runway between), and **update their `initial_position` to those real coordinates.** |
| 2.4 | **2-drone config** | No 2-drone HW yaml exists (only `crazyflies_hw.yaml`, 4 drones). ⚠️ NO REPO BASIS for a 2-drone HW config — **create one** (copy `crazyflies_hw.yaml`, set `enabled: false` for cf3/cf4) **or** just leave cf3/cf4 powered off (the driver only commands cf1/cf2). | Only cf1, cf2 enabled/powered; their `initial_position` set to the head-on placement from 2.3. |
| 2.5 | **Radio / URIs** | Crazyradio 2.0 dongle, udev rule at `/etc/udev/rules.d/99-bitcraze.rules` ([CRAZYSIM_MIGRATION.md:469](../CRAZYSIM_MIGRATION.md)). cf1/cf2 URIs `radio://0/100/2M/E7E7E7E7E1`/`…E2` ([crazyflies_hw.yaml:16,21](../config/crazyflies_hw.yaml#L16)). **Channel 100** (not 80 — WiFi interference, see memory `hw-radio-link-intermittent`). | `cfclient` connects to each drone on its URI; battery readable. |
| 2.6 | **Firmware version** | **2025.12, NOT 2026.04** — 2026.04 silently breaks EKF mocap fusion ([CRAZYSIM_MIGRATION.md:406-423](../CRAZYSIM_MIGRATION.md) Gotcha A). | cfclient → Service tool shows 2025.12 on both drones. |
| 2.7 | **Estimator / controller** | Kalman estimator 2 + PID controller 1, mocap trust `extPosStdDev:0.01` ([crazyflies_hw.yaml:53-54](../config/crazyflies_hw.yaml#L53)). | These are already in the yaml; nothing to do but confirm. |
| 2.8 | **Battery** | `voltage_warning:3.8 / voltage_critical:3.7` ([crazyflies_hw.yaml:42](../config/crazyflies_hw.yaml#L42)); supervisor refuses to arm < 3.7 V ([CRAZYSIM_MIGRATION.md:475](../CRAZYSIM_MIGRATION.md)). | Both packs fresh (> ~3.9 V resting). A weak pack = more PWM for lift ([CRAZYSIM_MIGRATION.md:506-508](../CRAZYSIM_MIGRATION.md)). |
| 2.9 | **Flight space** | Clear runway between the two head-on positions long enough for the swap + lateral sidestep (sim sidestep reached \|y\|≈0.6 m, [sim_validation_log.md Test 5](sim_validation_log.md); allow ≥1 m clearance each side). | No obstacles/people in the swap corridor or the sidestep margins. |
| 2.10 | **Kill switch ready** | `/cfN/emergency` + `/all/emergency` (Empty srv → firmware `send_emergency_stop()`) at [crazyflie_server.py:311,388,938-943](../src/crazyswarm2/crazyflie/scripts/crazyflie_server.py#L938). **This cuts motors instantly (drone drops).** | A terminal pre-typed with `ros2 service call /all/emergency std_srvs/srv/Empty "{}"` ready to hit Enter (see §5). |
| 2.11 | **★ radio↔Vicon mapping verified** | The single most dangerous failure for ≥2 drones. See **§2A** — run the move-test **every** multi-drone flight. | Moving the drone you call `cfN` moves **`/cfN/odom`** and no other. If the *wrong* odom moves → **STOP, do not arm** (mapping swapped). |

> **Kill-switch caveat.** `/all/emergency` is a **motor cut → free fall**, not a controlled land.
> For a soft stop use `/all/land`. Use emergency only when a crash is otherwise imminent.
> ⚠️ Whether a physical/joystick e-stop is also wired is **NO REPO BASIS** — the software service
> above is the kill switch this stack provides.

---

## 2A. ★ MANDATORY pre-flight gate — radio↔Vicon mapping (run EVERY multi-drone flight)

**This is not a one-time setup check — re-run it before every multi-drone arm.** Drones get
re-placed, re-imaged, or swapped between sessions; the mapping can be correct one flight and wrong
the next. Verified live 2026-06-30 (it caught a real swap that would have crashed both drones).

**The trap.** A drone's Vicon **rigid-body label is derived purely from `initial_position`**, not
from its radio identity. The launch file builds one rigid body per enabled robot, seeded at that
robot's `initial_position` ([launch.py:48-57](../src/crazyswarm2/crazyflie/launch/launch.py#L48)),
and `librigidbodytracker` then assigns whichever **marker is nearest each seed** to that label. So
if you physically place the drone whose radio is `…E2` where the `cf4` seed sits, Vicon will label
it **cf4** — while the server still sends `/cf4`'s extpos to it and the driver reads `/cf4/odom`
for it. The radio (`…E2`) and the Vicon label (`cf4`) now point at **different physical drones**.

**Why it crashes silently.** The server feeds extpos(Vicon-`cfN`) to radio-`cfN`. If those are
different drones, the EKF of radio-`cfN` is told it is where the *other* drone is; `/cfN/odom`
then reads a plausible, stable, **wrong** position. Nothing looks off on the ground — until takeoff,
when the drone flies to "correct" a position error that is really a label swap → immediate runaway.

**The test (definitive, ~30 s).** With the stack up and both drones tracked & stable on the ground:
1. Stream both odoms (e.g. a small subscriber, or `ros2 topic echo /cfN/odom`).
2. By hand, gently slide **one** drone you can identify (sticker/URI) ~30 cm and hold it.
3. **GO:** only **that drone's** `/cfN/odom` changes; the other stays put. Mapping is correct.
4. **NO-GO:** a *different* `cfN`'s odom moves → **STOP, do not arm.** The labels are swapped vs
   radio identity. Fix by setting each robot's `initial_position` to the **actual** physical
   position of *its radio drone* (so `cf2`'s seed sits where the `…E2` drone really is), relaunch,
   and re-run this test until it passes.

**If you can't tell which physical drone is which radio ID:** scan to confirm which URIs are live
(`cflib.crtp.scan_interfaces(addr)` per address), then bind radio→physical by actuating one drone
over its URI. Note: **motors are arming-gated** (firmware 2024+), so a `motorPowerSet` blip does
nothing until you `send_arming_request(True)` first; and these frames have **no LED-ring / buzzer
deck** (`deck.bcLedRing=0`, `deck.bcBuzzer=0`), so an armed single-motor blip is the available
physical identifier. Easiest is still the sticker + move-test above.

---

## 3. Flight execution order (step by step)

Four terminals. Run `source $CS2_WS/env.sh` in each first. **Do not skip the go/no-go checks.**

### Step 1 — (Vicon already running from §2.1)
No launcher to start (unlike sim — the Gazebo step is deleted, §0). Confirm Vicon Tracker is
streaming both rigid bodies.

### Step 2 — crazyswarm2 server (the (b) layer, HW config)
```bash
# Terminal 1
source $CS2_WS/env.sh
ros2 launch crazyflie launch.py \
    backend:=cflib \
    crazyflies_yaml_file:=$CS2_WS/config/crazyflies_hw.yaml \
    motion_capture_yaml_file:=$CS2_WS/config/motion_capture.yaml \
    mocap:=True \
    gui:=false
```
- **★ `motion_capture_yaml_file:=$CS2_WS/config/motion_capture.yaml` is MANDATORY.** Omit it and the
  launch falls back to the **installed package default** (`install/crazyflie/share/crazyflie/config/motion_capture.yaml`),
  whose `hostname` is the upstream placeholder **`141.23.110.143`** (a TU-Berlin address), **not** the lab
  Vicon `192.168.0.62`. The `motion_capture_tracking` node then connects to the wrong host, gets **zero
  frames** (`/poses` publisher count 0 — `nc :801` still "succeeds" because the socket is open), and with
  no extpos the **EKFs diverge** (odom reads tens of metres). Verified live 2026-06-30 — this omission
  cost a full debug cycle and was the upstream trigger of the Stage A crash sequence (§9). **Go-criterion:
  after launch, `ros2 topic hz /poses` reads ~400 Hz; if it's silent, you dropped this arg.**
- **`mocap:=True`** (opposite of sim's `mocap:=False`) — turns on the Vicon `motion_capture_tracking`
  node so `/poses`→extpos fusion runs ([hw_localization_path.md:45-52](hw_localization_path.md),
  [:73-75](hw_localization_path.md)). In sim it was False because there is no mocap
  ([CRAZYSIM_MIGRATION.md:772-779](../CRAZYSIM_MIGRATION.md)).
- Use the 2-drone-adjusted yaml from §2.4 if you made one; otherwise `crazyflies_hw.yaml` with
  cf3/cf4 powered off.
- **⚠️ NO REPO BASIS** that `peer_broadcast_hz:10` ([crazyflies_hw.yaml:50](../config/crazyflies_hw.yaml#L50))
  must be off for the Python CBF. The Python CBF reads `/cf2/odom` directly, **not** the firmware peer
  broadcast, so the broadcast is irrelevant to it — but `colAv.enable:1` ([:58](../config/crazyflies_hw.yaml#L58))
  means the **firmware BVC is also active** as a redundant safety net. That is *fine* (two layers), but be
  aware the firmware BVC may also bend paths; if you want the Python CBF to be the *sole* avoider for a
  clean comparison, set `peer_broadcast_hz:0` (firmware BVC goes no-op, as in the sim `_nobvc.yaml`).

**GO / NO-GO #1 — full connection + pose:**
- Server log shows **both** drones "fully connected" (the `fully_connected` callback,
  [crazyflie_server.py:165](../src/crazyswarm2/crazyflie/scripts/crazyflie_server.py#L165)).
- `ros2 topic echo /cf1/odom --once` and `/cf2/odom` return positions that **match the drones' real
  positions in the volume** (this confirms Vicon→EKF fusion — the exact check used historically,
  [CRAZYSIM_MIGRATION.md:471](../CRAZYSIM_MIGRATION.md)). If odom reads ~(0,0,0) while the drone is
  elsewhere, EKF is not fusing mocap → **STOP** (Gotcha A firmware, or marker not tracked).
- **EKF warm-up:** wait ~1–2 s after first pose for the estimate to converge
  ([CRAZYSIM_MIGRATION.md:502-505](../CRAZYSIM_MIGRATION.md)) before trusting odom.

### Step 3 — (optional) measure the 3 unknowns FIRST
Before any avoidance flight, do the measurement hover in §4 — you need the real delay to set `--R` (§6).
This can be a separate, earlier flight.

### Step 4 — arm (the missing step the driver does NOT do)
```bash
# Terminal 2 — REQUIRED on HW, not done by the driver (see §1c-1)
ros2 service call /cf1/arm crazyflie_interfaces/srv/Arm "{arm: true}"
ros2 service call /cf2/arm crazyflie_interfaces/srv/Arm "{arm: true}"
```
**GO / NO-GO #2:** both calls return success (supervisor allows arm ⇒ battery OK, [:475](../CRAZYSIM_MIGRATION.md)).

### Step 5 — run the CBF driver (the (c) layer)
The driver takes off both drones, streams the CBF swap, then lands ([main()](../scripts/cbf_headon_test.py#L549-561)).
**Use the conservative first-flight flags from §7.** Example conservative invocation:
```bash
# Terminal 2 (after arm) — CONSERVATIVE first flight (see §7 for the rationale of each value)
python3 $CS2_WS/scripts/cbf_headon_test.py \
    --label hw_first \
    --R 0.7 --vmax 0.15 --obs-speed 0.0 \
    --peer-hz 20 --peer-delay 0.0 --peer-noise-std 0.0 \
    --lat 0.3 \
    --out /tmp/cbf/hw_first.csv
```
**GO / NO-GO #3 (during the run, watch live):**
- Both drones reach hover at `z=1.0` and **hold** during the 3 s settle phase ([:299-305](../scripts/cbf_headon_test.py#L299)) before any motion. If either drifts badly → emergency-stop (§2.10) and abort.
- During the run, cf1 should **veer to its right (−y)** as it nears cf2 (the right-hand bias signature, [right_bias](../scripts/cbf_headon_test.py#L105); matches sim Test 4). If cf1 drives **straight at** cf2 → emergency-stop immediately.
- Watch true separation; the summary prints `MIN horiz separation` vs `R` at the end ([:583-584](../scripts/cbf_headon_test.py#L583)).

### Step 6 — land / teardown
The driver lands automatically ([land_both](../scripts/cbf_headon_test.py#L372), [:559-561](../scripts/cbf_headon_test.py#L559)).
After a land, **re-arm is required before another flight** (and a landed drone won't re-takeoff until
re-armed — same lesson as sim, [CRAZYSIM_MIGRATION.md:840-842](../CRAZYSIM_MIGRATION.md)). For a fresh
run, repeat from Step 4.

---

## 4. Measuring the 3 unknowns (first-flight logging)

[hw_localization_path.md:157-165](hw_localization_path.md) lists three quantities the repo does **not**
pin down and says to measure them on HW. How to get each — **using only existing topics**:

### 4.0 Measured results (2026-06-29, cf2, real HW) — ACTUAL MEASUREMENTS, not assumptions

The three unknowns below were **measured on real hardware** (cf2 single drone, Vicon volume,
PID controller / Kalman estimator, `extPosStdDev:0.01`). **Method:** a recorder node logged
`/poses` + `/cf2/odom` on a **single receive clock** (deliberately *not* header stamps — odom is
re-stamped, see §4.3); a **30 s static-hover bag** + a **36 s motion bag** (±0.3 m `go_to` steps at
0.15 m/s); delay via offline cross-correlation (position **and** velocity).

| Unknown (§) | **Measured** | Notes |
|---|---|---|
| Effective rate (§4.1) | `/poses` **397 Hz** (p95 dt 2.7 ms, no dropouts); `/cf2/odom` **20.0 Hz** (p95 dt 52 ms) | odom rate = what the CBF actually consumes. Confirms the 20 Hz config; `/poses` source rate is ~400 Hz (the 100 Hz QoS hint was *not* the source rate). |
| Horizontal position σ (§4.2) | **0.8–0.9 cm/axis** on `/cf2/odom` (x 8.1 mm, y 8.7 mm); z σ ~2.9 cm | Airborne hover, so this is station-keeping + sensor noise = an **upper bound** on pure sensor σ. **Below Test 5's 1.5 cm "negligible" threshold → noise is not the limiter**, as predicted. |
| End-to-end delay Vicon→`/cf2/odom` (§4.3) | **≲ 67 ms (1-hop)** — *not* finalized to a point value | Resolution-limited: the 0.15 m/s motion has a floor σ/slew = 10 mm / 150 mm·s⁻¹ ≈ **67 ms**; below that a real delay is invisible. Point estimate landed at 0–5 ms (pos & vel xcorr, corr 0.998–1.000). Physically bounded by the 20 Hz odom sampling (~25–50 ms) + minor pipeline. **Confirms the §4.3 scope note: the 1-hop `/cf2/odom` delay is ≪ Test 5's assumed 2-hop 100 ms.** |

**odom-follows-Vicon quality (motion bag):** horizontal tracking is **mm-accurate** — residual
RMS **x 2.2 mm, y 1.6 mm** (max ≤ 5.5 mm) after delay alignment, **no horizontal jumps / drops /
ghost-marker artifacts** (the axis the head-on CBF runs on). z showed ~**0.8 %** single-sample
~25 cm spikes (instant recovery, mid-hover, in pairs — likely the benign `Error no LogEntry id=1`
corrupted-log packet); **z-only, irrelevant to horizontal head-on avoidance.** 0 radio link drops.

**R_design impact (why the sharp-step re-fly was skipped — yak-shaving):** with a conservative
delay of 50 ms, `delay·v_rel` = 7.5 mm (Stage 1, v_rel 0.15) to 40 mm (full head-on, v_rel 0.8);
even at the **67 ms upper bound · v_rel 0.8 = 54 mm**. All **≪ R = 0.7 m** — so a precise delay
would not change the R margin. The measured numbers confirm R = 0.7 m is amply conservative and
that **neither noise nor delay is binding** for the first flights. (Pinning delay crisply would
need sharper steps, ~0.4–0.5 m/s, dropping the floor to ~20 ms — deferred as unnecessary.)

### 4.1 Vicon actual output rate
- The repo only has a **QoS deadline hint of 100 Hz** ([motion_capture.yaml:12-13](../config/motion_capture.yaml#L12)), which is *not* the source rate ([hw_localization_path.md:107](hw_localization_path.md)).
- **Measure:** `ros2 topic hz /poses` (the raw mocap `NamedPoseArray`, [hw_localization_path.md:19,47](hw_localization_path.md)).
- Also note the odom-log rate the CBF actually consumes: `ros2 topic hz /cf2/odom` — expect ~**20 Hz** ([crazyflies_hw.yaml:64](../config/crazyflies_hw.yaml#L64)).

### 4.2 Position noise σ
- **Measure:** place a drone **stationary**, record for ~30 s, compute **per-axis standard deviation**:
  ```bash
  ros2 topic echo /poses > /tmp/cbf/vicon_static.txt        # or: ros2 bag record /poses /cf2/odom
  ```
  then compute std(x), std(y), std(z) offline. The std of a *stationary* drone's reported position **is** σ.
- Cross-check against `/cf2/odom` (post-EKF) too — the CBF runs on odom, so its σ is what matters for `--peer-noise-std`-equivalent reasoning.
- Test 5 found σ≈1.5 cm was **negligible** for the barrier ([sim_validation_log.md Test 5](sim_validation_log.md), ablation D); so unless measured σ ≫ 1.5 cm, noise is not your problem — **delay is** (next).

### 4.3 End-to-end delay (the one that sets `--R`) — and the catch
Test 5's fix is `R_design = R_safety + delay · v_rel,max` ([sim_validation_log.md Test 5, "The fix that works (G)"](sim_validation_log.md)). You need **`delay`**.

- **⚠️ The easy method does NOT work.** `/cfN/odom` is **re-stamped with the server's publish time**, not
  the mocap capture time: `msg.header.stamp = self.get_clock().now().to_msg()` at
  [crazyflie_server.py:719](../src/crazyswarm2/crazyflie/scripts/crazyflie_server.py#L719). So
  `ros2 topic delay /cf2/odom` would read ≈0 and **mislead you** — it measures nothing about the real latency.
- **Method that works (step / cross-correlation test):** record **both** `/poses` (earliest, raw Vicon)
  and `/cf2/odom` (what the CBF sees) while you **sharply move a drone by hand** (a step):
  ```bash
  ros2 bag record /poses /cf2/odom        # then move the drone abruptly several times
  ```
  Offline, cross-correlate the two position time-series; the lag of peak correlation = the
  Vicon→EKF→odom-log→topic delay. That is the delay relevant to **this** Python CBF (it reads `/cf2/odom`).
- **Important scope note.** Test 5 modelled the *firmware peer-broadcast* **2-hop** delay
  (cf2 EKF → 20 Hz odom → server → 10 Hz re-broadcast → cf1 firmware,
  [hw_localization_path.md:124-132](hw_localization_path.md)). **This Python CBF does not use that path** —
  it reads `/cf2/odom` directly (1 hop: cf2 EKF → odom-log → server → ROS topic). So the delay you measure
  here is **smaller** than Test 5's assumed 2-hop 100 ms, and it is the *correct* one for `R_design` of the
  Python CBF. Measure it; don't reuse the 100 ms assumption blindly.
- The driver already logs everything else you need per tick (peer position, v_obs, separation,
  [cbf_headon_test.py:338-353](../scripts/cbf_headon_test.py#L338)); it does **not** log message header
  stamps, which is why the separate bag + cross-correlation is needed for delay.

**Then:** plug the measured delay and the cruise `v_rel,max` (≈ `vmax + obs_speed`) into
`R_design = R_safety + delay · v_rel,max` and set `--R` for subsequent flights.

---

## 5. First-flight safety design

Our data-path investigation flagged the worst-case coupling: **Vicon estimate degrades exactly when
drones are close** (ghost / wrong markers grabbed when drones pass close,
[motion_capture.yaml:53-57](../config/motion_capture.yaml#L53),
[hw_localization_path.md:113-114](hw_localization_path.md)). That is the moment the CBF matters most — so
the **first** HW flight must be conservative, not a full-speed head-on. Principles, in order:

1. **Slow.** Low `vmax` and a **static obstacle first** (`--obs-speed 0.0` → cf2 holds position,
   [_vdes](../scripts/cbf_headon_test.py#L271) returns ~0 → cf2 hovers). cf1 avoids a *stationary* cf2 —
   removes the moving-obstacle / v_obs-estimation difficulty entirely for flight #1. Low speed also shrinks
   the `delay·v_rel` margin you need (§4.3).
2. **Big R margin.** Start with `--R` well above the sim 0.5 m safety value and above the Test-5 `R_design`,
   because the close-range degradation is **unquantified** (⚠️ NO REPO BASIS). Sim min-sep sat just above R
   ([sim_validation_log.md Test 5](sim_validation_log.md)); give yourself extra on HW.
3. **Staged approach** (each stage only if the previous met its go criterion):
   - **Stage 0:** measurement hover, no encounter (§4) — get rate/σ/delay.
   - **Stage 1:** static obstacle (`--obs-speed 0.0`), big R, slow vmax, **`--lat` > 0** (geometric pass
     side + the bias, easiest case). cf1 must hold separation ≥ R and reach goal.
   - **Stage 2:** static obstacle, **`--lat 0.0`** (exact head-on; relies on the right-hand bias alone,
     the Test-4 deadlock cure [right_bias](../scripts/cbf_headon_test.py#L105)).
   - **Stage 3:** moving obstacle — raise `--obs-speed` gradually toward `--vmax`, recompute `--R` from the
     measured delay each time (§4.3). This is the real head-on; do it last.
4. **Kill switch armed** (§2.10) and a human ready to land (`/all/land`) or e-stop (`/all/emergency`).

### Recommended first-flight parameters
`cbf_headon_test.py` flags ([argparse :469-513](../scripts/cbf_headon_test.py#L469)) with conservative
starting values. **These are derived from sim defaults scaled down for a cautious first flight, not
HW-measured** — re-tune after §4.

| Flag | Sim default | **HW Stage 1 start** | Why |
|---|---|---|---|
| `--R` (safety radius) | 0.5 m ([:478](../scripts/cbf_headon_test.py#L478)) | **0.7 m** | Big margin vs unquantified close-range degradation; Test 5's G fix already needed 0.55 for just 100 ms assumed delay. |
| `--vmax` (ego cruise) | 0.4 m/s ([:480](../scripts/cbf_headon_test.py#L480)) | **0.15 m/s** | Slow = small `delay·v_rel`, more time for the filter, gentle if it fails. |
| `--obs-speed` | 0.4 m/s ([:481](../scripts/cbf_headon_test.py#L481)) | **0.0** (static) | Removes moving-obstacle + v_obs difficulty for flight #1 (§5.1). Raise only at Stage 3. |
| `--lat` | 0.2 ([:484](../scripts/cbf_headon_test.py#L484)) | **0.3** then **0.0** | Start with a defined geometric pass side (easiest), then go to exact head-on. |
| `--alpha` | 2.0 ([:479](../scripts/cbf_headon_test.py#L479)) | 2.0 (keep) | Raise for firmer/earlier braking if min-sep dips ([sim_validation_log.md Test 3](sim_validation_log.md)). |
| `--lookahead` | 0.6 s ([:483](../scripts/cbf_headon_test.py#L483)) | 0.6 s (keep) | Shrink if tracking lags under HW latency ([Test 3 note](sim_validation_log.md)). |
| `--bias-gain` | 0.3 m/s ([:488](../scripts/cbf_headon_test.py#L488)) | **~0.11 m/s** | Keep the sim ratio (≈0.75·vmax) at the lower vmax. ⚠️ ratio inferred from sim; confirm sidestep is clean on HW. |
| `--bias-margin` | 0.7 m ([:490](../scripts/cbf_headon_test.py#L490)) | 0.7 (keep) | Proximity ramp width; independent of speed. |

---

## 6. Peer-feed flags on HW (the most important change)

**The `PeerFeed` layer simulates HW degradation on top of sim's perfect state — on HW that degradation
is already real, so do not stack it.** ([cbf_headon_test.py:140-166](../scripts/cbf_headon_test.py#L140)
docstring; [hw_localization_path.md §3](hw_localization_path.md).)

- **Do NOT use the default degraded feed** (`--peer-hz 10 --peer-delay 0.1 --peer-noise-std 0.015`,
  [:500-505](../scripts/cbf_headon_test.py#L500)). Those *add* artificial delay/noise on top of the real
  `/cf2/odom` → double degradation.
- **`--ideal-peer`** makes the layer transparent ([:245-247](../scripts/cbf_headon_test.py#L245)) — but it
  forces the latch rate to **50 Hz** while `/cf2/odom` actually arrives at **~20 Hz**
  ([crazyflies_hw.yaml:64](../config/crazyflies_hw.yaml#L64)). The held value then doesn't change for ~2–3
  ticks and jumps on the 3rd, so the finite-difference `v_obs` ([:221-225](../scripts/cbf_headon_test.py#L221))
  gets jittery. ⚠️ Whether that jitter actually hurts on HW is **NO REPO BASIS** — Test 5 found jitter ≠
  penetration, but that was at the matched sim rate.
- **Recommended HW setting:** `--peer-hz 20 --peer-delay 0.0 --peer-noise-std 0.0` — match the latch rate to
  the **real** `/cf2/odom` rate so `v_obs` is finite-differenced over real position changes with a sensible
  `dt`, and add **no** artificial delay/noise (the real ones are already in the topic). This is the cleanest;
  **confirm against `--ideal-peer` on HW.**
- **Do NOT use `--proximity-degrade`** ([:508](../scripts/cbf_headon_test.py#L508)) — that is a sim-only
  *model* of the ghost-marker effect; on HW the real effect (if present) is already in the data.
- The **real** latency you removed from the artificial layer still exists in `/cf2/odom` — that is exactly
  why you measure it (§4.3) and compensate via `--R` / `--alpha`, **not** via v_obs filtering (Test 5 showed
  stronger LPF *backfires*, [sim_validation_log.md Test 5, run C](sim_validation_log.md)).

---

## 7. Quick reference — the conservative first command

After §2 (prep) and §3 steps 2 (server) + 4 (arm), Stage 1:
```bash
python3 $CS2_WS/scripts/cbf_headon_test.py \
    --label hw_stage1 \
    --R 0.7 --vmax 0.15 --obs-speed 0.0 \
    --lat 0.3 \
    --bias-gain 0.11 \
    --peer-hz 20 --peer-delay 0.0 --peer-noise-std 0.0 \
    --out /tmp/cbf/hw_stage1.csv
```
Emergency stop in a spare terminal, ready to fire:
```bash
ros2 service call /all/emergency std_srvs/srv/Empty "{}"
```
Then progress through the §5 stages, re-computing `--R` from the §4.3 measured delay before the
moving-obstacle stage.

---

## 8. 2-drone simultaneous hover — infra validation (2026-06-30, real HW) ✅ PASSED

The infra step **between** single-drone (§4, done 2026-06-29) and any avoidance flight. Goal is
**not** avoidance — only to confirm two drones can be told apart and held aloft at once before
trusting the CBF. Live drones this session: **cf2 (`…E2`) + cf4 (`…E4`)**; `cf1`/`cf3` did not
respond to a radio scan.

**Config:** dedicated [config/crazyflies_hw_2drone.yaml](../config/crazyflies_hw_2drone.yaml) —
only cf2+cf4 enabled, `initial_position` probed from live `/poses`, **`colAv.enable:0` and
`peer_broadcast_hz:0`** (hover-only, no avoidance). The shared 4-drone `crazyflies_hw.yaml`
(Raman's fed_dcsa demo) is **left untouched**.

**Procedure:**
1. Launch the stack with the 2-drone yaml, `mocap:=True backend:=cflib` (as §3 step 2).
2. Confirm both `fully connected`, both `/cfN/odom` at 20 Hz with **no dropouts** (`ros2 topic hz`
   — not `echo --once`, which races and shows false gaps), and each odom matching its `/poses` body.
3. **Run the §2A mapping gate (move-test).** ← this is where the swap was caught; do not skip.
4. Place the two drones **≥1.5 m apart** (removes the close-range ghost-marker variable for hover).
5. Arm both → `takeoff 0.5 m` **one drone first**, confirm stable hover ~5 s → takeoff the second →
   both hover 0.5 m ~10 s → `/all/land` → disarm both. Drive with odom monitored live; `/all/land`
   on any divergence, `/all/emergency` (human) as backstop.

**Result (PASSED):** sequential takeoff clean; both held 0.5 m (z 0.50 ±0.02); **no mutual
interference** (second takeoff did not perturb the first, ~1.58 m apart); **0 radio dropouts**
with both flying on the single dongle (odom age <50 ms throughout); smooth simultaneous land.

### 8.1 Gotchas observed (real HW)
- **★ radio↔Vicon mapping was SWAPPED** — the headline finding. Physical placement was opposite the
  old quadrant seeds, so Vicon labeled each drone as the *other* one. Caught by §2A, fixed by
  swapping the `initial_position` seeds to each radio drone's real position. **See §2A — this is now
  a mandatory every-flight gate.**
- **Single-marker tracking is occlusion-fragile.** Hand-holding a drone for the move-test occludes
  its one top marker → `librigidbodytracker` drops the body → extpos stops → the onboard EKF
  **diverges** (`/cfN/odom` ran to tens of metres) and does **not** self-recover; a stack restart
  re-initialises the body and resets the estimate. Irrelevant in flight (marker stays exposed), but
  expect it whenever you manually handle a flying-config drone, and **restart rather than trusting a
  recovered odom.**
- **A bad connection on one drone can starve the other (shared dongle).** The first 2-drone launch
  hit a transient `cf2` link error during log-config setup (`RadioDriver: Could not send packet` →
  `Could not add log config` → a thread crash); that poisoned the shared radio so **cf4's extpos was
  starved and its EKF diverged too**, even though cf4 itself was healthy. **A clean relaunch fixed
  it** — each drone is fine alone, and the retry connected both cleanly. If one drone misbehaves at
  connect, **relaunch the whole stack** rather than flying the "good" one.

---

## 9. Stage A — static-obstacle CBF, first real avoidance flight (2026-06-30) ⚠️ PARTIAL — avoidance reproduced, crashed on landing

First time the Python CBF ([scripts/cbf_headon_test.py](../scripts/cbf_headon_test.py)) ran on real
drones. Setup: **ego=cf2** (SOUTH/−Y) approaches **static obstacle cf4** (NORTH/+Y); `--R 0.7
--vmax 0.15 --obs-speed 0.0 --bias-gain 0.11 --z 0.5`, PeerFeed OFF, firmware colAv OFF (Python CBF
sole avoider). Live auto-abort monitor (sep<0.55 → kill+`/all/land`). **Honest verdict: the
avoidance worked, but the run is a FAIL on two counts — a stall and a crash.** Both below.

### 9.1 What worked — avoidance reproduced on HW
- **Sidestep signature matches sim Test 4.** cf2 drove +Y at static cf4, held separation at the
  barrier, and veered to its **right (+X, east)** — the right-hand bias firing on real hardware
  (bias active 86 % of ticks, `max|bias|=0.110`). Trajectory smooth, no oscillation/runaway.
- **Barrier held during the active-CBF window**: script-reported MIN horiz sep **0.660 m** (≥ R−0.05).
  ~4 cm inside nominal R=0.7 — consistent with sim (min-sep sits just at R) plus the measured HW
  latency eating a few cm exactly as Test 5 predicted; R=0.7 absorbed it. No collision.

### 9.2 Failure (a) — stalled at ~90°, half-loop never completed
- The ego goal was set to the obstacle's position (`cf1 goal=(0.02,1.07)` = cf4's spot). With a
  **static** obstacle sitting **on the goal**, once the ego reaches the barrier abeam (~90° around)
  `v_des` points almost purely **radially** (at the goal=obstacle), leaving near-zero tangential drive;
  the only thing pushing it further around is the bias, and **`--bias-gain 0.11` (vs sim 0.3) was too
  weak** → it crept and stalled at 90°, never completing the half-loop / continuing straight.
- **Fix:** `--goal-beyond` (extend the ego goal PAST the obstacle along the approach line so `v_des`
  keeps a forward+tangential component all the way around) **+ `--bias-gain 0.3`**.

### 9.3 Failure (b) — CRASH on landing (root cause + sim before/after)
- **Symptom:** at the end of the run **both** drones dropped from 0.5 m and **cf2 flipped** — a hard
  fall, not a controlled land. Live monitor: z 0.49→0.21→−0.09 in ~0.8 s (below-floor plunge).
- **NOT** battery (no in-flight voltage was logged — see fix 4 — but both fell at the *exact* land
  instant, ruling out brownout), **NOT** EKF divergence (CSV z stable ~0.50 until streaming stopped),
  **NOT** the CBF (the avoidance window was clean).
- **Root cause — missing low→high handoff.** The driver takes off via the **high-level** commander,
  then streams **low-level** `/cfN/cmd_position` (50 Hz), which latches low-level setpoint priority in
  firmware. `land_both()` then called the **high-level** `Land` **without** first calling
  `/cfN/notify_setpoints_stop`. Result: `Land` is ignored, the last low-level setpoint expires with
  nothing replacing it → commander watchdog timeout → **both drones fall simultaneously**; cf2, mid-
  circle with horizontal motion, flips. `notify_setpoints_stop` is crazyswarm2's *standard* mechanism
  for exactly this transition.
- **Why sim hid it (Tests 3–5):** the same bug was always present in sim, but the sim validation only
  ever measured *separation* — **nobody logged z through the land phase**, and a sim "crash" has no
  physical consequence, so it printed a normal summary and went unnoticed.
- **Causal proof — sim before/after, same stack/scenario (2026-06-30):**

  | land path | run/hold z | land-phase z trajectory |
  |---|---|---|
  | **OLD** (no notify) | 1.00 m hover | 1.00 → 0.94 → 0.37 → **−0.249** in ~0.8 s (uncontrolled drop, below-floor) |
  | **FIXED** (notify) | 1.00 m hover | 1.00 → 0.57 → 0.27 → 0.07 → 0, ~2.2 s **decelerating descent**, settles +0.015 |

  The OLD sim plunge (−0.249, below floor) is the **same failure mode** as the HW crash → fix confirmed
  causal, and the smooth FIXED descent is the controlled land.

### 9.4 Fixes applied (verified in sim; not yet re-flown)
1. **Crash fix** — `land_both()` calls `/cfN/notify_setpoints_stop` (`remain_valid_millisecs=0`) on each
   drone *before* `Land`. ([cbf_headon_test.py](../scripts/cbf_headon_test.py) `land_both`.)
2. **Pass fix** — `--goal-beyond <m>` extends the ego goal past the obstacle (default 0 = old/sim behaviour).
3. **bias-gain** — use **0.3** (sim default) on HW, not 0.11; abeam, the bias is the sole tangential drive.
4. **Logging** — CSV now logs ego/obstacle **attitude** (roll/pitch/yaw from odom, for flip detection) and
   **battery + supervisor `IS_TUMBLED`** via a new `status` log topic in
   [crazyflies_hw_2drone.yaml](../config/crazyflies_hw_2drone.yaml). (The crash had neither — flip was
   inferred from physical observation only.)

The CBF/bias/QP math is unchanged; sim defaults reproduce Tests 3–5. **Re-fly Stage A with**
`--ego-id cf2 --obs-id cf4 --R 0.7 --vmax 0.15 --obs-speed 0.0 --goal-beyond 1.0 --bias-gain 0.3
--peer-hz 20 --peer-delay 0 --peer-noise-std 0 --z 0.5`, and **do not omit `motion_capture_yaml_file`**
at launch (§3 step 2). Then re-assess Stage B.

---

## Open items (⚠️ NO REPO BASIS — resolve on HW)
- ~~**Drone-ID reconciliation for the avoidance stage:** driver commands cf1+cf2 but live drones are
  cf2+cf4.~~ **✅ DONE 2026-06-30** — driver parametrised with `--ego-id`/`--obs-id` (default cf1/cf2 =
  sim-identical); HW uses `--ego-id cf2 --obs-id cf4`. §2A gate re-run and passed (no swap). See §9.
- ~~Vicon real rate, position σ, end-to-end `/cf2/odom` delay — measure in §4 (Stage 0).~~
  **✅ MEASURED 2026-06-29 (cf2, real HW) — see §4.0.** Rate 20 Hz (odom) / 397 Hz (`/poses`);
  horizontal σ 0.8–0.9 cm; delay ≲ 67 ms (1-hop, resolution-limited). Not assumptions — actual
  measurements. R = 0.7 m confirmed amply conservative.
- ~~A dedicated 2-drone HW yaml (§2.4) — does not exist; create or power off cf3/cf4.~~
  **✅ DONE 2026-06-30 — [config/crazyflies_hw_2drone.yaml](../config/crazyflies_hw_2drone.yaml)**
  (cf2+cf4, hover-only, colAv/peer off). See §8.
- Head-on physical layout + matching `initial_position` (§2.3) — current yaml is a quadrant layout.
- Best peer-feed flags (`--ideal-peer` vs `--peer-hz 20`) on HW (§6) — confirm empirically.
- Whether `peer_broadcast_hz` / firmware BVC should be off for a clean Python-CBF-only comparison (§3 step 2).
- Whether a hardware/joystick e-stop exists beyond the software `/all/emergency` (§2.10).
```
