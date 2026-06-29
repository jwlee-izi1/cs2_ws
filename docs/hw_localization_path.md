# Hardware localization & peer-position data path

Investigation (read-only, no code changed) of how state estimation (ego pose) and
inter-drone position sharing are wired in this repo, and how they change sim → hardware.
Motivation: hardening the CBF / BVC avoidance for real flight — knowing the exact data
path and estimate quality of "where cf1 thinks cf2 is."

> Scope: infrastructure analysis supporting the CBF migration work
> (`docs/cbf_migration_analysis.md`). All claims below cite file:line. Items the repo
> does not pin down are marked **NO REPO BASIS** rather than guessed.

---

## TL;DR — exact path "how cf1 learns cf2's position" (hardware)

```
Vicon (192.168.0.62:801)
  → motion_capture_tracking node → /poses (NamedPoseArray)
  → crazyflie_server._poses_changed → cf2.extpos.send_extpos(e)   [CRTP LOCALIZATION / EXT_POS]
  → cf2 firmware EKF (estimator 2) fuses → cf2 stateEstimate.x/y/z
  → cf2 odom log (20 Hz) back to server → server._peer_latest_pos['cf2']
  → peer_broadcast timer (10 Hz) → cf1.loc.send_extpos_packed([(id,x,y,z)])  [EXT_POSITION_PACKED]
  → cf1 firmware peer_localization → BVC (collision_avoidance.c)
```

- **Centralized, not peer-to-peer.** All peer traffic goes through the server hub.
- The "central" source is **not** mocap re-broadcasting raw observations — it is **each
  drone's own EKF estimate** (mocap-fused), relayed one extra hop. What cf1 receives for
  cf2 is "cf2's estimate of cf2," not "Vicon's view of cf2."
- **Same mechanism in sim and hardware.** Only the cflib transport differs:
  `udp://` (sim) vs `radio://` (hardware). Server code has no sim/hw branch.

---

## 1. Ego self-localization (hardware)

External system = **Vicon motion capture** (not Lighthouse / UWB / onboard-only).

- `config/motion_capture.yaml:1-6` — `type: "vicon"`, `hostname: "192.168.0.62"`.
- `config/crazyflies_hw.yaml:37-40` — `tracking: "librigidbodytracker"`,
  `marker: default_single_marker` (1 retro ball per drone).
- `CRAZYSIM_MIGRATION.md:399-402`, `:471` — Vicon Tracker 3.10; mocap → firmware EKF
  fusion confirmed against `/cfN/odom`.

Pose injection path (hardware): server subscribes `/poses`, injects per-drone extpos via CRTP.

- `src/crazyswarm2/crazyflie/scripts/crazyflie_server.py:375-378` — `/poses` subscription.
- `crazyflie_server.py:1161-1184` — `_poses_changed` → `cf.extpos.send_extpos(x,y,z)`
  (position-only when quat is NaN) / `send_extpose(...)`.
- Estimator 2 (Kalman) + PID controller: `config/crazyflies_hw.yaml:53`; mocap trust
  `locSrv.extPosStdDev: 0.01` at `:54`.

### sim yaml vs hw yaml — how pose enters the estimator

| | sim (`crazyflies_sitl_2drone.yaml`) | hw (`crazyflies_hw.yaml`) |
|---|---|---|
| motion_capture | `enabled: false`, `tracking: "off"` (`:24-26`) | enabled, librigidbodytracker (`:37-40`) |
| uri | `udp://127.0.0.1:1985N` (`:13-19`) | `radio://0/100/2M/E7E7E7E7EN` (`:16-31`) |
| ego pose source | Gazebo plugin injects ground truth (below) | Vicon → `_poses_changed` → extpos |

In sim the server's `/poses`→extpos path does **not** run (motion_capture off). Instead the
**Gazebo plugin injects ground-truth pose directly over the same CRTP ext-pose channel** —
confirming the "perfect mocap via CrtpExtPose" premise:

- `crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/plugins/CrazySim/crazysim_plugin.cpp:178-196`
  — `OdomCallback` packs Gazebo `gz.msgs.Odometry` (OdometryPublisher ground truth) into
  `CrtpExtPose_s { header = CRTP_PORT_LOCALIZATION, id = CRTP_GEN_LOC_ID_EXT_POS, x,y,z,q }`.
- So **sim ego-pose channel == hw Vicon→extpos channel** (same CRTP ext-pose port 0x06);
  only the source quality differs (Gazebo truth vs Vicon).
- Sim IMU/baro also feed the EKF: `crazyflie-firmware/src/hal/src/sensors_sitl.c:205-225`;
  `crazysim_plugin.cpp:142-176`.

**`mocap:=False` → hardware:** the disabled-in-sim mocap is replaced by enabling the
`robot_types.cf21.motion_capture` block in `crazyflies_hw.yaml` + the Vicon backend in
`motion_capture.yaml` (`CRAZYSIM_MIGRATION.md:220-222`).

---

## 2. Inter-drone position sharing

Same mechanism both sides (not sim-only). Server-side code is transport-agnostic.

- No sim/hw branch in the patch: `vendor/crazyswarm2/bvc-peer-broadcast.patch:42-60` —
  `_broadcast_peer_positions` calls `swarm._cfs[uri].cf.loc.send_extpos_packed(peers)`.
  Whether that link is UDP or radio is decided by the cflib URI; same code.
- Explicit: `CRAZYSIM_MIGRATION.md:325-326` — "identical mechanism over sim UDP or real
  radio", "both".

Peer position source = each drone's own EKF `stateEstimate` (not mocap directly):

- `bvc-peer-broadcast.patch:65-77` — `_peer_latest_pos[cf_name] = (x,y,z)` filled from the
  `stateEstimate.x/y/z` firmware odom-log callback.
- ⇒ origin is **(a) central**: external mocap → each drone EKF → server collects → re-broadcasts.
  **Not (b) direct drone-to-drone.** All peer traffic transits the server.
- Self never appears as its own peer: `bvc-peer-broadcast.patch:16-20` (ids 1..N, never my_id),
  each drone gets only its peers (`:50-54`).
- Firmware consumes it: peer_localization → BVC at
  `crazyflie-firmware/src/modules/src/collision_avoidance.c:307-333`.

---

## 3. Estimate-quality difference sim vs hw (directly affects CBF robustness)

| | sim | hardware |
|---|---|---|
| ego pose source | Gazebo OdometryPublisher truth → CrtpExtPose | Vicon → extpos |
| ego pose inject rate | **200 Hz** (`model.sdf.jinja:306` `odom_publish_frequency: 200`) | Vicon rate — repo only hints QoS deadline **100 Hz** (`motion_capture.yaml:12-13`); actual rate/noise/latency **NO REPO BASIS** |
| ego noise / latency | 0 (ground truth) | quantitative values **NO REPO BASIS**; qualitative only: EKF converge ~1–2 s, generic latency note (`CRAZYSIM_MIGRATION.md:502-509`) |

- sim-"perfect" basis: `CRAZYSIM_MIGRATION.md:174`, `:503-504`.
- Vicon noise/latency not quantified anywhere → **NO REPO BASIS**. But a tracker-quality
  fragility trace exists: `max_fitness_score 0.001`, plus a logged failure where ghost/wrong
  markers were grabbed when drones passed close (`motion_capture.yaml:53-57`). Implication:
  **hw estimate is weakest exactly when drones are close** — the moment the CBF matters most.

### Peer broadcast rate (drives the CBF's v_obs)

- sim: `peer_broadcast_hz: 30.0` (`crazyflies_sitl_2drone.yaml:39`), odom log 30 Hz (`:55`).
- **hw: `peer_broadcast_hz: 10.0`** (`crazyflies_hw.yaml:50`, "30 Hz was heavy"); odom-log
  source **20 Hz** (`:64`). Effective peer update = min(20, 10) = **10 Hz**, 1/3 of sim.
- Firmware staleness bound 5 s: `collision_avoidance.c:279` `maxPeerLocAgeMillis = 5000`,
  filtered at `:328`.

### ⚠️ The peer feed carries POSITION ONLY — no velocity

- `collision_avoidance.c:322-333` uses only `otherPos->pos.x/y/z`; `send_extpos_packed`
  payload is `(id,x,y,z)` only (`bvc-peer-broadcast.patch:52-54`).
- Implication for the CBF: **v_obs (relative velocity) must be finite-differenced from these
  position broadcasts.** Its quality is bounded by (i) the 10 Hz hw broadcast rate, (ii) the
  2-hop relay delay (cf2 EKF → 20 Hz odom → server → 10 Hz re-broadcast), (iii) Vicon+EKF
  position noise. The smooth, 30 Hz, noise-free v_obs in sim arrives at 10 Hz with noise and
  latency on hardware — the core target for robustness hardening.

---

## 4. Hardware run path

Same nodes; only backend/yaml swap — no separate launch needed.

- `CRAZYSIM_MIGRATION.md:109-149` "one control plane, two backends": planner / streamer /
  CBF node etc. are identical sim↔hw; the server publishes the same topics/services.
- Transition (`CRAZYSIM_MIGRATION.md:204-222`):
  ```bash
  # sim
  ros2 launch crazyflie launch.py backend:=cflib crazyflies_yaml_file:=.../crazyflies_sitl_multi.yaml
  # hw (same node code)
  ros2 launch crazyflie launch.py backend:=cflib crazyflies_yaml_file:=.../crazyflies_hw.yaml
  ```
  Both use `backend:=cflib`; the diff is the yaml `uri:` (`radio://` vs `udp://`) + the
  `motion_capture` block (`:220-222`).
- The head-on test uses a dedicated sim launcher (`scripts/sitl_2drone_headon.sh`, Gazebo
  `gz sim -s` + per-drone SITL containers). That only brings up sim infra; the control plane
  above it is identical.

---

## Open questions — NO REPO BASIS (measure these during CBF hw validation)

- Vicon actual output rate (100 Hz is a QoS deadline hint, not the source rate), position
  noise σ, and end-to-end latency — quantitative values.
- Quantitative model of mocap degradation when drones are close (only the qualitative
  ghost-marker trace at `motion_capture.yaml:57`).
- Measured total 2-hop peer-relay delay.

These three are the blanks to fill empirically (log `/cfN/odom` + peer-broadcast timestamps).
