# CBF migration analysis — replacing the BVC safety filter with a CBF-QP

**Status:** analysis only (no code changes). Reference-grade; every claim carries a
`file:line` anchor. Goal: replace the **BVC (Buffered Voronoi Cell)** collision-avoidance
safety filter with a **CBF-QP** safety filter, keeping the high-level planner and
low-level controller deliberately simple — only the safety/avoidance layer changes.

This doc is the working context for the CBF-swap project. It is **not** yet wired into
`CRAZYSIM_MIGRATION.md §4` (project index) — do that when the project is greenlit.

---

## 0. ⚠️ Checkout state — read first (verified 2026-06-26)

The three external forks are present at their **pinned upstream commits with clean working
trees — the `vendor/` patches are NOT applied**. This matters because the BVC *runtime*
described in the workspace docs depends on those patches.

| Fork | On-disk commit | Patch state | Effect |
|---|---|---|---|
| `crazyflie-firmware/` | llanesc @`aa6571dc` (clean) | `sitl-bvc.patch` **NOT applied** | SITL build does **not** compile/run BVC (see below) |
| `cflib-src/` | bitcraze @`c8bf364` (clean) | `extpos-packed.patch` **NOT applied** | `send_extpos_packed` absent → no packed peer broadcast |
| `src/src/crazyswarm2/` | IMRCLab @`3e52b2f` (clean) | no vendor patch exists for it | no peer-broadcast timer; different layout than docs assume |

Evidence:
- Firmware SITL guard still live: [stabilizer.c:332-336](../crazyflie-firmware/src/modules/src/stabilizer.c#L332-L336)
  (`// TODO: Add collision avoidance to SITL` + `#ifndef CONFIG_PLATFORM_SITL`).
- `collision_avoidance.c` absent from the SITL source list `sitl_make/CMakeLists.txt`
  (grep returns nothing; cf. the would-be add in [vendor/.../sitl-bvc.patch:9](../vendor/crazyflie-firmware/sitl-bvc.patch#L9)).
- `send_extpos_packed` / `EXT_POSITION_PACKED` absent from
  `cflib-src/cflib/crazyflie/localization.py` (the patch that would add them is
  [vendor/cflib/extpos-packed.patch](../vendor/cflib/extpos-packed.patch)).
- No `peer_broadcast_hz` reader and no `send_extpos_packed` caller anywhere in
  `src/src/crazyswarm2` (only config comments + verification doc reference it).

**Net:** with the checkout as-is, BVC is inert in SITL, and the peer feed is unwired on
both transports. The firmware *receiver* + `peer_localization` + BVC core are all present
(upstream code) and fully analyzable — that is what Phases 1–2 below document. To exercise
BVC at runtime you must apply the three `vendor/` patches (firmware + cflib) **and** supply
a crazyswarm2 peer-broadcaster (the missing piece — there is no captured patch for it).
This gap is itself relevant to Phase 3: a CBF-QP node would need that same peer feed.

Layer legend used throughout: **[ROS-Py]** ROS2 Python · **[cflib]** Python transport ·
**[FW-C]** Crazyflie firmware C.

---

## 1. Control-stack map (high-level goal → motor commands)

Each stage with its layer and a verified anchor. BVC hook location is the load-bearing fact.

```
 high-level goal
   │
 [ROS-Py] OPTIMIZER  fed_dcsa/radial_coverage_node.py
          → /coverage/leash (Leash: per-drone radii rᵢᵏ)            architecture_map.md:60
   │
 [ROS-Py] PLANNER  cf_coverage_planner/quadrant_figure8_node.py
          (or coverage_planner_node / rl_planner_node)              architecture_map.md:78-80
          → /cfN/policy_target (PoseStamped, ~5 Hz)
   │
 [ROS-Py] STREAMER  setpoint_streamer_node.py
          rate-limited setpoint walk (≤ max_step/tick)              setpoint_streamer_node.py:81-98
          → /cfN/cmd_position (crazyflie_interfaces/Position, 20 Hz) setpoint_streamer_node.py:54-55
   │
 [ROS-Py] BRIDGE  crazyswarm2 crazyflie_server.py (cflib backend)
          sub /cfN/cmd_position                                     crazyflie_server.py:384
          → _cmd_position_changed → cf.commander.send_position_setpoint(x,y,z,yaw)
                                                                     crazyflie_server.py:1164-1170
   │
 [cflib]  TRANSPORT  Commander.send_position_setpoint
          packs TYPE_POSITION on COMMANDER_GENERIC/SET_SETPOINT_CHANNEL
                                                                     commander.py:222-235
          send_packet over UDP (sim) / radio (HW)
   │  CRTP
 [FW-C]   COMMANDER  crtpCommanderGenericDecodeSetpoint
          memset(setpoint,0) then positionDecoder                   crtp_commander_generic.c:414-426
          [positionType] = positionDecoder                          crtp_commander_generic.c:410
          → setpoint.mode.{x,y,z}=modeAbs, position set, velocity=0 crtp_commander_generic.c:383-399
   │
 [FW-C]   stabilizerTask() per-tick loop:                           stabilizer.c:326-341
            commanderGetSetpoint(&setpoint,&state)                  stabilizer.c:326
            supervisorUpdate(&sensorData,&setpoint,step)            stabilizer.c:330
        ►►  collisionAvoidanceUpdateSetpoint(&setpoint,...) ◄◄ BVC  stabilizer.c:335
            supervisorOverrideSetpoint(&setpoint)                   stabilizer.c:339
   │
 [FW-C]   controller(&control,&setpoint,&sensorData,&state,step)    stabilizer.c:341
          (PID / Mellinger / Brescianini / INDI)
   │
 [FW-C]   power distribution / mixer → motor PWM → ESCs | Gazebo
```

**Peer-position side-channel** (feeds BVC, parallel to the setpoint path):

```
 [ROS-Py] crazyswarm2 server — INTENDED peer broadcaster (per config peer_broadcast_hz)
          ✗ NOT PRESENT in this checkout (see §0)
   │  (intended) cflib Localization.send_extpos_packed([(id,x,y,z),...])  ✗ patch unapplied
   │  CRTP port=LOCALIZATION, channel=EXT_POSITION_PACKED(2)
 [FW-C]   crtp_localization_service.c: case EXT_POSITION_PACKED → extPositionPackedHandler
                                                                     crtp_localization_service.c:167-168, 328
          per item: id==my_id → own extpos (estimatorEnqueuePosition)
                    else      → peerLocalizationTellPosition(id,&pos) crtp_localization_service.c:338-344
   │
 [FW-C]   peer_localization.c stores pos + timestamp=xTaskGetTickCount peer_localization.c:22-30
   │
 [FW-C]   BVC reads via peerLocalizationGetPositionByIdx + age filter collision_avoidance.c:320-336
```

### 1.1 Where the BVC hook sits (confirmed, not inferred)

`collisionAvoidanceUpdateSetpoint(&setpoint, &sensorData, &state, stabilizerStep)` runs
**after** the commander has produced the setpoint and `supervisorUpdate` has run, and
**before** `supervisorOverrideSetpoint` and `controller`
([stabilizer.c:330-341](../crazyflie-firmware/src/modules/src/stabilizer.c#L330-L341)). It
mutates the shared `setpoint_t` **in place**; the controller downstream consumes whatever
BVC leaves. This is exactly the seam a CBF-QP filter would occupy (Phase 3).

> In this checkout the call is wrapped in `#ifndef CONFIG_PLATFORM_SITL`
> ([stabilizer.c:333-337](../crazyflie-firmware/src/modules/src/stabilizer.c#L333-L337)),
> so it is compiled out for SITL and only live on HW builds. The `vendor/sitl-bvc.patch`
> removes that guard.

---

## 2. BVC safety-filter contract (the thing being replaced)

### 2.1 Function signatures & I/O contract

**Wrapper** — firmware-facing, reads globals, filters peers, calls Core:
[collision_avoidance.c:307-341](../crazyflie-firmware/src/modules/src/collision_avoidance.c#L307-L341)

```c
void collisionAvoidanceUpdateSetpoint(
    setpoint_t *setpoint, sensorData_t const *sensorData,
    state_t const *state, stabilizerStep_t stabilizerStep);   // :307-308
```

| | What | Anchor |
|---|---|---|
| **READS** | `collisionAvoidanceEnable` (early-return if 0) | :310-312 |
| | static `params` (the `colAv` PARAM group) | :315, :328 |
| | peers: `peerLocalizationGetPositionByIdx(i)`, i∈[0,`PEER_LOCALIZATION_MAX_NEIGHBORS`=10) | :320-322 |
| | skips empty slots (`otherPos==NULL \|\| id==0`) | :324-326 |
| | age filter: drop if `now - pos.timestamp > maxPeerLocAgeMillis` (when ≥0) | :315, :328-330 |
| | `state->position` (own pos, read inside Core) | :117 |
| **WRITES** | packs surviving peers into `workspace[3*nOthers..]`; `nOthers` count | :332-336 |
| | delegates all setpoint mutation to Core | :338 |
| | `latency` LOG var | :305, :340, :343-345 |

**Core** — pure (no FreeRTOS/params/peer deps), does the geometry:
[collision_avoidance.c:97-253](../crazyflie-firmware/src/modules/src/collision_avoidance.c#L97-L253)

```c
void collisionAvoidanceUpdateSetpointCore(
    collision_avoidance_params_t const *params,
    collision_avoidance_state_t *collisionState,
    int nOthers, float const *otherPositions, float *workspace,
    setpoint_t *setpoint, sensorData_t const *sensorData, state_t const *state);  // :97-103
```

| | What | Anchor |
|---|---|---|
| **READS** | `state->position` → `ourPos` | :117 |
| | `otherPositions` (nOthers × xyz), `params->ellipsoidRadii` (stretched metric) | :116, :122-131 |
| | `params->bboxMin/Max`, `horizonSecs`, `maxSpeed` → bounding-box rows + max-step | :135-147 |
| | `setpoint->position`, `setpoint->velocity`, `setpoint->mode.x` | :155-158, :192 |
| | `collisionState->lastFeasibleSetPosition` (fallback when cell empty) | :209-210 |
| **WRITES** | `setpoint->position = svec2vec(setPos)` | :251 |
| | `setpoint->velocity = svec2vec(setVel)` | :252 |
| | `collisionState->lastFeasibleSetPosition` (modeVelocity & modeAbs paths) | :190, :245 |

The Voronoi cell is built as a polytope `A x ≤ B`: one half-plane per peer in an
ellipsoid-stretched metric (downwash-aware, radii 0.3/0.3/0.9)
[:122-131](../crazyflie-firmware/src/modules/src/collision_avoidance.c#L122-L131), plus 6
bounding-box faces that also cap the per-tick step to `maxDist = horizonSecs * maxSpeed`
[:135-147](../crazyflie-firmware/src/modules/src/collision_avoidance.c#L135-L147). It does
**not** touch attitude, yaw, thrust, or `mode.{y,z,yaw}` — only x/y/z position & velocity.

### 2.2 The three branches (on `setpoint->mode.x`)

| Branch | Condition | What it does to position / velocity | Anchor |
|---|---|---|---|
| **modeVelocity** | `mode.x == modeVelocity` | Treats setpoint as a goal *velocity*. If `ourPos` is inside the cell, projects `pseudoGoal = horizon·setVel` into the cell (sidestep heuristic) and rescales velocity; if projection fails → `setVel = 0`. If `ourPos` is **outside** the cell, ignores goal vel and steers velocity toward nearest in-cell point, clamped to `maxSpeed`. **Position untouched**; velocity rewritten. | :158-191 |
| **modeAbs** | `mode.x == modeAbs` | Treats setpoint as an absolute *position*. Projects the relative goal `setPos-ourPos` into the buffered cell (`setPosRelativeNew`). Three sub-cases below. | :192-246 |
| **(unsupported)** | else | **Do nothing** — setpoint passes through unmodified. | :247-249 |

**modeAbs sub-cases** (this is the path real flights hit — see §2.4):

| Sub-case | Condition | Effect | Anchor |
|---|---|---|---|
| cell empty | projection didn't converge | `setVel=0`; `setPos=lastFeasibleSetPosition` (or `ourPos` if NaN) → hold | :198-219 |
| **waypoint** | `setVel == 0` (position, no velocity) | `setPos = ourPos + setPosRelativeNew` (clamped into buffered cell); velocity stays 0 | :220-224 |
| trajectory | `setVel != 0` | if goal left the cell → degrade to waypoint (`setVel=0`); else scale velocity by ray-cast so it can't exit cell within horizon | :225-244 |

After any branch: `setpoint->position`/`velocity` written back
[:251-252](../crazyflie-firmware/src/modules/src/collision_avoidance.c#L251-L252).

### 2.3 Peer-position path (end to end)

**Broadcaster (ROS side) — MISSING in this checkout.** Intended design: the crazyswarm2
server pushes every drone its neighbors' `(id,x,y,z)` at `peer_broadcast_hz`
([config/crazyflies_sitl_multi.yaml:49](../config/crazyflies_sitl_multi.yaml#L49) = 30 Hz;
[config/crazyflies_hw.yaml:50](../config/crazyflies_hw.yaml#L50) = 10 Hz) via
`cf.extpos.send_extpos_packed(...)`. **Reality:** the on-disk server
([crazyflie_server.py](../src/src/crazyswarm2/crazyflie_server_py/crazyflie_server_py/crazyflie_server.py))
only forwards each drone's **own** mocap pose via `send_extpos`/`send_extpose`
([:1131-1149](../src/src/crazyswarm2/crazyflie_server_py/crazyflie_server_py/crazyflie_server.py#L1131-L1149)),
has no `peer_broadcast_hz` reader, and `send_extpos_packed` does not exist in the on-disk
cflib (§0). So no peer data reaches the firmware as the checkout stands.

**cflib pack (intended)** — `send_extpos_packed` would pack each `(id,x,y,z)` as
`<Bhhh>` (mm) on `port=LOCALIZATION, channel=EXT_POSITION_PACKED(2)`, ≤4 per packet
([vendor/cflib/extpos-packed.patch:17-40](../vendor/cflib/extpos-packed.patch#L17-L40)).

**Firmware receive → store → read (all present upstream):**
1. `EXT_POSITION_PACKED = 2`; dispatch `case EXT_POSITION_PACKED → extPositionPackedHandler`
   ([crtp_localization_service.c:71](../crazyflie-firmware/src/modules/src/crtp_localization_service.c#L71),
   [:167-168](../crazyflie-firmware/src/modules/src/crtp_localization_service.c#L167-L168)).
2. Handler splits by id: `item->id == my_id` (own addr LSB, set at
   [:151](../crazyflie-firmware/src/modules/src/crtp_localization_service.c#L151)) →
   `estimatorEnqueuePosition`; **else → `peerLocalizationTellPosition(item->id,&ext_pos)`**
   ([:338-344](../crazyflie-firmware/src/modules/src/crtp_localization_service.c#L338-L344)).
3. `peerLocalizationTellPosition` stores pos + stamps `timestamp = xTaskGetTickCount()`
   ([peer_localization.c:22-30](../crazyflie-firmware/src/modules/src/peer_localization.c#L22-L30));
   capacity `PEER_LOCALIZATION_MAX_NEIGHBORS = 10`
   ([peer_localization.h:16](../crazyflie-firmware/src/modules/interface/peer_localization.h#L16)).
4. BVC reads each slot via `peerLocalizationGetPositionByIdx(i)`
   ([peer_localization.c:57](../crazyflie-firmware/src/modules/src/peer_localization.c#L57))
   and applies the `maxPeerLocAgeMillis` staleness filter
   ([collision_avoidance.c:322-330](../crazyflie-firmware/src/modules/src/collision_avoidance.c#L322-L330)).
   Stale/empty peers are simply excluded → BVC silently degrades to bbox-only avoidance.

### 2.4 Which mode real flights actually exercise

Traced `/cfN/cmd_position` → motors:
- Streamer publishes `crazyflie_interfaces/Position`
  ([setpoint_streamer_node.py:91-98](../src/cf_coverage_planner/cf_coverage_planner/setpoint_streamer_node.py#L91-L98)).
- Server `_cmd_position_changed` → `cf.commander.send_position_setpoint(x,y,z,yaw)`
  ([crazyflie_server.py:1164-1170](../src/src/crazyswarm2/crazyflie_server_py/crazyflie_server_py/crazyflie_server.py#L1164-L1170)).
- cflib packs `TYPE_POSITION`
  ([commander.py:222-235](../cflib-src/cflib/crazyflie/commander.py#L222-L235)).
- Firmware: `memset(setpoint,0)` then `positionDecoder` sets
  `mode.{x,y,z}=modeAbs`, `position={x,y,z}`, and **leaves velocity at 0**
  ([crtp_commander_generic.c:414-426](../crazyflie-firmware/src/modules/src/crtp_commander_generic.c#L414-L426),
  [:383-399](../crazyflie-firmware/src/modules/src/crtp_commander_generic.c#L383-L399),
  [:410](../crazyflie-firmware/src/modules/src/crtp_commander_generic.c#L410)).

**⇒ Real flights hit the BVC `modeAbs` + `setVel==0` "waypoint" branch**
([collision_avoidance.c:220-224](../crazyflie-firmware/src/modules/src/collision_avoidance.c#L220-L224)):
the commanded absolute position is projected into the buffered Voronoi cell each tick
(`setPos = ourPos + setPosRelativeNew`), velocity held at 0. Even with **zero peers**, the
bbox rows still cap the relative setpoint to a `maxDist = horizon·maxSpeed = 1.0·0.5 =
0.5 m` box around current position
([:135-147](../crazyflie-firmware/src/modules/src/collision_avoidance.c#L135-L147)) — a
real behavioral effect of BVC independent of inter-drone avoidance.

> Note: the RL path uses `receding_horizon_streamer_node.py` → `/cfN/upload_trajectory` +
> `/cfN/start_trajectory` (high-level commander), a different setpoint path
> ([architecture_map.md:82](architecture_map.md)). Whether that path produces modeAbs with
> nonzero velocity (trajectory sub-case) vs. high-level-commander setpoints is a follow-up
> to confirm if CBF must cover the RL deployment too.

### 2.5 Params, enable flag, defaults, sim-vs-HW

**Enable flag:** `colAv.enable` → static `collisionAvoidanceEnable`, **default 0 (off)**
([collision_avoidance.c:270](../crazyflie-firmware/src/modules/src/collision_avoidance.c#L270),
PARAM [:397](../crazyflie-firmware/src/modules/src/collision_avoidance.c#L397)). Set on via
firmware_params `colAv.enable: 1`
([config/crazyflies_sitl_multi.yaml:59-60](../config/crazyflies_sitl_multi.yaml#L59-L60)).

**`colAv` PARAM group** ([collision_avoidance.c:390-428](../crazyflie-firmware/src/modules/src/collision_avoidance.c#L390-L428)),
defaults from the static `params` init ([:272-282](../crazyflie-firmware/src/modules/src/collision_avoidance.c#L272-L282)):

| Param (`colAv.`) | Field | Default | Meaning |
|---|---|---|---|
| `enable` | `collisionAvoidanceEnable` | 0 | master on/off |
| `ellipsoidX/Y/Z` | `ellipsoidRadii` | 0.3 / 0.3 / 0.9 | collision ellipsoid (z>xy = downwash) |
| `bboxMin{X,Y,Z}` | `bboxMin` | −FLT_MAX | flight-area lower bound |
| `bboxMax{X,Y,Z}` | `bboxMax` | +FLT_MAX | flight-area upper bound |
| `horizon` | `horizonSecs` | 1.0 | planning horizon (s) |
| `maxSpeed` | `maxSpeed` | 0.5 | max speed / step scale (m/s) |
| `sidestepThrsh` | `sidestepThreshold` | 0.25 | deadlock sidestep trigger |
| `maxPeerLocAge` | `maxPeerLocAgeMillis` | 5000 | peer staleness cutoff (ms); <0 disables filter |
| `vorTol` | `voronoiProjectionTolerance` | 1e-5 | projection convergence tol |
| `vorIters` | `voronoiProjectionMaxIters` | 100 | projection iteration cap |

**Sim-vs-HW differences in how BVC is enabled/fed:**

| Concern | Sim (SITL) | Hardware |
|---|---|---|
| BVC compiled in | Only if `sitl-bvc.patch` applied (adds `collision_avoidance.c` to `sitl_make/CMakeLists.txt` + removes the `#ifndef CONFIG_PLATFORM_SITL` guard). **Not applied in this checkout** → BVC absent. | Always compiled (guard inactive on HW). |
| BVC enabled | `colAv.enable: 1` via YAML firmware_params ([crazyflies_sitl_multi.yaml:59-60](../config/crazyflies_sitl_multi.yaml#L59-L60)) | same YAML mechanism ([crazyflies_hw.yaml](../config/crazyflies_hw.yaml)) |
| Own-pose source | Gazebo ground truth into firmware EKF | mocap/Lighthouse → EKF ([CRAZYSIM_MIGRATION.md:174-175](../CRAZYSIM_MIGRATION.md#L174)) |
| Peer feed | intended `peer_broadcast_hz: 30.0` over UDP | intended `peer_broadcast_hz: 10.0` over radio. **Broadcaster + cflib pack absent in this checkout (§0)** |
| Peer transport | identical CRTP mechanism, UDP vs radio (transport-agnostic by design) | — |

### 2.6 Full enable→effect chain (every knob that touches BVC)

```
1. BUILD     sitl_make/CMakeLists.txt includes collision_avoidance.c   [sim only; patch]
             AND stabilizer.c guard removed (#ifndef CONFIG_PLATFORM_SITL)  [sim only; patch]
                 → otherwise the hook at stabilizer.c:335 is compiled out
2. ENABLE    firmware_param colAv.enable = 1   (default 0)             collision_avoidance.c:310,397
3. PEER FEED crazyswarm2 broadcaster @ peer_broadcast_hz              [MISSING in checkout]
             → cflib send_extpos_packed [patch] → CRTP ch 2
             → extPositionPackedHandler → peerLocalizationTellPosition  crtp_localization_service.c:344
             → peer_localization store + timestamp                     peer_localization.c:30
4. TUNE      colAv.{ellipsoid*, bbox*, horizon, maxSpeed, maxPeerLocAge, vorTol, vorIters}
                                                                       collision_avoidance.c:402-427
5. RUNTIME   stabilizer.c:335 → UpdateSetpoint → age-filter peers      collision_avoidance.c:328
             → Core: modeAbs waypoint projection                       collision_avoidance.c:220-224
             → writes setpoint.position/velocity                       collision_avoidance.c:251-252
6. EFFECT    controller(&setpoint,...) consumes modified setpoint      stabilizer.c:341
```

Every link above is required; the chain currently breaks at **steps 1 and 3** in this
checkout (patches unapplied + broadcaster missing). Any of `colAv.enable=0`, BVC not
compiled, or peers stale/absent reduces BVC to (at most) bbox self-limiting.

---

*Phase 3 (CBF insertion options) to be appended after review.*
