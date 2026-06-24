# Multi-drone hardware handoff — 1 → 4 drones + thermal mapping

Written 2026-05-23. **Handoff from the single-drone debug chat
(2026-05-20 → 23) to the multi-drone phase chat.**

Single-drone HW is now solid. This doc takes you the rest of the way:
**4 drones flying simultaneously + thermal mapping for all 4** — ready
as a demo-video flight. Stops before fed_dcsa (which needs sim
re-finetuning first — see §7).

**This is a transitional handoff** (single-drone debug chat → multi-
drone phase chat). The **per-drone verification procedure** it invokes
lives in a separate **infrastructure document**:
[docs/hw_drone_verification.md](hw_drone_verification.md) — the
canonical per-drone single-marker verification procedure, reusable for
any drone, anytime (also the per-drone log, §7 of that doc). This
handoff invokes that procedure once per drone (see §4); read it first
if you're unfamiliar with the per-drone steps. The two non-negotiables
(yaw≈0 placement, channel 100) are restated briefly in §3 here for
quick reference, but the full explanation lives in the verification doc.

## 1. What this doc is for

You inherit a workspace where single-drone HW flights are solid (cf2,
cf3, cf4 verified; cf1 dogfooded — see plan file). The research target
is 4 drones flying together for thermal mapping, and eventually
fed_dcsa. This doc:

1. Walks the **per-drone bringup** for all 4 drones (delegating to
   the verification doc for the actual procedure).
2. Adds the **multi-drone simultaneous test** (staggered heights to
   eliminate collision risk).
3. Layers in **thermal mapping for all 4 drones** (the demo video).
4. Hands off to fed_dcsa via §7 (with prerequisites: sim re-finetuning
   informed by HW constraints).

## 2. State of play (what's verified)

| Drone | Status (2026-05-23) | Notes |
|---|---|---|
| cf1 | dogfood pending (this chat) | ch100 set; placement+test in Part A of plan |
| cf2 | ✅ envelope Pass 1 @ 1.0 m/s clean | first complete clean envelope flight |
| cf3 | ✅ flew clean repeatedly | the "reliable one" — confirmed yaw fix |
| cf4 | ✅ takeoff-hover-land clean (2026-05-21) | not damaged after all (was yaw+ch80) |

All on **firmware 2025.12**, **channel 100**, **single-marker**, with
the **yaw≈0 placement requirement**.

**What didn't need to change** (good news for the next chat — the
plumbing is already multi-drone):
- `firmware_params` in `config/crazyflies_hw.yaml` (PID controller,
  Kalman estimator, `extPosStdDev: 0.01`).
- `config/motion_capture.yaml` (single-marker `default_single_marker`,
  librigidbodytracker).
- `src/fed_dcsa/` — `radial_coverage_node` is fully N-drone
  parameterized via the `drone_names` list.
- `src/cf_coverage_planner/launch/coverage_demo.launch.py` — already
  spawns N planners + N streamers from the YAML drone list.
- `scripts/thermal_demo.sh`, `scripts/coverage_restart.sh` — already
  swap sim ↔ HW via the `CFLIES_YAML` env var.

So scaling 1 → 4 on HW is **almost entirely a config + procedure
problem**, not a code problem.

## 3. The two non-negotiables (quick reference)

Full mechanism + signature + fix lives in
[hw_drone_verification.md §3](hw_drone_verification.md). Restating
here so this doc is self-contained:

1. **Place every drone at yaw ≈ 0** — front (+X) pointing along the
   Vicon +X axis, ~10–20° eyeball. Single-marker mocap gives no yaw;
   the firmware EKF assumes yaw=0 at boot. Wrong placement → drone
   runs away on takeoff. Memory: `single-marker-yaw-zero-placement`.
2. **Every drone on channel 100** — set per-drone in cfclient AND in
   the YAML URI (`radio://0/100/2M/E7E7E7E7E[N]`). Channel 80 sits in
   the WiFi band; intermittent link drops. Memory:
   `hw-radio-link-intermittent`.

## 4. Per-drone bringup (1 → 4)

**For each drone (cf1, cf2, cf3, cf4):** run the full **infrastructure
procedure** in [hw_drone_verification.md §4](hw_drone_verification.md).
That doc is the canonical single-marker per-drone verification — this
handoff just *invokes* it once per drone, then adds the multi-drone
layer on top. The verification doc covers: cfclient → ch100, place at
yaw≈0, fresh battery, Vicon probe for `initial_position`, YAML config,
launch + 60s idle-link gate, instrumented takeoff-hover-land, pass
criteria. **Log each per-drone result in `hw_drone_verification.md`
§7** (the per-drone verification log) so the fleet state stays
canonical.

After all 4 individually pass, edit `config/crazyflies_hw.yaml` to
enable all 4 simultaneously:
```yaml
robots:
  cf1: { enabled: true, uri: radio://0/100/2M/E7E7E7E7E1,
         initial_position: [x1, y1, z1], type: cf21 }
  cf2: { enabled: true, uri: radio://0/100/2M/E7E7E7E7E2,
         initial_position: [x2, y2, z2], type: cf21 }
  cf3: { enabled: true, uri: radio://0/100/2M/E7E7E7E7E3,
         initial_position: [x3, y3, z3], type: cf21 }
  cf4: { enabled: true, uri: radio://0/100/2M/E7E7E7E7E4,
         initial_position: [x4, y4, z4], type: cf21 }
```

Each drone's `initial_position` from its **own Vicon probe** while the
other drones are out of the volume — that's the cleanest way to be
certain which marker is which.

**Placement for multi-drone:** spread the 4 drones out so their markers
are physically **≥ 30 cm apart**. librigidbodytracker assigns each
drone's rigid body to the nearest marker at startup, using
`initial_position`. Markers too close → assignment confusion or ID
swaps.

Relaunch the flight stack with all 4 enabled:
```bash
cd ~/cs2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
nohup ros2 launch crazyflie launch.py \
  backend:=cflib \
  crazyflies_yaml_file:=$HOME/cs2_ws/config/crazyflies_hw.yaml \
  motion_capture_yaml_file:=$HOME/cs2_ws/config/motion_capture.yaml \
  gui:=false > /tmp/hw_bringup.log 2>&1 &
```

Wait for **four** `is fully connected!` lines. If any drone fails to
connect, re-run that drone's single-drone verification in isolation
(`enabled: false` for the other three) to isolate.

**Per-drone 60s idle-link gate** (cheap insurance with 4 radios
simultaneously):
```bash
sleep 60
DROPS=$(grep -cE "Too many packets|is disconnected" /tmp/hw_bringup.log)
echo "link drops in 60s: $DROPS"
# Expect 0.
```

## 5. Multi-drone simultaneous test

**The full multi-drone verification procedure is its own infrastructure
doc:** [docs/hw_multi_drone_verification.md](hw_multi_drone_verification.md).

That doc covers (parallel to how `hw_drone_verification.md` covers
single-drone): staged 2 → 3 → 4 bringup with explicit gates per
stage, a per-quadrant figure-8 motion stage on top of the existing
planner-streamer chain, onboard BVC firmware collision avoidance (Stage
3b onwards), a BVC sanity test (Stage 3c), a multi-drone envelope sweep
mirroring the single-drone one, a multi-drone flight_logger that
reports per-second inter-drone min/max spacing, and a per-test log
in §7 so swarm-level state stays canonical.

**Use that doc for any multi-drone bringup.** This handoff doc invokes
it once for the full 4-drone phase; demo morning re-runs the same
procedure end-to-end.

For the **demo video specifically**: after `hw_multi_drone_verification.md`
Stages 0-sim through 4 pass, combine §4.9 (figure-8 + thermal mapping)
with the RViz GridMap visualization — each drone, at the single
verification height (0.6 m), contributes thermal samples from its own
quadrant, the map fills in, the RViz heatmap is what you screen-capture.
Thermal pipeline details live in
[docs/thermal_mapping_demo.md](thermal_mapping_demo.md).

**DO NOT** use the single-drone cross pattern for multi-drone — 4
drones all crossing the origin = collision. The single-drone cross
pattern is a per-drone speed-envelope test (see plan file's envelope
section), not a multi-drone test.

## 6. Thermal mapping for all 4 drones

```bash
ros2 launch thermal_mapping thermal_mapping_demo.launch.py \
  drone_names:=cf1,cf2,cf3,cf4
```

Brings up:
- One `thermal_sensor_node` per drone — subscribes to `/cfN/odom`,
  publishes `/cfN/thermal/raw` (16×16 synthetic readings at 5 Hz,
  gated on `z > 0.2 m`).
- One central `thermal_mapper_node` — aggregates all per-drone
  readings into `/thermal_map` at 2 Hz.
- RViz with the GridMap plugin pre-configured to show `/thermal_map`.

Map fills **~4× faster** than single-drone — much better for the demo
video.

For the video: combine §5's multi-drone simultaneous hover with
thermal mapping running — each drone, at a different height, contributes
thermal samples from its own (x, y) location, the map fills in, the
RViz heatmap is what you screen-capture.

Thermal pipeline details (sensor model, hot-spot config, gating):
[docs/thermal_mapping_demo.md](thermal_mapping_demo.md). The original
single-drone-thermal handoff doc
[docs/thermal_mapping_hw_handoff.md](thermal_mapping_hw_handoff.md)
covers the multi-drone `drone_names` arg pattern (it already supports
N drones).

## 7. What this doc does NOT cover / next phase

**fed_dcsa-on-HW is deferred.** Before running fed_dcsa on the real
fleet, the next-next chat must **re-finetune the fed_dcsa sim params**
with HW-informed constraints we discovered:

1. **Battery 4–7 minutes per flight.** The current
   `src/cf_coverage_planner/config/arena_4drone.yaml` was tuned for
   indefinite sim runs. On HW the demo must initialize and converge
   faster:
   - Lower round count (or higher `round_rate_hz` so each round is
     shorter).
   - Quicker takeoff + land (use HLC with shorter durations).
   - Trim any per-round settle time.

2. **Range limit ≈ 1.8 m per drone.** The −Y wall in the lab arena is
   at about y = −2.3 m; with takeoff-position offset + decel overshoot
   at speed, drones must stay inside ~1.8 m radius. **Cap
   `r_star_per_drone` and `r_max_per_drone`** in `arena_4drone.yaml`
   accordingly. Currently they are 2.0 / 2.3 m, which is unsafe
   on this arena.

3. **`max_setpoint_velocity` per drone** — use envelope-test data. cf3
   was verified clean to **1.5 m/s** in the prior session. The sim
   default is 1.2 m/s, which fits comfortably. If you want a faster
   demo for the paper video, bump to 1.5 m/s — but verify each drone
   individually first (re-run the envelope sweep per drone).

After sim re-finetuning → fed_dcsa-on-HW dry run (just controller +
planner, no thermal yet) → thermal + fed_dcsa demo video.

## 8. Cross-references / how this fits

| What | Where |
|---|---|
| **Per-drone verification & debug** | [docs/hw_drone_verification.md](hw_drone_verification.md) |
| **Multi-drone verification (staged 2→3→4 + figure-8 + BVC + per-test log)** | [docs/hw_multi_drone_verification.md](hw_multi_drone_verification.md) |
| **Workspace infrastructure & history** | [CRAZYSIM_MIGRATION.md §3](../CRAZYSIM_MIGRATION.md) |
| Thermal mapping pipeline | [docs/thermal_mapping_demo.md](thermal_mapping_demo.md), [docs/thermal_mapping_hw_handoff.md](thermal_mapping_hw_handoff.md) |
| Federated coverage / fed_dcsa | [docs/federated_coverage.md](federated_coverage.md) |
| Multi-drone control codepath | `src/cf_coverage_planner/launch/coverage_demo.launch.py`, `src/fed_dcsa/fed_dcsa/radial_coverage_node.py` |
| Figure-8 verification planner | `src/cf_coverage_planner/cf_coverage_planner/quadrant_figure8_node.py` |
| Multi-drone sim YAML pattern | `config/crazyflies_sitl_multi.yaml` |
| Memories | `single-marker-yaw-zero-placement`, `hw-radio-link-intermittent`, `multi-drone-hw-verification` |

## 9. Quick troubleshooting (multi-drone-specific)

**The full multi-drone-specific symptom → cause table lives in
[hw_multi_drone_verification.md §5.5](hw_multi_drone_verification.md).**
Includes everything that used to live here (connection failures, ID
swaps, drift-into-neighbor, multi-drone-only failures) plus the
figure-8 / BVC / thermal-coverage entries added with the new
verification stages.

For single-drone-specific symptoms, the diagnostic flow in
[hw_drone_verification.md §5](hw_drone_verification.md) is the entry
point: instrument with the flight_logger, look at where the
`/poses` ↔ `/cfN/odom` chain breaks, and triage from there.
