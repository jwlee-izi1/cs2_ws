# Thermal Mapping Demo

## Goal

A multi-drone shared-occupancy heatmap demo. Each drone synthesises a 16×16 downward-facing
"thermal sensor" frame from its current pose over a configured ground-truth thermal
field. A central `thermal_mapper_node` publishes a unified `grid_map_msgs/GridMap`
on `/thermal_map`, which RViz renders.

The demo serves two purposes:

1. **Standalone demonstration** of multi-drone shared mapping using crazyswarm2's
   `/cfN/odom` as the only pose source — sim/hw agnostic.
2. **Data plane** for the federated coverage project (see
   [federated_coverage.md](federated_coverage.md)) — drones generate thermal readings
   as they sweep their sectors; the resulting map shows coverage.

**Two field modes** (configured in `thermal_field.yaml`):

- **Static** — sum of Gaussian hot spots over an ambient temperature. The legacy
  demo mode. Mapper accumulates the running mean across drones.
- **Dynamic fire (M4 rotating-wind wavefront + burn profile)** — used by
  federated_coverage as the substrate the optimizer (and later RL policy)
  chases. The mapper switches to last-write-wins fusion with `age_seconds` +
  `last_update_time` staleness layers. See
  [federated_coverage.md → Dynamic fire substrate](federated_coverage.md#dynamic-fire-substrate)
  for the model, the oracle $q_i$ estimator, and the locked YAML.

The simulated thermal sensor is intentionally generic: it could be any downward-facing
2D sensor (camera, occupancy grid). The pattern shown here generalises.

## Packages

| Package | Role |
|---|---|
| `src/thermal_mapping` | `thermal_sensor_node` (per drone, generates ThermalFrame from odom + field config) + `thermal_mapper_node` (central, accumulates mean per cell, publishes `/thermal_map`). Pure-Python helpers in `thermal_field.py`, `grid_map_helpers.py`. |
| `src/thermal_mapping_interfaces` | `ThermalFrame.msg` — the per-drone observation message. |

## Architecture / topic contracts

```
crazyswarm2 ── /cfN/odom (nav_msgs/Odometry) ──→ thermal_sensor_node (one per drone)
                                                       │
                                                       │ /cfN/thermal_frame
                                                       │ (ThermalFrame.msg)
                                                       ▼
                                              thermal_mapper_node ── /thermal_map ──→ RViz GridMap
                                              (single instance, all 4)   (grid_map_msgs/GridMap, 2 Hz)
```

**ThermalFrame.msg fields:** `Header header`, `geometry_msgs/Pose pose` (drone pose at
capture time — embedded so mapper doesn't need to time-sync against odom),
`float32 footprint_size` (ground footprint side, derived from FOV × altitude),
`uint16 width`, `uint16 height` (pixel grid; both = 16), `float32[] data`
(`width*height` temperatures in °C).

**Map config** (in `src/thermal_mapping/config/thermal_mapping_params.yaml`):
- `length_x: 5.0`, `length_y: 5.0` — 5 m × 5 m map centered at origin (matches arena).
- `resolution: 0.01` — 1 cm cells → 500×500 grid.
- `publish_rate_hz: 2.0` — mapper publishes GridMap at 2 Hz.

**Field config** (in `src/thermal_mapping/config/thermal_field.yaml`):
- `ambient` temperature + a list of `hot_spots` (Gaussian peaks: x, y, peak, radius).

## Status

| Feature | State | Notes |
|---|---|---|
| Sim thermal sensor (per drone) | ✅ | 16×16 pixels, configurable FOV (default 45° in standalone, 15° when used by federated_coverage), gates publishing below `min_altitude=0.2 m`, optional Gaussian noise. |
| Mapper (per-cell mean) | ✅ | 500×500 grid, 2 Hz publish, NaN initialisation, mean updated per observation. |
| RViz visualisation | ✅ | `grid_map_rviz_plugin/GridMap` display with rainbow colormap. |
| End-to-end verification | ✅ | All 3 configured hot spots read truth values within noise tolerance after a sweep. |
| One-command bringup | ✅ | `~/cs2_ws/scripts/thermal_demo.sh up`. Subcommands: `down`, `restart`, `status`, `map`, `check`. |
| Pre-flight (`check` subcommand) | ✅ | Verifies docker, cflib, ROS env, image, YAML config; fails fast with clear errors. |
| **Hardware bringup** | ✅ partial | First HW flight (cf4) completed 2026-05-17: autonomous takeoff + hover + land via mocap. Single-drone thermal pipeline E2E verified 2026-05-18 (cf1 sim flew over hot spot at (1.5,-0.5), map shows 33.89°C peak matching synthetic field). Still ⏳: 4-drone hardware scale-up. See [thermal_mapping_hw_handoff.md](thermal_mapping_hw_handoff.md). |
| **Dynamic field (for future RL)** | ❌ | Field is static. When the field becomes dynamic (e.g. spreading fire), expose a per-cell staleness map from the mapper so RL can prioritise stale regions. |

## Next steps

This subsystem is the **recommended infra smoke test** for hardware bringup of the
whole workspace — see [CRAZYSIM_MIGRATION.md §3.1](../CRAZYSIM_MIGRATION.md). Running

```bash
CFLIES_YAML=$HOME/cs2_ws/config/crazyflies_hw.yaml ~/cs2_ws/scripts/thermal_demo.sh up
```

is the smallest possible pipeline that exercises radio + positioning + real EKF on
`/cfN/odom` + a multi-drone aggregating ROS node (the mapper). If this works, the
infra is fine and you can proceed to layer application stacks on top (e.g. the
[federated coverage](federated_coverage.md) project's `coverage_restart.sh`).

What does NOT change on hardware:
- `thermal_sensor_node` keeps simulating the thermal field over real drone positions
  (no real thermal sensor on CF2.1; this demo is about the architecture, not real
  thermal perception).
- `thermal_mapper_node` is unchanged.
- `thermal_field.yaml` is unchanged (synthetic hot spots).

Future: dynamic thermal field + per-cell staleness map will be needed when the
federated coverage project enters its RL phase. Defer until then.

## How to run

**Standalone bringup** (sim → sensors → mapper → RViz):

```bash
~/cs2_ws/scripts/thermal_demo.sh up
```

**Useful subcommands** (full reference in [`scripts/THERMAL_DEMO.md`](../scripts/THERMAL_DEMO.md)):

| Command | What it does |
|---|---|
| `thermal_demo.sh up` | Full bringup. Refuses if a previous stack is detected; runs `down` first. |
| `thermal_demo.sh up --no-thermal` | Sim + crazyswarm2 only (no sensors/mapper). Useful when you want to launch a different downstream stack (e.g. `coverage_restart.sh` uses this internally). |
| `thermal_demo.sh down` | Teardown — walks pgrep results explicitly. Plain `pkill` missed double-launches in testing; this version is reliable. |
| `thermal_demo.sh status` | What's running. |
| `thermal_demo.sh map` | One-shot subscribe to `/thermal_map`; prints finite-cell count + temp range. Use to verify convergence. |
| `thermal_demo.sh check` | Pre-flight without bringup. Verifies docker, cflib, ROS env, image, YAML. Run if `up` fails. |

**Configuration:**
- `src/thermal_mapping/config/thermal_mapping_params.yaml` — sensor + mapper params.
- `src/thermal_mapping/config/thermal_field.yaml` — ambient temperature + hot spots.
- `config/crazyflies_sitl_multi.yaml` — drone setup (must have `firmware_logging.odom`
  enabled so `/cfN/odom` publishes).

For deep troubleshooting (reboot recovery, daemon cache, container restart patterns),
see [`scripts/THERMAL_DEMO.md`](../scripts/THERMAL_DEMO.md).
