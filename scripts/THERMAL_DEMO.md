# Thermal Mapping Demo — bringup skill

One-page reference for bringing up the multi-drone simulated thermal mapping pipeline on the CrazySim + Crazyswarm2 + Gazebo stack.

If you just want it to work, run:
```bash
~/cs2_ws/scripts/thermal_demo.sh up
```

If something doesn't work, the rest of this document is the troubleshooting guide.

---

## What this brings up

| Component | Process | Brings | Notes |
|---|---|---|---|
| **T1** Sim | `bash sitl_multiagent_square.sh` → `gz sim` + 4× `cf2-sitl:22.04` Docker containers | 4 simulated Crazyflies in a 1m square (cf1..cf4 at corners) | UDP 1985N ↔ 1995N per drone |
| **T2** Server | `ros2 launch crazyflie launch.py` | crazyswarm2 server, all `/cfN/{takeoff,go_to,...}` services, `/cfN/odom` (since logging is enabled in the YAML) | depends on T1 being fully up |
| **T3** Takeoff | `ros2 service call /all/takeoff` | drones to z=1.0m | one-shot, not a process |
| **T4** Thermal | `ros2 launch thermal_mapping thermal_mapping_demo.launch.py` | 4× `thermal_sensor_node` + 1× `thermal_mapper_node` + `rviz2` with GridMap display | depends on T2 + drones airborne |

The script runs them in order with waits between (`wait_until` polls until a condition is met before proceeding to the next stage).

## Subcommands

| Command | What it does |
|---|---|
| `thermal_demo.sh up` | Full bringup. Refuses if existing stack is detected — runs `down` first. |
| `thermal_demo.sh down` | Tears everything down (T4 → T2 → T1 → containers) by walking pgrep results explicitly. Plain `pkill` patterns missed double-launches in testing; this version is reliable. |
| `thermal_demo.sh restart` | `down` + `up`. |
| `thermal_demo.sh status` | Shows currently-running process per layer. |
| `thermal_demo.sh map` | Subscribes `/thermal_map` once, prints `finite/NaN` cell counts and temperature range. Use this to verify the pipeline is converging. |
| `thermal_demo.sh check` | Pre-flight without bringup. Verifies docker access, cflib import, ROS env, file paths, image presence, YAML config. Run this if `up` fails. |

## Flags (for `up`)

| Flag | Default | When to use |
|---|---|---|
| `--no-takeoff` | takeoff sent | If you want drones on the ground (e.g. testing thermal sensor while landed — but the sensor's `min_altitude=0.2` will skip readings, so map stays NaN). |
| `--no-rviz` | rviz launched | Headless runs (CI, no DISPLAY). |
| `--no-thermal` | thermal launched | If you only want sim + server (e.g. for an unrelated planner). |
| `--num-drones N` | 4 | 1-4 supported by the SITL square pattern; >4 needs `crazyflies_sitl_multi.yaml` extension. |

## Pre-flight (what `check` verifies)

1. `/opt/ros/jazzy/setup.bash` exists — i.e. `ros-jazzy-desktop` is installed.
2. `~/cs2_ws/install/setup.bash` exists — i.e. workspace is built (warn-only; mapper still launches).
3. `~/miniconda3/envs/crazyflie/bin/python3 -c "import cflib"` succeeds — cflib editable install is intact at `~/cs2_ws/cflib-src/`. **If reboot wiped `/tmp/cflib-src`**, see "cflib reinstall" below.
4. `docker ps` works for current user — daemon up, socket ACL'd. **If reboot reset the socket ACL**, see "docker socket ACL" below.
5. `sitl_multiagent_square.sh` and `crazyflies_sitl_multi.yaml` exist.
6. `crazyflies_sitl_multi.yaml` has `firmware_logging.enabled: true` with `default_topics.odom` — without this, `/cfN/odom` doesn't publish and the thermal sensor nodes have no pose source.
7. `cf2-sitl:22.04` Docker image is built — see [CRAZYSIM_MIGRATION.md](../CRAZYSIM_MIGRATION.md) §"One-time setup" step 3 if missing.

## Verification (after `up`)

```bash
# 1. Map is publishing (look for "10 messages over 5s", deltas ~0.5s, age <1s)
~/cs2_ws/scripts/thermal_demo.sh map
# Expect: "/thermal_map: <N> finite, <250000-N> NaN cells", range covering 21°C to ~50°C

# 2. Drones flying — drive over the configured hot spots
source /opt/ros/jazzy/setup.bash && source ~/cs2_ws/install/setup.bash
ros2 service call /cf1/go_to crazyflie_interfaces/srv/GoTo \
  "{group_mask: 0, relative: false, goal: {x: -1.0, y: -1.0, z: 1.0}, yaw: 0, duration: {sec: 4, nanosec: 0}}"
ros2 service call /cf2/go_to crazyflie_interfaces/srv/GoTo \
  "{group_mask: 0, relative: false, goal: {x:  1.5, y: -0.5, z: 1.0}, yaw: 0, duration: {sec: 4, nanosec: 0}}"

# 3. After ~10s of flying, sample the truth at the hot-spot cells:
sleep 10 && ~/cs2_ws/scripts/thermal_demo.sh map
# Expect peak ≈ 50°C (ambient 22 + (1.0,1.0) hot spot peak 28)
```

In RViz: GridMap display under "ThermalMap" should show three colored blobs at world `(1, 1)`, `(-1, -1)`, `(1.5, -0.5)`. Unvisited regions stay grey (NaN cells render transparent over the grey background).

## Customizing the demo

| Want to change... | Edit | Notes |
|---|---|---|
| Hot-spot locations / temperatures | [`src/thermal_mapping/config/thermal_field.yaml`](../src/thermal_mapping/config/thermal_field.yaml) | Edit and `thermal_demo.sh restart` (no rebuild needed) |
| Map extent / resolution | [`src/thermal_mapping/config/thermal_mapping_params.yaml`](../src/thermal_mapping/config/thermal_mapping_params.yaml) | `length_x`, `length_y`, `resolution`. Rebuild after changing. |
| Sensor FOV / rate / noise | [`src/thermal_mapping/launch/thermal_mapping_demo.launch.py`](../src/thermal_mapping/launch/thermal_mapping_demo.launch.py) | parameters dict per `thermal_sensor_node`. Rebuild after changing. |
| RViz colormap / min-max range | [`src/thermal_mapping/rviz/thermal_mapping.rviz`](../src/thermal_mapping/rviz/thermal_mapping.rviz) | `Min Intensity` / `Max Intensity` properties. No rebuild needed. |

After editing source files (params YAML, launch.py): `cd ~/cs2_ws && colcon build --packages-select thermal_mapping`, then `thermal_demo.sh restart`.

## Common failure modes & fixes

### `cflib not importable from .../crazyflie/bin/python3`
- **When:** reboot wiped `/tmp/cflib-src`.
- **Fix:** clone into `~/cs2_ws/cflib-src` (NOT `/tmp/`):
  ```bash
  cd ~/cs2_ws
  git clone --depth=1 https://github.com/bitcraze/crazyflie-lib-python.git cflib-src
  cd cflib-src
  ~/miniconda3/envs/crazyflie/bin/pip uninstall -y cflib
  SETUPTOOLS_SCM_PRETEND_VERSION=0.1.31 ~/miniconda3/envs/crazyflie/bin/pip install -e .
  ```

### `docker daemon unreachable for $USER`
- **When:** post-reboot. Docker daemon is enabled but the socket ACL isn't persistent (gotcha #2 in CRAZYSIM_MIGRATION.md).
- **Fix:**
  ```bash
  sudo systemctl start docker     # if not already running
  sudo setfacl -m u:$USER:rw /var/run/docker.sock
  ```

### `cf2-sitl:22.04 docker image not built`
- **When:** fresh machine, never built the image.
- **Fix:** see [CRAZYSIM_MIGRATION.md](../CRAZYSIM_MIGRATION.md) §"One-time setup" step 3.

### "All Crazyflies are fully connected!" but `/cfN/odom` is empty
- **Cause:** `firmware_logging.enabled: false` in the YAML. crazyswarm2 publishes `/poses` and `/tf` but not `/cfN/odom` unless logging is enabled.
- **Fix:** the bundled `config/crazyflies_sitl_multi.yaml` already has it enabled. If you forked it, check the `all.firmware_logging` block:
  ```yaml
  firmware_logging:
    enabled: true
    default_topics:
      odom:
        frequency: 20
  ```

### `cflib echoes 0xFF` works but full handshake hangs after a previous run
- **Cause:** gotcha #4 from CRAZYSIM_MIGRATION.md — Gazebo plugin's `socketInit` latches the first cf2's port and points at a dead one if you restart cf2 mid-run.
- **Fix:** `thermal_demo.sh down` then `thermal_demo.sh up`. The teardown stops Gazebo and the containers, which resets the latch.

### `BadParamException: This member is not been selected` on launch
- **Cause:** ABI mismatch between fastcdr/fastrtps and the rest of ROS — typically from a partial `apt install`.
- **Fix:** sync everything to the current versions:
  ```bash
  sudo apt update
  sudo apt install -y --only-upgrade $(dpkg -l | awk '/^ii  ros-jazzy-/ {print $2}')
  ```
  ⚠️ **Do NOT use `~nros-jazzy-` patterns** — apt's `--only-upgrade` with that selector REMOVES non-upgradable packages (verified on this exact machine). The `dpkg -l | awk` form is unambiguous and safe.

### Map stays all-NaN in RViz
- **Causes (in order of likelihood):**
  1. Drones at z < `min_altitude` (default 0.2m) — sensor skips readings during takeoff. Wait until takeoff completes.
  2. Sensor nodes can't subscribe to `/cfN/odom` — see "All Crazyflies fully connected but /cfN/odom empty" above.
  3. Drones flying outside the 5m × 5m map — they're at `(1,1)` etc. by default, well within bounds, but if you commanded `go_to` to e.g. `(10, 10)` the readings get masked.

## Architecture pointer

For the design discussion behind this pipeline, see the plan file at `/home/rk32226/.claude/plans/wait-can-we-get-unified-catmull.md` and the source under [`src/thermal_mapping/`](../src/thermal_mapping/) and [`src/thermal_mapping_interfaces/`](../src/thermal_mapping_interfaces/). The mapper math (column-major Eigen layout, weighted `np.bincount` scatter-add, NaN-as-unvisited convention) lives in [`src/thermal_mapping/thermal_mapping/grid_map_helpers.py`](../src/thermal_mapping/thermal_mapping/grid_map_helpers.py) and [`thermal_mapper_node.py`](../src/thermal_mapping/thermal_mapping/thermal_mapper_node.py).
