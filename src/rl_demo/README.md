# rl_demo

Playback layer for 2D RL Crazyflie trajectories in Gazebo. Wraps shared
`ros_gz_crazyflie` packages without editing them — uses `GZ_SIM_RESOURCE_PATH`
prepend for a private unscaled model (visual scale=1; stock plugin gains).

## Phase A — tuning

Controller parameters locked via Phase A sweep. See
[exp1/tuning/summary.csv](../../exp1/tuning/summary.csv) for the full log.

Best stable regime with the stock Gazebo multicopter plugin:

| Parameter    | Value | Notes                              |
|--------------|-------|-------------------------------------|
| `kp_xy`      | 3.0   | Pushed beyond this, no gain       |
| `kd_xy`      | 0.3   | Damping for the outer loop         |
| `max_vel_xy` | 0.7   | Critical — outer loop saturates at 0.4 |
| `kp_z`, `kd_z`, `kp_yaw` | stock | Hover altitude static |
| Sim speed    | **0.3 m/s** | Above this the plugin tumbles the drone |

Tracking achieved: **max 11.2 cm, RMS 9.6 cm, leg-SS 8.7 cm**, no tumbles.

Gains apply at runtime via `SetParameters` on `/control_services_cf1`. The
shared launch does not thread gain parameters through, so runtime set is the
non-invasive path. Persisted in [config/controller_gains.yaml](config/controller_gains.yaml).

## Phase B — playback

### Launch the arena

```bash
ros2 launch rl_demo rl_demo.launch.py
```

Arena: 4×4 m walled, 5 cylindrical obstacles (r=0.20 m, hardcoded positions —
adjust in [worlds/rl_arena.sdf](worlds/rl_arena.sdf) to match your 2D env
layout), green goal marker disc. Top-down camera at z=4 m publishes on
`/camera/image` (30 Hz requested; actual framerate CPU-bound on no-GPU
systems).

Drone spawns at (-1.5, 0, 0.03) per [config/tuning_drones.yaml](config/tuning_drones.yaml).

### Play an NPZ

```bash
ros2 run rl_demo waypoint_player.py --npz /path/to/rollout.npz --tag demo1
```

NPZ must contain:
- `positions`: Nx2 float array of (x, y) in meters, world frame (ENU).
- `timestamps`: N float array in seconds, policy-native cadence (1 m/s scale).

Optional (currently ignored by the executor but recommended to preserve):
`goal_position`, `obstacles`, `arena_bounds`, `outcome`, `policy_id`.

The executor will:
1. Load NPZ, multiply timestamps by 3.333 (so 1 m/s policy → 0.3 m/s sim).
2. Apply the locked gains via `SetParameters`.
3. Takeoff to 0.5 m, reposition to trajectory start.
4. Fit cubic Hermite through each segment, upload via `upload_trajectory`,
   start via `start_trajectory`.
5. Log odom at sim-time, save CSV and plot under `~/cs2_ws/exp1/playback/`.

### Test without an NPZ

```bash
ros2 run rl_demo waypoint_player.py --synthetic --tag smoketest
```

Uses an internal S-shaped trajectory. Note: the synthetic path is NOT planned
around the arena obstacles — it will clip `obstacle_4` at (1.1, 0.6). Use
only to verify the pipeline, not tracking.

### Record video

The camera publishes on `/camera/image`. Simplest recording recipe:

```bash
# Terminal 1
ros2 launch rl_demo rl_demo.launch.py

# Terminal 2 — record bag while playback runs
ros2 bag record -o demo.bag /camera/image

# Terminal 3 — run playback, then Ctrl-C terminal 2
ros2 run rl_demo waypoint_player.py --npz rollout.npz
```

Then convert to MP4 (one option):

```bash
# Extract frames via a custom node that subscribes to the bag
# (see scripts/record_video.sh for a commented pipeline). Simplest:
ros2 bag play demo.bag &
# In another shell, subscribe & dump images with image_view or a tiny script.
# Finally:
ffmpeg -framerate 30 -i frames/frame%04d.jpg -c:v libx264 -pix_fmt yuv420p raw.mp4
ffmpeg -i raw.mp4 -filter:v 'setpts=0.3*PTS' -an demo_ff.mp4  # fast-forward 3.33x
```

The `setpts=0.3*PTS` step speeds the sim-slow playback back up to the
policy's native 1 m/s appearance.

## Structure

```
rl_demo/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── tuning_drones.yaml       # Single drone cf1, spawn at (-1.5, 0, 0.03)
│   └── controller_gains.yaml    # Locked Phase A gains
├── launch/
│   ├── tune.launch.py           # Phase A (stock world, tuning)
│   └── rl_demo.launch.py        # Phase B (arena + obstacles + camera)
├── models/crazyflie/
│   ├── model.sdf.in             # Private model: scale=1 visuals; stock plugin
│   ├── model.config
│   └── meshes/                  # Copied from shared package
├── worlds/
│   └── rl_arena.sdf             # 4×4 arena, 5 obstacles, top-down camera
└── scripts/
    ├── tune_tracking.py         # Phase A tuning harness
    ├── waypoint_player.py       # Phase B executor (NPZ → upload_trajectory)
    └── record_video.sh          # Recording recipe (documentation)
```

## Non-invasiveness

No edits to shared packages. The private model takes effect only when
`rl_demo.launch.py` or `tune.launch.py` prepends `GZ_SIM_RESOURCE_PATH`;
other projects (cf_payload_world, fed_dcsa, multinash_cs2_bridge) keep
using the shared stock model.
