"""
payload_world.launch.py
=======================
Launch the cf_payload_world Gazebo simulation.

Usage:
  ros2 launch cf_payload_world payload_world.launch.py config:=level
  ros2 launch cf_payload_world payload_world.launch.py config:=tilted

The 'config' argument selects initial drone hover z-heights.  All other
geometry (rod orientations, payload pose, propeller positions) is derived
automatically.  The world launches PAUSED so you can inspect the initial
configuration before starting physics.

Adding a new configuration
--------------------------
Add an entry to CONFIGS with four drone z-heights:

    CONFIGS['my_config'] = {'cf1': 1.25, 'cf2': 1.30, 'cf3': 1.25, 'cf4': 1.30}

Then launch with  config:=my_config.
"""

import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# ---------------------------------------------------------------------------
# Fixed geometry
# ---------------------------------------------------------------------------

# Drone XY positions in world frame (fixed across all configs)
#   cf1: +X +Y,  cf2: −X +Y,  cf3: −X −Y,  cf4: +X −Y
DRONE_XY = {
    'cf1': (+0.40, +0.24),
    'cf2': (-0.40, +0.24),
    'cf3': (-0.40, -0.24),
    'cf4': (+0.40, -0.24),
}

# Payload top-face corner XY positions (payload half-extents: 0.20 × 0.12)
CORNER_XY = {
    'cf1': (+0.20, +0.12),
    'cf2': (-0.20, +0.12),
    'cf3': (-0.20, -0.12),
    'cf4': (+0.20, -0.12),
}

DELTA_Z        = 0.44    # vertical offset: corner → drone body origin (m)
LINK_LEN       = 0.498   # rod length (m) — sqrt(0.20²+0.12²+0.44²)
PAYLOAD_HALF_Z = 0.01    # half-thickness of payload box (m)

# Propeller offsets relative to drone body origin (from crazyflie/model.sdf)
PROP_OFFSETS = [
    (+0.031, -0.031, +0.021),  # m1  ccw
    (-0.031, -0.031, +0.021),  # m2  cw
    (-0.031, +0.031, +0.021),  # m3  ccw
    (+0.031, +0.031, +0.021),  # m4  cw
]

# ---------------------------------------------------------------------------
# Known configurations  (drone z-heights in metres)
# ---------------------------------------------------------------------------

CONFIGS = {
    # All drones at the same height — payload hangs level.
    'level': {
        'cf1': 1.25,
        'cf2': 1.25,
        'cf3': 1.25,
        'cf4': 1.25,
    },
    # cf1/cf4 (+X side) lower, cf2/cf3 (−X side) higher → ~15cm tilt.
    'tilted': {
        'cf1': 1.175,
        'cf2': 1.325,
        'cf3': 1.325,
        'cf4': 1.175,
    },
}

# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _fmt(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    """Format a 6-DOF pose string for SDF."""
    return f'{x:.6f} {y:.6f} {z:.6f} {roll:.6f} {pitch:.6f} {yaw:.6f}'


def compute_poses(drone_z):
    """
    Return a dict  {PLACEHOLDER_NAME: pose_string}  for all 25 placeholders
    in payload_world.sdf, derived from the four drone z-heights in drone_z.

    Parameters
    ----------
    drone_z : dict  {'cf1': float, 'cf2': float, 'cf3': float, 'cf4': float}
        World-frame z coordinate of each drone body origin.
    """
    poses = {}
    drones = ['cf1', 'cf2', 'cf3', 'cf4']

    # -- Corner positions (rod bottom, payload top-face corner) ---------------
    corners = {}
    for name in drones:
        cx, cy = CORNER_XY[name]
        cz = drone_z[name] - DELTA_Z
        corners[name] = (cx, cy, cz)

    # -- Payload pose ---------------------------------------------------------
    all_cz = [corners[n][2] for n in drones]
    mean_cz = sum(all_cz) / 4
    payload_z = mean_cz - PAYLOAD_HALF_Z   # centre of payload box

    # Tilt about world Y from X-asymmetry (−X side minus +X side)
    pos_x_cz = (corners['cf1'][2] + corners['cf4'][2]) / 2   # +X corners mean z
    neg_x_cz = (corners['cf2'][2] + corners['cf3'][2]) / 2   # −X corners mean z
    payload_pitch = math.atan2(neg_x_cz - pos_x_cz, 0.40)

    # Tilt about world X from Y-asymmetry (fully general)
    pos_y_cz = (corners['cf1'][2] + corners['cf2'][2]) / 2   # +Y corners mean z
    neg_y_cz = (corners['cf3'][2] + corners['cf4'][2]) / 2   # −Y corners mean z
    payload_roll = math.atan2(pos_y_cz - neg_y_cz, 0.24)

    poses['PAYLOAD_POSE'] = _fmt(0.0, 0.0, payload_z, payload_roll, payload_pitch, 0.0)

    # -- Rod poses ------------------------------------------------------------
    # Each rod origin sits at its corner C_i.  The rod's local +Z axis is
    # aligned with the direction C_i → D_i.
    #
    # SDF RPY convention: final orientation = Rz(yaw) · Ry(pitch) · Rx(roll)
    # Starting from local Z = (0,0,1):
    #   after Ry(pitch): Z → (sin p, 0, cos p)
    #   after Rz(yaw):   Z → (sin p · cos y, sin p · sin y, cos p)
    # so  pitch = acos(Δz / L),  yaw = atan2(Δy, Δx)
    for i, name in enumerate(drones, 1):
        dx = DRONE_XY[name][0] - CORNER_XY[name][0]   # ±0.20
        dy = DRONE_XY[name][1] - CORNER_XY[name][1]   # ±0.12
        dz = DELTA_Z                                    #  0.44
        L  = math.sqrt(dx**2 + dy**2 + dz**2)          # ≈ 0.498

        rod_pitch = math.acos(dz / L)
        rod_yaw   = math.atan2(dy, dx)

        cx, cy, cz = corners[name]
        poses[f'ROD{i}_POSE'] = _fmt(cx, cy, cz, 0.0, rod_pitch, rod_yaw)

    # -- Drone body poses -----------------------------------------------------
    for i, name in enumerate(drones, 1):
        dx, dy = DRONE_XY[name]
        dz = drone_z[name]
        poses[f'CF{i}_POSE'] = _fmt(dx, dy, dz)

    # Propeller poses are now hardcoded as relative offsets in the SDF
    # (inside nested <model> elements), so no CF*_M*_POSE placeholders needed.

    return poses


# ---------------------------------------------------------------------------
# Launch setup (runs at launch time so config arg is available)
# ---------------------------------------------------------------------------

def launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context)

    if config not in CONFIGS:
        raise ValueError(
            f"Unknown config '{config}'. "
            f"Available: {sorted(CONFIGS.keys())}"
        )

    drone_z = CONFIGS[config]
    poses   = compute_poses(drone_z)

    # Read the SDF template
    world_share   = get_package_share_directory('cf_payload_world')
    template_path = os.path.join(world_share, 'worlds', 'payload_world.sdf')
    with open(template_path) as fh:
        sdf_text = fh.read()

    # Fill every placeholder
    for key, value in poses.items():
        sdf_text = sdf_text.replace('{' + key + '}', value)

    # Warn if any placeholder was missed
    import re
    remaining = re.findall(r'\{[A-Z0-9_]+\}', sdf_text)
    if remaining:
        print(f'[payload_world] WARNING: unfilled placeholders: {remaining}')

    # Write filled SDF to a temp file (Gazebo needs a path, not stdin)
    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        suffix=f'_cf_payload_{config}.sdf',
        delete=False,
        prefix='/tmp/',
    )
    tmp.write(sdf_text)
    tmp.flush()
    tmp.close()
    sdf_path = tmp.name
    print(f'[payload_world] Filled SDF written to: {sdf_path}')

    # Set GZ_SIM_RESOURCE_PATH so model://crazyflie/meshes/ URIs resolve.
    # ros_gz_crazyflie_gazebo doesn't install its models/ dir, so we locate the
    # workspace root from AMENT_PREFIX_PATH and point at the source tree.
    models_dir = ''
    ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    if ament_prefix:
        # AMENT_PREFIX_PATH entries: /path/to/ws/install/<pkg>  → ws root is 2 levels up
        first_entry = ament_prefix.split(':')[0]
        ws_root = os.path.dirname(os.path.dirname(first_entry))
        candidate = os.path.join(ws_root, 'src', 'ros_gz_crazyflie',
                                 'ros_gz_crazyflie_gazebo', 'models')
        if os.path.isdir(candidate):
            models_dir = candidate
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if models_dir and models_dir not in existing:
        os.environ['GZ_SIM_RESOURCE_PATH'] = (
            models_dir + ':' + existing if existing else models_dir
        )
    print(f'[payload_world] GZ_SIM_RESOURCE_PATH = {os.environ["GZ_SIM_RESOURCE_PATH"]}')

    # gz sim 8 starts paused by default; pass -r to run immediately.
    paused = LaunchConfiguration('paused').perform(context)
    gz_cmd = ['gz', 'sim', sdf_path]
    if paused.lower() == 'false':
        gz_cmd.insert(2, '-r')

    return [
        ExecuteProcess(
            cmd=gz_cmd,
            output='screen',
            additional_env={'GZ_SIM_RESOURCE_PATH': os.environ['GZ_SIM_RESOURCE_PATH']},
        )
    ]


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='level',
            description=(
                'Initial drone configuration.  '
                '"level" — all drones at 1.25 m, payload flat.  '
                '"tilted" — cf1/cf4 at 1.181 m, cf2/cf3 at 1.318 m, payload ~19° tilt.'
            ),
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='true',
            description='Start Gazebo paused (true) or running (false).',
        ),
        OpaqueFunction(function=launch_setup),
    ])
