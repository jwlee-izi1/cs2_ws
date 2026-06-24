"""Phase B demo launch: Crazyflie in the rl_arena world with obstacles.

The shared ros_gz_crazyflie_bringup launch hardcodes its world path with no
override argument, so this file reimplements the core spawn flow (same
create_drone_model + generate_world_sdf helpers, re-expressed here) pointing
at rl_demo's arena. Shared packages are not edited.

Runs:
    ros2 launch rl_demo rl_demo.launch.py

Then:
    ros2 run rl_demo waypoint_player.py --npz /path/to/trajectory.npz
"""

import os
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def create_drone_model(name, model_sdf_path, models_dir):
    """Copy the private crazyflie model into a per-drone sibling dir,
    rewriting <robotNamespace> and the lidar topic. Mirrors the shared
    bringup's helper of the same name."""
    drone_dir = os.path.join(models_dir, name)
    os.makedirs(drone_dir, exist_ok=True)
    with open(model_sdf_path, 'r') as f:
        sdf = f.read()
    sdf = sdf.replace('<robotNamespace>crazyflie</robotNamespace>',
                      f'<robotNamespace>{name}</robotNamespace>')
    sdf = sdf.replace('<topic>lidar</topic>', f'<topic>{name}/lidar</topic>')
    with open(os.path.join(drone_dir, 'model.sdf'), 'w') as f:
        f.write(sdf)
    with open(os.path.join(drone_dir, 'model.config'), 'w') as f:
        f.write(f'<?xml version="1.0"?>\n<model>\n  <name>{name}</name>\n'
                f'  <version>1.0</version>\n  <sdf version="1.8">model.sdf</sdf>\n</model>\n')


def generate_world_sdf(base_world_path, drones, out_path):
    """Inject per-drone <include> tags into the arena world."""
    with open(base_world_path, 'r') as f:
        world = f.read()
    includes = ''
    for name, pos in drones:
        includes += (
            f'\n    <include>\n'
            f'      <uri>model://{name}</uri>\n'
            f'      <name>{name}</name>\n'
            f'      <pose>{pos[0]} {pos[1]} {pos[2]} 0 0 0</pose>\n'
            f'    </include>\n'
        )
    world = world.replace('  </world>', includes + '  </world>')
    with open(out_path, 'w') as f:
        f.write(world)


def launch_setup(context):
    rl_demo_share = get_package_share_directory('rl_demo')
    models_dir = os.path.join(rl_demo_share, 'models')
    base_world = LaunchConfiguration('world').perform(context)
    if not base_world:
        base_world = os.path.join(rl_demo_share, 'worlds', 'rl_arena.sdf')
    yaml_path = LaunchConfiguration('crazyflies_yaml_file').perform(context)

    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    drones = []
    for name, cfg in config.get('robots', {}).items():
        if not cfg.get('enabled', False):
            continue
        drones.append((name, cfg.get('initial_position', [0.0, 0.0, 0.03])))
    if not drones:
        raise RuntimeError(f'No enabled drones in {yaml_path}')

    # Pick the private crazyflie/model.sdf (our override with scale=1 visuals).
    model_sdf = os.path.join(models_dir, 'crazyflie', 'model.sdf')
    if not os.path.exists(model_sdf):
        raise RuntimeError(f'Expected private model at {model_sdf}. Rebuild rl_demo.')

    for name, _ in drones:
        create_drone_model(name, model_sdf, models_dir)

    world_out = os.path.join(tempfile.gettempdir(), 'rl_demo_world.sdf')
    generate_world_sdf(base_world, drones, world_out)

    entities = []

    # Gazebo
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    entities.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world_out} -r'}.items(),
    ))

    # Bridge config (clock + per-drone topics + camera image)
    bridge_entries = [{
        'ros_topic_name': '/clock',
        'gz_topic_name': '/clock',
        'ros_type_name': 'rosgraph_msgs/msg/Clock',
        'gz_type_name': 'gz.msgs.Clock',
        'direction': 'GZ_TO_ROS',
    }, {
        'ros_topic_name': '/camera/image',
        'gz_topic_name': '/camera/image',
        'ros_type_name': 'sensor_msgs/msg/Image',
        'gz_type_name': 'gz.msgs.Image',
        'direction': 'GZ_TO_ROS',
    }]

    for name, _ in drones:
        entities.append(Node(
            package='ros_gz_crazyflie_control',
            executable='control_services',
            name=f'control_services_{name}',
            output='screen',
            parameters=[
                {'hover_height': 0.5},
                {'robot_prefix': f'/{name}'},
                {'incoming_twist_topic': '/cmd_vel_teleop'},
                {'max_ang_z_rate': 0.4},
                {'use_sim_time': True},
            ],
        ))
        bridge_entries.extend([
            {'ros_topic_name': f'/{name}/cmd_vel',
             'gz_topic_name': f'/{name}/gazebo/command/twist',
             'ros_type_name': 'geometry_msgs/msg/Twist',
             'gz_type_name': 'gz.msgs.Twist',
             'direction': 'ROS_TO_GZ'},
            {'ros_topic_name': f'/{name}/odom',
             'gz_topic_name': f'/model/{name}/odometry',
             'ros_type_name': 'nav_msgs/msg/Odometry',
             'gz_type_name': 'gz.msgs.Odometry',
             'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': '/tf',
             'gz_topic_name': f'/model/{name}/pose',
             'ros_type_name': 'tf2_msgs/msg/TFMessage',
             'gz_type_name': 'gz.msgs.Pose_V',
             'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': f'/{name}/scan',
             'gz_topic_name': f'/{name}/lidar',
             'ros_type_name': 'sensor_msgs/msg/LaserScan',
             'gz_type_name': 'gz.msgs.LaserScan',
             'direction': 'GZ_TO_ROS'},
        ])

    bridge_yaml = os.path.join(tempfile.gettempdir(), 'rl_demo_bridge.yaml')
    with open(bridge_yaml, 'w') as f:
        yaml.dump(bridge_entries, f, default_flow_style=False)

    entities.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{'config_file': bridge_yaml}, {'use_sim_time': True}],
        output='screen',
    ))

    return entities


def generate_launch_description():
    rl_demo_share = get_package_share_directory('rl_demo')
    default_yaml = os.path.join(rl_demo_share, 'config', 'tuning_drones.yaml')
    models_dir = os.path.join(rl_demo_share, 'models')

    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_path = models_dir + (':' + existing if existing else '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = new_path

    return LaunchDescription([
        DeclareLaunchArgument('crazyflies_yaml_file', default_value=default_yaml),
        DeclareLaunchArgument('world', default_value='',
                              description='Override arena SDF path (e.g. /tmp/rl_arena_current.sdf from prepare_arena.py)'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_path),
        OpaqueFunction(function=launch_setup),
    ])
