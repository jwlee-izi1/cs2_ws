# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import shutil
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def create_drone_model(name, model_sdf_path, models_dir):
    """Create a per-drone model directory with modified robotNamespace.

    The new directory is a sibling of the original crazyflie model so that
    relative mesh URIs (../../../meshes/...) resolve identically.
    """
    drone_dir = os.path.join(models_dir, name)
    os.makedirs(drone_dir, exist_ok=True)

    # Read the original model.sdf
    with open(model_sdf_path, 'r') as f:
        sdf = f.read()

    # Replace robotNamespace
    sdf = sdf.replace(
        '<robotNamespace>crazyflie</robotNamespace>',
        f'<robotNamespace>{name}</robotNamespace>'
    )

    # Replace lidar topic to be per-drone
    sdf = sdf.replace(
        '<topic>lidar</topic>',
        f'<topic>{name}/lidar</topic>'
    )

    # Write the modified model.sdf
    with open(os.path.join(drone_dir, 'model.sdf'), 'w') as f:
        f.write(sdf)

    # Write a minimal model.config
    with open(os.path.join(drone_dir, 'model.config'), 'w') as f:
        f.write(f"""<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
</model>
""")


def generate_world_sdf(base_world_path, drones):
    """Generate a world SDF that includes per-drone models."""
    with open(base_world_path, 'r') as f:
        world = f.read()

    # Build <include> blocks for each drone
    includes = ''
    for name, pos in drones:
        includes += f"""
    <include>
      <uri>model://{name}</uri>
      <name>{name}</name>
      <pose>{pos[0]} {pos[1]} {pos[2]} 0 0 0</pose>
    </include>
"""

    # Insert includes before the closing </world> tag
    world = world.replace('  </world>', includes + '  </world>')

    # Write to temp file
    world_path = os.path.join(tempfile.gettempdir(), 'crazyflie_multi_world.sdf')
    with open(world_path, 'w') as f:
        f.write(world)

    return world_path


def launch_multi_drones(context):
    """Parse crazyflies YAML and set up multi-drone simulation."""

    crazyflies_yaml_path = LaunchConfiguration('crazyflies_yaml_file').perform(context)
    with open(crazyflies_yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    robots = config.get('robots', {})

    # Find the model.sdf from GZ_SIM_RESOURCE_PATH
    gz_resource_path = os.getenv('GZ_SIM_RESOURCE_PATH', '')
    model_sdf_path = None
    models_dir = None
    for path in gz_resource_path.split(':'):
        candidate = os.path.join(path, 'crazyflie', 'model.sdf')
        if os.path.exists(candidate):
            model_sdf_path = candidate
            models_dir = path  # Parent dir containing model directories
            break

    if model_sdf_path is None:
        raise RuntimeError(
            'Could not find crazyflie model.sdf in GZ_SIM_RESOURCE_PATH. '
            f'GZ_SIM_RESOURCE_PATH={gz_resource_path}'
        )

    # Collect enabled drones
    drones = []
    for name, robot_cfg in robots.items():
        if not robot_cfg.get('enabled', False):
            continue
        pos = robot_cfg.get('initial_position', [0.0, 0.0, 0.0])
        drones.append((name, pos))

    # Create per-drone model directories alongside the original crazyflie model
    for name, _ in drones:
        create_drone_model(name, model_sdf_path, models_dir)

    # Generate world SDF with all drones included
    pkg_project_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    base_world_path = os.path.join(pkg_project_gazebo, 'worlds', 'crazyflie_world.sdf')
    world_path = generate_world_sdf(base_world_path, drones)

    # Build bridge config and control nodes
    entities = []
    bridge_entries = []

    # Clock bridge (shared)
    bridge_entries.append({
        'ros_topic_name': '/clock',
        'gz_topic_name': '/clock',
        'ros_type_name': 'rosgraph_msgs/msg/Clock',
        'gz_type_name': 'gz.msgs.Clock',
        'direction': 'GZ_TO_ROS',
    })

    for name, _ in drones:
        # Per-drone control services node
        control_node = Node(
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
        )
        entities.append(control_node)

        # Bridge entries for this drone
        bridge_entries.append({
            'ros_topic_name': f'/{name}/cmd_vel',
            'gz_topic_name': f'/{name}/gazebo/command/twist',
            'ros_type_name': 'geometry_msgs/msg/Twist',
            'gz_type_name': 'gz.msgs.Twist',
            'direction': 'ROS_TO_GZ',
        })
        bridge_entries.append({
            'ros_topic_name': f'/{name}/odom',
            'gz_topic_name': f'/model/{name}/odometry',
            'ros_type_name': 'nav_msgs/msg/Odometry',
            'gz_type_name': 'gz.msgs.Odometry',
            'direction': 'GZ_TO_ROS',
        })
        bridge_entries.append({
            'ros_topic_name': '/tf',
            'gz_topic_name': f'/model/{name}/pose',
            'ros_type_name': 'tf2_msgs/msg/TFMessage',
            'gz_type_name': 'gz.msgs.Pose_V',
            'direction': 'GZ_TO_ROS',
        })
        bridge_entries.append({
            'ros_topic_name': f'/{name}/scan',
            'gz_topic_name': f'/{name}/lidar',
            'ros_type_name': 'sensor_msgs/msg/LaserScan',
            'gz_type_name': 'gz.msgs.LaserScan',
            'direction': 'GZ_TO_ROS',
        })

    # Write bridge config
    bridge_yaml_path = os.path.join(
        tempfile.gettempdir(),
        'ros_gz_crazyflie_multi_bridge.yaml'
    )
    with open(bridge_yaml_path, 'w') as f:
        yaml.dump(bridge_entries, f, default_flow_style=False)

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{'config_file': bridge_yaml_path}, {'use_sim_time': True}],
        output='screen',
    )
    entities.append(bridge_node)

    return entities


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Default config path
    default_crazyflies_yaml = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.dirname(get_package_share_directory('ros_gz_crazyflie_bringup'))
        )))),
        'config', 'crazyflies_sim.yaml'
    )
    ws_config = os.path.join(os.path.expanduser('~'), 'cs2_ws', 'config', 'crazyflies_sim.yaml')
    if not os.path.exists(default_crazyflies_yaml) and os.path.exists(ws_config):
        default_crazyflies_yaml = ws_config

    return LaunchDescription([
        DeclareLaunchArgument(
            'crazyflies_yaml_file',
            default_value=default_crazyflies_yaml,
            description='Path to crazyflies YAML config',
        ),
        DeclareLaunchArgument(
            'gazebo_launch',
            default_value='True',
        ),
        # Gazebo world and drone spawning handled by OpaqueFunction
        OpaqueFunction(function=launch_with_world),
    ])


def launch_with_world(context):
    """Launch Gazebo with the generated world, then set up bridges and control."""
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    crazyflies_yaml_path = LaunchConfiguration('crazyflies_yaml_file').perform(context)
    with open(crazyflies_yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    robots = config.get('robots', {})

    # Find model.sdf in GZ_SIM_RESOURCE_PATH
    gz_resource_path = os.getenv('GZ_SIM_RESOURCE_PATH', '')
    model_sdf_path = None
    models_dir = None
    for path in gz_resource_path.split(':'):
        candidate = os.path.join(path, 'crazyflie', 'model.sdf')
        if os.path.exists(candidate):
            model_sdf_path = candidate
            models_dir = path
            break

    if model_sdf_path is None:
        raise RuntimeError(
            'Could not find crazyflie model.sdf in GZ_SIM_RESOURCE_PATH. '
            f'GZ_SIM_RESOURCE_PATH={gz_resource_path}'
        )

    # Collect enabled drones
    drones = []
    for name, robot_cfg in robots.items():
        if not robot_cfg.get('enabled', False):
            continue
        pos = robot_cfg.get('initial_position', [0.0, 0.0, 0.0])
        drones.append((name, pos))

    # Create per-drone model directories (siblings of crazyflie/ in GZ_SIM_RESOURCE_PATH)
    for name, _ in drones:
        create_drone_model(name, model_sdf_path, models_dir)

    # Generate world SDF with drone includes
    pkg_project_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    base_world_path = os.path.join(pkg_project_gazebo, 'worlds', 'crazyflie_world.sdf')
    world_path = generate_world_sdf(base_world_path, drones)

    entities = []

    # Launch Gazebo with generated world
    gazebo_launch = LaunchConfiguration('gazebo_launch').perform(context)
    if gazebo_launch.lower() == 'true':
        gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'{world_path} -r'}.items(),
        )
        entities.append(gz_sim)

    # Bridge and control setup
    bridge_entries = [{
        'ros_topic_name': '/clock',
        'gz_topic_name': '/clock',
        'ros_type_name': 'rosgraph_msgs/msg/Clock',
        'gz_type_name': 'gz.msgs.Clock',
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
            {
                'ros_topic_name': f'/{name}/cmd_vel',
                'gz_topic_name': f'/{name}/gazebo/command/twist',
                'ros_type_name': 'geometry_msgs/msg/Twist',
                'gz_type_name': 'gz.msgs.Twist',
                'direction': 'ROS_TO_GZ',
            },
            {
                'ros_topic_name': f'/{name}/odom',
                'gz_topic_name': f'/model/{name}/odometry',
                'ros_type_name': 'nav_msgs/msg/Odometry',
                'gz_type_name': 'gz.msgs.Odometry',
                'direction': 'GZ_TO_ROS',
            },
            {
                'ros_topic_name': '/tf',
                'gz_topic_name': f'/model/{name}/pose',
                'ros_type_name': 'tf2_msgs/msg/TFMessage',
                'gz_type_name': 'gz.msgs.Pose_V',
                'direction': 'GZ_TO_ROS',
            },
            {
                'ros_topic_name': f'/{name}/scan',
                'gz_topic_name': f'/{name}/lidar',
                'ros_type_name': 'sensor_msgs/msg/LaserScan',
                'gz_type_name': 'gz.msgs.LaserScan',
                'direction': 'GZ_TO_ROS',
            },
        ])

    bridge_yaml_path = os.path.join(tempfile.gettempdir(), 'ros_gz_crazyflie_multi_bridge.yaml')
    with open(bridge_yaml_path, 'w') as f:
        yaml.dump(bridge_entries, f, default_flow_style=False)

    entities.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{'config_file': bridge_yaml_path}, {'use_sim_time': True}],
        output='screen',
    ))

    return entities
