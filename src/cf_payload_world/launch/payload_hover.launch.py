"""
payload_hover.launch.py
=======================
Launch the full payload hover stack: Gazebo world + ros_gz_bridge + control nodes.

Usage:
  ros2 launch cf_payload_world payload_hover.launch.py config:=level
  ros2 launch cf_payload_world payload_hover.launch.py config:=tilted

Gazebo launches paused. Unpause via the GUI play button.
"""

import os
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


DRONES = ['cf1', 'cf2', 'cf3', 'cf4']


def launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context)

    # Include the existing payload_world.launch.py (starts Gazebo only)
    payload_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cf_payload_world'),
                'launch', 'payload_world.launch.py',
            )
        ),
        launch_arguments={'config': config, 'paused': 'false'}.items(),
    )

    entities = [payload_world_launch]

    # Bridge entries
    bridge_entries = [
        {
            'ros_topic_name': '/clock',
            'gz_topic_name': '/clock',
            'ros_type_name': 'rosgraph_msgs/msg/Clock',
            'gz_type_name': 'gz.msgs.Clock',
            'direction': 'GZ_TO_ROS',
        },
    ]

    for name in DRONES:
        # Control services node
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

        # Bridge: cmd_vel (ROS -> Gazebo)
        bridge_entries.append({
            'ros_topic_name': f'/{name}/cmd_vel',
            'gz_topic_name': f'/{name}/gazebo/command/twist',
            'ros_type_name': 'geometry_msgs/msg/Twist',
            'gz_type_name': 'gz.msgs.Twist',
            'direction': 'ROS_TO_GZ',
        })
        # Bridge: odom (Gazebo -> ROS)
        bridge_entries.append({
            'ros_topic_name': f'/{name}/odom',
            'gz_topic_name': f'/model/{name}/odometry',
            'ros_type_name': 'nav_msgs/msg/Odometry',
            'gz_type_name': 'gz.msgs.Odometry',
            'direction': 'GZ_TO_ROS',
        })

    # Write bridge config to temp YAML
    bridge_yaml_path = os.path.join(
        tempfile.gettempdir(), 'cf_payload_bridge.yaml')
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


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='level',
            description='Drone height configuration: "level" or "tilted"',
        ),
        OpaqueFunction(function=launch_setup),
    ])
