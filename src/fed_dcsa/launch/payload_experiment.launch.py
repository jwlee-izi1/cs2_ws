"""Launch the payload optimizer node with config YAML.

Assumes payload_hover.launch.py is already running.

Usage:
  ros2 launch fed_dcsa payload_experiment.launch.py
  ros2 launch fed_dcsa payload_experiment.launch.py dry_run:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('fed_dcsa')
    default_params = os.path.join(pkg_dir, 'config', 'payload_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to payload optimizer YAML config',
        ),
        DeclareLaunchArgument(
            'dry_run',
            default_value='false',
            description='If true, compute diagnostics and exit without sending commands',
        ),
        Node(
            package='fed_dcsa',
            executable='payload_optimizer',
            name='payload_optimizer',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'dry_run': LaunchConfiguration('dry_run')},
            ],
        ),
    ])
