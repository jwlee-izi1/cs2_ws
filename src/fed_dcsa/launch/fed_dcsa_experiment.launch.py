"""Launch the Fed-DCSA coordinator node.

Assumes the Gazebo simulation is already running via:
    ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py

This launch file only adds the coordinator node on top.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('fed_dcsa')
    default_params = os.path.join(pkg_share, 'config', 'fed_dcsa_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to Fed-DCSA parameter YAML file',
        ),
        Node(
            package='fed_dcsa',
            executable='fed_dcsa_coordinator',
            name='fed_dcsa',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': True},
            ],
        ),
    ])
