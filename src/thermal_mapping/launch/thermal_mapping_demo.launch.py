"""Spawn N thermal_sensor_node instances + the central thermal_mapper_node + RViz.

Assumes the multi-drone CrazySim stack and crazyswarm2 server are already
running in other terminals (see CRAZYSIM_MIGRATION.md). This launch file only
adds the thermal mapping pipeline on top.

Usage:
    # Default 4-drone sim demo: spawns thermal sensors for cf1..cf4
    ros2 launch thermal_mapping thermal_mapping_demo.launch.py
    # Sim with fewer sequential drones
    ros2 launch thermal_mapping thermal_mapping_demo.launch.py num_drones:=2
    # Hardware single-drone (or any non-sequential set), explicit names override
    # (comma-separated, no quotes/brackets):
    ros2 launch thermal_mapping thermal_mapping_demo.launch.py drone_names:=cf3
    ros2 launch thermal_mapping thermal_mapping_demo.launch.py drone_names:=cf1,cf3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nodes(context, *args, **kwargs):
    pkg_share = get_package_share_directory('thermal_mapping')
    field_yaml = LaunchConfiguration('field_yaml').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    fov_deg = float(LaunchConfiguration('fov_deg').perform(context))
    arena_yaml = LaunchConfiguration('arena_yaml').perform(context)
    qi_rate_hz = float(LaunchConfiguration('qi_rate_hz').perform(context))
    enable_qi_estimator = LaunchConfiguration('enable_qi_estimator').perform(
        context
    ).lower() in ('true', '1', 'yes')
    enable_ground_truth = LaunchConfiguration('enable_ground_truth').perform(
        context
    ).lower() in ('true', '1', 'yes')

    # drone_names override (comma-separated, e.g. 'cf3' or 'cf1,cf3') wins;
    # else derive from num_drones
    drone_names_arg = LaunchConfiguration('drone_names').perform(context).strip()
    if drone_names_arg:
        drone_names = [n.strip() for n in drone_names_arg.split(',') if n.strip()]
        if not drone_names:
            raise ValueError(f"drone_names is empty after parsing: {drone_names_arg!r}")
    else:
        num_drones = int(LaunchConfiguration('num_drones').perform(context))
        drone_names = [f'cf{i + 1}' for i in range(num_drones)]

    nodes = []
    for name in drone_names:
        nodes.append(Node(
            package='thermal_mapping',
            executable='thermal_sensor_node',
            name=f'thermal_sensor_{name}',
            output='screen',
            parameters=[
                params_file,
                {'drone_name': name,
                 'field_yaml': field_yaml,
                 'fov_deg': fov_deg,
                 'resolution': 16,
                 'rate_hz': 5.0,
                 'min_altitude': 0.2,
                 'noise_sigma': 0.5,
                 'use_sim_time': False},
            ],
        ))

    nodes.append(Node(
        package='thermal_mapping',
        executable='thermal_mapper_node',
        name='thermal_mapper',
        output='screen',
        parameters=[
            params_file,
            {'drone_names': drone_names, 'use_sim_time': False},
        ],
    ))

    if enable_ground_truth:
        nodes.append(Node(
            package='thermal_mapping',
            executable='thermal_ground_truth_node',
            name='thermal_ground_truth',
            output='screen',
            parameters=[
                params_file,
                {'field_yaml': field_yaml, 'use_sim_time': False},
            ],
        ))

    if enable_qi_estimator:
        if not arena_yaml:
            raise RuntimeError(
                'enable_qi_estimator=true but arena_yaml is empty. '
                'Pass arena_yaml:=<path-to-arena_4drone.yaml>.'
            )
        nodes.append(Node(
            package='thermal_mapping',
            executable='qi_estimator_node',
            name='qi_estimator',
            output='screen',
            parameters=[
                {'field_yaml': field_yaml,
                 'arena_yaml': arena_yaml,
                 'rate_hz': qi_rate_hz,
                 'use_sim_time': False},
            ],
        ))

    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        name='thermal_mapping_rviz',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': False}],
    ))

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('thermal_mapping')
    default_field = os.path.join(pkg_share, 'config', 'thermal_field.yaml')
    default_params = os.path.join(pkg_share, 'config', 'thermal_mapping_params.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'thermal_mapping.rviz')

    try:
        default_arena = os.path.join(
            get_package_share_directory('cf_coverage_planner'),
            'config',
            'arena_4drone.yaml',
        )
    except Exception:
        default_arena = ''

    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='4'),
        DeclareLaunchArgument('drone_names', default_value='',
            description="Optional explicit drone names list, e.g. '[cf3]'. "
                        "Overrides num_drones-derived cf1..cfN sequence if non-empty."),
        DeclareLaunchArgument('field_yaml', default_value=default_field),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('fov_deg', default_value='45.0',
            description="Per-drone sensor FOV in degrees. Default 45 preserves "
                        "legacy single-drone behaviour. Pass fov_deg:=35 for the "
                        "Stage 4 multi-drone demo (matches arena_4drone.yaml)."),
        DeclareLaunchArgument('arena_yaml', default_value=default_arena,
            description="Path to arena_4drone.yaml — sector geometry used by the "
                        "q_i estimator. Defaults to the installed cf_coverage_planner "
                        "config if that package is built."),
        DeclareLaunchArgument('enable_qi_estimator', default_value='true',
            description="If true, spawn qi_estimator_node and publish "
                        "/coverage/sector_weights for the FedDCSA optimizer."),
        DeclareLaunchArgument('enable_ground_truth', default_value='true',
            description="If true, spawn thermal_ground_truth_node publishing "
                        "/thermal_truth GridMap for RViz overlay/rosbag."),
        DeclareLaunchArgument('qi_rate_hz', default_value='1.0',
            description="Rate at which /coverage/sector_weights is published. "
                        "Default 1.0 matches the optimizer round_rate_hz."),
        OpaqueFunction(function=_build_nodes),
    ])
