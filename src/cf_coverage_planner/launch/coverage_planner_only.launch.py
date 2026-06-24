"""Spawn ONLY the per-drone planners + streamers (no optimizer, no takeoff, no thermal).

Use when you want to run a custom optimizer in a separate terminal and let it
drive the planners through /coverage/leash. Assumes drones are already in
stable hover at the configured altitude.
"""

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _build_nodes(context, *args, **kwargs):
    pkg_share = get_package_share_directory('cf_coverage_planner')
    with open(os.path.join(pkg_share, 'config', 'arena_4drone.yaml')) as f:
        cfg = yaml.safe_load(f)

    drone_names = list(cfg['drone_names'])
    arena = cfg['arena']
    sectors = cfg['sectors']
    streamer_cfg = cfg['streamer']
    planner_cfg = cfg['planner']

    nodes = []
    for name in drone_names:
        s = sectors[name]
        nodes.append(Node(
            package='cf_coverage_planner',
            executable='coverage_planner_node',
            name=f'coverage_planner_{name}',
            output='screen',
            parameters=[{
                'drone_name': name,
                'center_xy': list(arena['center_xy']),
                'altitude': float(arena['altitude']),
                'phi_mid_deg': float(s['phi_mid_deg']),
                'phi_half_deg': float(s['phi_half_deg']),
                'r_star': float(s['r_star']),
                'r_max': float(s['r_max']),
                'q': float(s['q']),
                'c': float(s['c']),
                'footprint_radius': float(planner_cfg['footprint_radius']),
                'deadband': float(planner_cfg.get('deadband', 0.10)),
                'drone_speed': float(streamer_cfg['max_setpoint_velocity']),
                'num_spokes': int(planner_cfg.get('num_spokes', 5)),
                'inner_radius': float(planner_cfg.get('inner_radius', 0.5)),
                'planner_rate_hz': float(planner_cfg['planner_rate_hz']),
            }],
        ))
        nodes.append(Node(
            package='cf_coverage_planner',
            executable='setpoint_streamer_node',
            name=f'setpoint_streamer_{name}',
            output='screen',
            parameters=[{
                'drone_name': name,
                'setpoint_rate_hz': float(streamer_cfg['setpoint_rate_hz']),
                'max_setpoint_velocity': float(streamer_cfg['max_setpoint_velocity']),
            }],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_build_nodes),
    ])
