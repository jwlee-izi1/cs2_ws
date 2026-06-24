"""
payload_hover_crazysim.launch.py
================================
Bring up the cf_payload_world simulation with the **CrazySim firmware-in-the-loop**
backend instead of the old MulticopterVelocityControl plugin.

Pipeline:
    Gazebo Harmonic + payload_world_crazysim.sdf (4 drones inline, gz_crazysim_plugin
    each, ports 19951..19954 / 19851..19854)
            ↑ UDP loopback (containers use --network=host)
    4 cf2-N docker containers (cf2-sitl:22.04 image) running full Crazyflie firmware
            ↑ CRTP/UDP from crazyswarm2 server
    crazyswarm2 server (cflib backend) serving /cfN/takeoff, /cfN/go_to, /all/*

Usage:
    ros2 launch cf_payload_world payload_hover_crazysim.launch.py
    ros2 launch cf_payload_world payload_hover_crazysim.launch.py config:=tilted

Prereqs:
    - cf2-sitl:22.04 image already built  (see CRAZYSIM_MIGRATION.md)
    - crazyflie-firmware built on host    (gz plugin already on GZ_SIM_SYSTEM_PLUGIN_PATH)
    - cflib installed from source         (newer UdpDriver)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DRONES = ['cf1', 'cf2', 'cf3', 'cf4']      # cf_id 1..4 ↔ ports 19851..19854 / 19951..19954
DEFAULT_YAML = os.path.expanduser('~/cs2_ws/config/crazyflies_payload.yaml')


def launch_setup(context, *args, **kwargs):
    config         = LaunchConfiguration('config').perform(context)
    crazyflies_yaml = LaunchConfiguration('crazyflies_yaml_file').perform(context)
    paused         = LaunchConfiguration('paused').perform(context)

    entities = []

    # 1) Gazebo + payload_world_crazysim.sdf (uses payload_world.launch.py with overridden world_file)
    payload_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('cf_payload_world'),
                'launch', 'payload_world.launch.py',
            )
        ),
        launch_arguments={
            'config': config,
            'paused': paused,
            'world_file': 'payload_world_crazysim.sdf',
        }.items(),
    )
    entities.append(payload_world_launch)

    # 2) Wire GZ_SIM_SYSTEM_PLUGIN_PATH so libgz_crazysim_plugin.so is found
    fw_dir = os.path.expanduser('~/cs2_ws/crazyflie-firmware')
    plugin_dir = os.path.join(fw_dir, 'sitl_make/build/build_crazysim_gz')
    existing = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    if plugin_dir not in existing:
        os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = (
            plugin_dir + ':' + existing if existing else plugin_dir
        )

    # 3) Spawn one cf2 docker container per drone (one firmware-in-loop instance each).
    #    cf2-N listens for handshake on port 19950+N, talks cflib on port 19850+N.
    for i, name in enumerate(DRONES, start=1):
        cffirm_port = 19950 + i
        entities.append(LogInfo(msg=f'[crazysim] starting container cf2-{i} -> port {cffirm_port}'))
        # Detached docker run — container survives the launch action's lifetime via OnShutdown.
        entities.append(ExecuteProcess(
            cmd=[
                'docker', 'run', '-d', '--rm',
                '--network=host',
                '--name', f'cf2-{i}',
                'cf2-sitl:22.04',
                str(cffirm_port), '127.0.0.1',
            ],
            output='screen',
        ))

    # 4) crazyswarm2 server with the per-drone UDP YAML.
    # Delay 15 s so cf2 containers have time to boot and complete the 0xF3 handshake
    # with the Gazebo plugin. Without this, cflib sends to a not-yet-initialised plugin
    # cflib socket, the kernel returns ICMP-unreachable, and the server's UDP recv
    # raises ConnectionRefusedError → server exits.
    entities.append(TimerAction(
        period=15.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('crazyflie'),
                    'launch', 'launch.py',
                )
            ),
            launch_arguments={
                'backend': 'cflib',
                'crazyflies_yaml_file': crazyflies_yaml,
                'gui': 'false',
            }.items(),
        )],
    ))

    # 5) Cleanup: stop cf2-* containers when the launch is killed (Ctrl-C / shutdown).
    entities.append(RegisterEventHandler(
        OnShutdown(
            on_shutdown=[ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    'docker ps -q --filter "name=cf2-" | xargs -r docker stop || true',
                ],
                output='screen',
            )]
        )
    ))

    return entities


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='level',
            description='Drone height configuration: "level" or "tilted".',
        ),
        DeclareLaunchArgument(
            'paused',
            default_value='true',
            description=(
                'Start Gazebo paused. Recommended TRUE for the payload world: '
                'the drones are rod-coupled to a 50 g payload and will tumble '
                'if physics runs before they can produce hover thrust. After '
                'the crazyswarm2 server reports all drones fully connected, '
                'send /all/takeoff and THEN unpause via the Gazebo GUI play '
                'button (or `gz service -s /world/cf_payload_world/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 1000 --req "pause: false"`).'
            ),
        ),
        DeclareLaunchArgument(
            'crazyflies_yaml_file',
            default_value=DEFAULT_YAML,
            description='Path to crazyflies YAML for the crazyswarm2 server.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
