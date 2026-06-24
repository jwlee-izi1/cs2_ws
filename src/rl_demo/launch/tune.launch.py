"""Phase A tuning launch: one Crazyflie in the stock open world.

Delegates to ros_gz_crazyflie_bringup/crazyflie_simulation.launch.py with a
single-drone YAML. No model override (Phase A doesn't care about mesh scale).
Gains are set at runtime by tune_tracking.py via the ROS2 param API — the
shared launch doesn't pass gain params through, so runtime-set is the
non-invasive path.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_launch = os.path.join(
        get_package_share_directory('ros_gz_crazyflie_bringup'),
        'launch', 'crazyflie_simulation.launch.py',
    )
    rl_demo_share = get_package_share_directory('rl_demo')
    tuning_yaml = os.path.join(rl_demo_share, 'config', 'tuning_drones.yaml')
    rl_demo_models = os.path.join(rl_demo_share, 'models')

    # Prepend our private models dir so the shared bringup's GZ_SIM_RESOURCE_PATH
    # lookup finds our crazyflie/model.sdf first. Shared model left untouched;
    # other launches (without this prepend) still find the shared model.
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_path = rl_demo_models + (':' + existing if existing else '')
    # Set immediately so the shared launch's OpaqueFunction (which reads
    # os.getenv at invocation time) sees our prepended path.
    os.environ['GZ_SIM_RESOURCE_PATH'] = new_path

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={'crazyflies_yaml_file': tuning_yaml}.items(),
        ),
    ])
