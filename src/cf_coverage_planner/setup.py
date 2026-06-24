from glob import glob

from setuptools import find_packages, setup

package_name = 'cf_coverage_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rk32226',
    maintainer_email='rk32226@example.com',
    description='Per-drone coverage planner + rate-limited setpoint streamer for the radial coverage demo.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'coverage_planner_node = cf_coverage_planner.coverage_planner_node:main',
            'setpoint_streamer_node = cf_coverage_planner.setpoint_streamer_node:main',
            'quadrant_figure8_node = cf_coverage_planner.quadrant_figure8_node:main',
            'rl_planner_node = cf_coverage_planner.rl_planner_node:main',
            'trajectory_streamer_node = cf_coverage_planner.trajectory_streamer_node:main',
            'fullstate_streamer_node = cf_coverage_planner.fullstate_streamer_node:main',
            'polynomial_streamer_node = cf_coverage_planner.polynomial_streamer_node:main',
            'receding_horizon_streamer_node = cf_coverage_planner.receding_horizon_streamer_node:main',
        ],
    },
)
