import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'thermal_mapping'

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
    description='Simulated thermal sensor + shared multi-drone heat map for the CrazySim stack.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'thermal_sensor_node = thermal_mapping.thermal_sensor_node:main',
            'thermal_mapper_node = thermal_mapping.thermal_mapper_node:main',
            'thermal_ground_truth_node = '
            'thermal_mapping.thermal_ground_truth_node:main',
            'qi_estimator_node = thermal_mapping.qi_estimator_node:main',
        ],
    },
)
