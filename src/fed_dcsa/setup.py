import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fed_dcsa'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rk32226',
    maintainer_email='rk32226@example.com',
    description='Fed-DCSA: Distributed Resource Allocation with Server-Coordinated Gate',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fed_dcsa_coordinator = fed_dcsa.coordinator_node:main',
            'payload_optimizer = fed_dcsa.payload_optimizer_node:main',
            'radial_coverage_node = fed_dcsa.radial_coverage_node:main',
        ],
    },
)
