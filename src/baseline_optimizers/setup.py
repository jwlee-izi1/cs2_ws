from setuptools import find_packages, setup

package_name = 'baseline_optimizers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rk32226',
    maintainer_email='rk32226@example.com',
    description='Mock leash publishers for testing the coverage planner without a real optimizer.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'constant_leash_node = baseline_optimizers.constant_leash_node:main',
            'sinusoidal_leash_node = baseline_optimizers.sinusoidal_leash_node:main',
        ],
    },
)
