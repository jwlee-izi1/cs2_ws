from setuptools import find_packages, setup

package_name = 'multinash_cs2_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rk32226',
    maintainer_email='josephymikhail@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'multinash_bridge = multinash_cs2_bridge.multinash_bridge:main',
            'multinash_exec_cf1 = multinash_cs2_bridge.multinash_exec_cf1:main',
            'multinash_exec_cf1_mnash = multinash_cs2_bridge.multinash_exec_cf1_mnash:main',
            'multinash_exec_two_cf_mnash = multinash_cs2_bridge.multinash_exec_two_cf_mnash:main',
            'multinash_exec_N_cf_mnash = multinash_cs2_bridge.multinash_exec_N_cf_mnash:main',
            'multinash_exec_N_cf_traj = multinash_cs2_bridge.multinash_exec_N_cf_traj:main',
        ],
    },
)
