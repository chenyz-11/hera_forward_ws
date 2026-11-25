from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hera_forward'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyz',
    maintainer_email='cyz@todo.todo',
    description='Six-vehicle forward advance (~100 m) that preserves the recorded formation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'six_forward_100m = hera_forward.six_forward_100m:main',
            'formation_switch_publisher = hera_forward.formation_switch_publisher:main',
            'formation_switch_staged_publisher = hera_forward.formation_switch_staged_publisher:main',
            'formation_advance_turn_publisher = hera_forward.formation_advance_turn_publisher:main',
            'formation_drop_return_publisher = hera_forward.formation_drop_return_publisher:main',
            'reconfiguration_publisher = hera_forward.reconfiguration_publisher:main',
            'obstacle_avoidance_publisher = hera_forward.obstacle_avoidance_publisher:main',
            'damaged_node_publisher = hera_forward.damaged_node_publisher:main',
            'uav_formation_switch_publisher = hera_forward.uav_formation_switch_publisher:main',
            'uav_motion_publisher = hera_forward.uav_motion_publisher:main',
            'uav_system_control = hera_forward.system_control_sender:main',
            'recovery_preview = hera_forward.recovery_preview_node:main',
        ],
    },
)
