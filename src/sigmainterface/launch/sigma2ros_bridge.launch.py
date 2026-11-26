"""
起 sigma2ros_bridge 桥接节点。

参数默认从包内 config/sigma2ros_bridge.yaml 读取，可整体替换：
    ros2 launch sigma2ros_bridge sigma2ros_bridge.launch.py
    ros2 launch sigma2ros_bridge sigma2ros_bridge.launch.py params_file:=/path/to/my.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('sigma2ros_bridge'),
                                  'config', 'bridge_params.yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='节点参数 YAML（默认包内 config/bridge_params.yaml）'),

        Node(
            package='sigma2ros_bridge',
            executable='sigma2ros_bridge_node',
            name='sigma2ros_bridge',
            output='screen',
            parameters=[params_file],
        ),
    ])
