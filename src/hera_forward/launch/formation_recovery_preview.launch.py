"""Read platform odometry and preview recovery; no formal command publishers."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mappings = [
        f'{i}|4|0|0|{i}|/sigma/camp_0/squad_0/MiddleUGV_{i}/model_info|'
        f'/sigma/camp_0/squad_0/MiddleUGV_{i}/motion_command'
        for i in range(1, 7)]
    return LaunchDescription([
        Node(package='sigma2ros_bridge', executable='sigma2ros_bridge_node', output='screen',
             parameters=[{
                 'ue_ip': '192.168.1.130', 'ue_port': 9990, 'local_port': 26530,
                 'pull_interval_ms': 100, 'pull_method': 1, 'home_timeout_ms': -1,
                 'home_config_path': '/home/cyz/hera_swarm_ws/src/third_party/sigmainterface/config/home.json',
                 'namespace': 'sigma',
                 'pull_contents': ', '.join(f'0_0_MiddleUGV_{i}' for i in range(1, 7)),
             }]),
        Node(package='hera_sigma_bridge', executable='sigma_ros_adapter.py', output='screen',
             parameters=[{'ugv_mappings': mappings, 'require_complete_swarm': True,
                          'enable_motion': False, 'enable_reference_trajectory': True,
                          'formal_output_authorized': True, 'dry_run_only': False,
                          'publish_dry_run_topics': False}]),
        Node(package='hera_forward', executable='recovery_preview', output='screen'),
    ])
