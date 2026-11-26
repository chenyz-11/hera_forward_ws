"""BigUAV feedback control: six-aircraft translation or existing formation geometry."""
import json
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    ids = json.loads(get('robot_ids'))
    sdk = {'ue_ip': get('ue_ip'), 'ue_port': 9990, 'local_port': 26530,
           'pull_interval_ms': 50, 'home_source': 'ue', 'home_wait_ms': 8000,
           'namespace': 'sigma', 'biguav_altitude_scale_m': 1.0,
           'pull_contents': ','.join(f'0_0_BigUAV_{i}' for i in range(1, 7))}
    adapter = {
        'enable_motion': False, 'enable_reference_trajectory': False, 'enable_velocity_command': True,
        'velocity_command_flags': 3, 'velocity_heading_rad': 0.0,
        'velocity_command_input_topics': [f'{i}|/hera/algorithm/uav{i}/velocity' for i in range(1, 7)],
        'formal_output_authorized': True, 'entity_mappings_confirmed': True,
        'coordinate_transform_confirmed': True, 'dry_run_only': False,
        'require_complete_swarm': True, 'state_freshness_s': .5,
        'ugv_mappings': [f'{i}|0|0|0|{i}|/sigma/camp_0/squad_0/BigUAV_{i}/model_info|'
                         f'/sigma/camp_0/squad_0/BigUAV_{i}/motion_command' for i in range(1, 7)]}
    publisher = {'robot_ids': ids, 'mission_mode': get('mission_mode'),
                 'autostart': get('autostart').lower() == 'true',
                 'record_directory': get('record_directory')}
    for key in ('translate_x_m', 'translate_y_m', 'flight_height_m', 'cruise_speed_mps',
                'max_horizontal_speed_mps', 'advance_distance_m', 'wide_spacing_m',
                'column_spacing_m', 'hold_distance_m'):
        publisher[key] = float(get(key))
    return [
        Node(package='sigma2ros_bridge', executable='sigma2ros_bridge_node', name='sigma2ros_bridge',
             output='screen', parameters=[sdk]),
        Node(package='hera_sigma_bridge', executable='sigma_ros_adapter.py', name='sigma_ros_adapter',
             output='screen', parameters=[adapter]),
        Node(package='hera_forward', executable='uav_velocity_publisher', name='uav_velocity_publisher',
             output='screen', parameters=[publisher])]


def generate_launch_description():
    defaults = {'ue_ip': '192.168.1.130', 'robot_ids': '[1,2,3,4,5,6]',
                'mission_mode': 'translate', 'autostart': 'true', 'record_directory': '',
                'translate_x_m': '6.0', 'translate_y_m': '0.0', 'flight_height_m': '5.0',
                'cruise_speed_mps': '0.6', 'max_horizontal_speed_mps': '1.0',
                'advance_distance_m': '50.0', 'wide_spacing_m': '15.0',
                'column_spacing_m': '20.0', 'hold_distance_m': '50.0'}
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
                             + [OpaqueFunction(function=nodes)])
