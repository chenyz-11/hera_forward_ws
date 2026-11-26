"""UAV formation switch: wide_2x3 -> single_column -> wide_2x3 -> hold.

Six BigUAVs follow the existing formation geometry using measured position and
height feedback. Velocity commands continue as position/height hold at the end.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# New-interface bridge parameters: home_timeout_ms / home_config_path are gone;
# home_source + home_wait_ms replace them.
SDK_PARAMETERS = {
    'ue_ip': '192.168.1.130', 'ue_port': 9990, 'local_port': 26530,
    'pull_interval_ms': 50, 'pull_method': 1,
    'home_source': 'ue', 'home_wait_ms': 5000,
    'namespace': 'sigma',
    'biguav_altitude_scale_m': 1.0,
    'pull_contents': '0_0_BigUAV_1, 0_0_BigUAV_2, 0_0_BigUAV_3, '
                     '0_0_BigUAV_4, 0_0_BigUAV_5, 0_0_BigUAV_6',
}

# BigUAV model_type=0. Adapter relays measured state to /ugvN/world_odom.
UAV_MAPPINGS = [
    f'{i}|0|0|0|{i}|/sigma/camp_0/squad_0/BigUAV_{i}/model_info|'
    f'/sigma/camp_0/squad_0/BigUAV_{i}/motion_command'
    for i in range(1, 7)
]

ADAPTER_PARAMETERS = {
    'enable_motion': False, 'enable_reference_trajectory': False,
    'enable_velocity_command': True,
    'velocity_command_flags': 3,
    'velocity_heading_rad': 0.0,
    'velocity_command_input_topics': [
        f'{i}|/hera/algorithm/uav{i}/velocity' for i in range(1, 7)],
    'formal_output_authorized': True,
    'entity_mappings_confirmed': True, 'coordinate_transform_confirmed': True,
    'dry_run_only': False, 'require_complete_swarm': True,
    'state_freshness_s': 0.5,
    'coordinated_arrival_enabled': False,
    'arrival_deceleration_enabled': False,
    'bounded_profile_enabled': False,
    'reference_trajectory_input_topics': [
        f'{i}|/hera/algorithm/uav{i}/path' for i in range(1, 7)],
    'map_translation_x': 0.0, 'map_translation_y': 0.0,
    'map_translation_z': 0.0, 'map_yaw_rad': 0.0,
    'ugv_mappings': UAV_MAPPINGS,
}

PUBLISHER_DEFAULTS = {
    'advance_distance_m': 50.0,
    'wide_spacing_m': 15.0,
    'column_spacing_m': 20.0,
    'hold_distance_m': 50.0,
    'flight_height_m': 5.0,
    'control_hz': 20.0,
    'max_horizontal_speed_mps': 5.0,
    'max_vertical_speed_mps': 5.0,
    'min_height_m': 2.0,
    'max_height_m': 8.0,
}


def _numeric(name, default):
    if isinstance(default, bool):
        return ParameterValue(LaunchConfiguration(name), value_type=bool)
    return ParameterValue(LaunchConfiguration(name), value_type=type(default))


def generate_launch_description():
    speed = LaunchConfiguration('reference_speed_mps')

    adapter = dict(ADAPTER_PARAMETERS)

    publisher_params = {
        name: _numeric(name, default) for name, default in PUBLISHER_DEFAULTS.items()
    }
    publisher_params['initial_poses_yaml'] = '/home/cyz/hera_forward_ws/src/hera_forward/config/sigma_biguav_initial_poses.yaml'
    publisher_params['cruise_speed_mps'] = ParameterValue(speed, value_type=float)
    publisher_params['mission_mode'] = 'formation'
    publisher_params['record_directory'] = LaunchConfiguration('record_directory')
    publisher_params['autostart'] = ParameterValue(LaunchConfiguration('autostart'), value_type=bool)
    # The existing path publisher remains available for read-only preview.
    preview_params = {k: publisher_params[k] for k in
                      ('advance_distance_m', 'wide_spacing_m', 'column_spacing_m',
                       'hold_distance_m', 'flight_height_m', 'initial_poses_yaml')}
    preview_params['reference_speed_mps'] = ParameterValue(speed, value_type=float)
    preview_params['sample_step_m'] = ParameterValue(LaunchConfiguration('sample_step_m'), value_type=float)
    preview_params['fixed_altitude'] = True

    return LaunchDescription([
        DeclareLaunchArgument('detour_preview_only', default_value='false',
                              description='Disable adapter formal output for candidate preview'),
        DeclareLaunchArgument('reference_speed_mps', default_value='5.0',
                              description='Cruise speed in m/s'),
        DeclareLaunchArgument('advance_distance_m', default_value='50.0',
                              description='Advance distance per form/switch segment'),
        DeclareLaunchArgument('wide_spacing_m', default_value='15.0',
                              description='Slot spacing of wide_2x3'),
        DeclareLaunchArgument('column_spacing_m', default_value='20.0',
                              description='Slot spacing of single_column'),
        DeclareLaunchArgument('hold_distance_m', default_value='50.0',
                              description='Final wide-formation advance after last switch'),
        DeclareLaunchArgument('sample_step_m', default_value='1.0',
                              description='Path sample spacing (anchor progress per point)'),
        DeclareLaunchArgument('flight_height_m', default_value='5.0',
                              description='Fixed cruise altitude in metres'),
        DeclareLaunchArgument('fixed_altitude', default_value='true',
                              description='Compatibility argument: execution always maintains flight_height_m'),
        DeclareLaunchArgument('control_hz', default_value='20.0'),
        DeclareLaunchArgument('max_horizontal_speed_mps', default_value='5.0'),
        DeclareLaunchArgument('max_vertical_speed_mps', default_value='0.4'),
        DeclareLaunchArgument('min_height_m', default_value='2.0'),
        DeclareLaunchArgument('max_height_m', default_value='8.0'),
        DeclareLaunchArgument('record_directory', default_value=''),
        DeclareLaunchArgument('autostart', default_value='true'),

        Node(
            package='sigma2ros_bridge', executable='sigma2ros_bridge_node',
            name='sigma2ros_bridge', output='screen',
            parameters=[SDK_PARAMETERS]),
        Node(
            package='hera_sigma_bridge', executable='sigma_ros_adapter.py',
            name='sigma_ros_adapter', output='screen',
            parameters=[adapter],
            condition=UnlessCondition(LaunchConfiguration('detour_preview_only'))),
        Node(
            package='hera_sigma_bridge', executable='sigma_ros_adapter.py',
            name='sigma_ros_adapter', output='screen',
            parameters=[adapter, {
                'enable_motion': False, 'enable_reference_trajectory': False,
                'enable_velocity_command': False,
                'coordinated_arrival_enabled': True,
                'formal_output_authorized': False, 'dry_run_only': True,
            }], condition=IfCondition(LaunchConfiguration('detour_preview_only'))),
        Node(
            package='hera_forward', executable='uav_velocity_publisher',
            name='uav_velocity_publisher', output='screen',
            parameters=[publisher_params],
            condition=UnlessCondition(LaunchConfiguration('detour_preview_only'))),
        Node(
            package='hera_forward', executable='uav_formation_switch_publisher',
            name='uav_formation_switch_publisher', output='screen',
            parameters=[preview_params],
            condition=IfCondition(LaunchConfiguration('detour_preview_only'))),
    ])
