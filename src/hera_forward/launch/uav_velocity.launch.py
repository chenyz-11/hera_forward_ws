"""UAV formation switch via the new VelocityCommand interface.

Six UAVs (MiddleUAV entities) start from sigma_uav_only_initial_poses.yaml and
perform wide_2x3 -> single_column -> wide_2x3 -> hold while climbing from the
ground to a cruise altitude.  The publisher streams Twist velocities; the
(new-interface) adapter converts them to Sigma VelocityCommand on
``{entity}/velocity_command``.  Requires the NEW sigma2ros_bridge interface.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# New-interface bridge parameters: home_timeout_ms / home_config_path are gone;
# home_source + home_wait_ms replace them.  Keep ue_ip as used by prior demos.
SDK_PARAMETERS = {
    'ue_ip': '192.168.1.130', 'ue_port': 9990, 'local_port': 26530,
    'pull_interval_ms': 100, 'pull_method': 1,
    'home_source': 'ue', 'home_wait_ms': 5000,
    'namespace': 'sigma',
    'pull_contents': '0_0_BigUAV_1, 0_0_BigUAV_2, 0_0_BigUAV_3, '
                     '0_0_BigUAV_4, 0_0_BigUAV_5, 0_0_BigUAV_6',
}

# model_type=1 -> 'MiddleUAV'.  Adapter relays state to /ugvN/world_odom and
# derives {entity}/velocity_command from the motion_command topic.
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
    'map_translation_x': 0.0, 'map_translation_y': 0.0,
    'map_translation_z': 0.0, 'map_yaw_rad': 0.0,
    'ugv_mappings': UAV_MAPPINGS,
}

PUBLISHER_DEFAULTS = {
    'advance_distance_m': 50.0,
    'wide_spacing_m': 15.0,
    'column_spacing_m': 20.0,
    'hold_distance_m': 50.0,
    'cruise_speed_mps': 0.6,
    'control_hz': 20.0,
    'flight_height_m': 5.0,
    'climb_distance_m': 50.0,
    'auto_altitude': True,
    'stop_hold_s': 3.0,
}


def _numeric(name, default):
    return ParameterValue(LaunchConfiguration(name), value_type=type(default))


def generate_launch_description():
    velocity_flags = LaunchConfiguration('velocity_command_flags')
    velocity_heading = LaunchConfiguration('velocity_heading_rad')
    adapter = dict(ADAPTER_PARAMETERS)
    adapter['velocity_command_flags'] = ParameterValue(velocity_flags, value_type=int)
    adapter['velocity_heading_rad'] = ParameterValue(velocity_heading, value_type=float)
    dry_adapter = dict(adapter)

    publisher_params = {
        name: _numeric(name, default) for name, default in PUBLISHER_DEFAULTS.items()
    }
    publisher_params['initial_poses_yaml'] = '/home/cyz/hera_forward_ws/src/hera_forward/config/sigma_biguav_initial_poses.yaml'

    dry_adapter.update({
        'enable_velocity_command': False,
        'formal_output_authorized': False, 'dry_run_only': True,
    })

    return LaunchDescription([
        DeclareLaunchArgument('detour_preview_only', default_value='false',
                              description='Disable adapter formal output for candidate preview'),
        DeclareLaunchArgument('velocity_command_flags', default_value='3',
                              description='VelocityCommand flags (1 velocity, 3 velocity+yaw, 5 velocity+yaw_rate)'),
        DeclareLaunchArgument('velocity_heading_rad', default_value='0.0',
                              description='Heading setpoint (ENU rad) sent with velocity commands'),
        DeclareLaunchArgument('cruise_speed_mps', default_value='0.6',
                              description='Cruise speed in m/s'),
        DeclareLaunchArgument('control_hz', default_value='20.0',
                              description='Velocity command publish rate'),
        DeclareLaunchArgument('advance_distance_m', default_value='50.0',
                              description='Advance distance per form/switch segment'),
        DeclareLaunchArgument('wide_spacing_m', default_value='15.0',
                              description='Slot spacing of wide_2x3'),
        DeclareLaunchArgument('column_spacing_m', default_value='20.0',
                              description='Slot spacing of single_column'),
        DeclareLaunchArgument('hold_distance_m', default_value='50.0',
                              description='Final wide-formation advance after last switch'),
        DeclareLaunchArgument('flight_height_m', default_value='5.0',
                              description='Cruise altitude (UAV climbs during form stage)'),
        DeclareLaunchArgument('climb_distance_m', default_value='50.0',
                              description='Advance over which climb completes'),
        DeclareLaunchArgument('auto_altitude', default_value='true',
                              description='UE auto-manages altitude: keep vz=0 (true) vs climb profile (false)'),
        DeclareLaunchArgument('stop_hold_s', default_value='3.0',
                              description='Seconds to hold zero velocity at the end'),

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
            parameters=[dry_adapter],
            condition=IfCondition(LaunchConfiguration('detour_preview_only'))),
        Node(
            package='hera_forward', executable='uav_velocity_publisher',
            name='uav_velocity_publisher', output='screen',
            parameters=[publisher_params]),
    ])
