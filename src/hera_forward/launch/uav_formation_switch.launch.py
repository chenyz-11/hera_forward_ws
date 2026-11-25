"""UAV formation switch: wide_2x3 -> single_column -> wide_2x3 -> hold.

Six UAVs (MiddleUAV entities) start from sigma_uav_only_initial_poses.yaml at
their recorded altitude (~0.05 m, no height control).  The publisher emits one
continuous timed ``nav_msgs/Path`` per UAV; the adapter relays them to UE as
reference trajectories.  Horizontal plane only, no altitude command.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SDK_PARAMETERS = {
    'ue_ip': '192.168.1.130', 'ue_port': 9990, 'local_port': 26530,
    'pull_interval_ms': 100, 'pull_method': 1, 'home_timeout_ms': -1,
    'home_config_path': '/home/cyz/hera_swarm_ws/src/third_party/sigmainterface/config/home.json',
    'namespace': 'sigma',
    'pull_contents': '0_0_MiddleUAV_1, 0_0_MiddleUAV_2, 0_0_MiddleUAV_3, '
                     '0_0_MiddleUAV_4, 0_0_MiddleUAV_5, 0_0_MiddleUAV_6',
}

# model_type=1 -> name 'MiddleUAV' (sigma_adapter_logic names[1]); entity topics
# must use that exact spelling.  Adapter relays state to /ugvN/world_odom.
UAV_MAPPINGS = [
    f'{i}|1|0|0|{i}|/sigma/camp_0/squad_0/MiddleUAV_{i}/model_info|'
    f'/sigma/camp_0/squad_0/MiddleUAV_{i}/motion_command'
    for i in range(1, 7)
]

ADAPTER_PARAMETERS = {
    'enable_motion': False, 'enable_reference_trajectory': True,
    'formal_output_authorized': True,
    'entity_mappings_confirmed': True, 'coordinate_transform_confirmed': True,
    'dry_run_only': False, 'require_complete_swarm': True,
    'state_freshness_s': 0.5,
    'coordinated_arrival_enabled': True,
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
    'sample_step_m': 1.0,
    'flight_height_m': 10.0,
}


def _numeric(name, default):
    return ParameterValue(LaunchConfiguration(name), value_type=type(default))


def generate_launch_description():
    speed = LaunchConfiguration('reference_speed_mps')

    adapter = dict(ADAPTER_PARAMETERS)
    adapter['reference_trajectory_speed_mps'] = ParameterValue(speed, value_type=float)

    publisher_params = {
        name: _numeric(name, default) for name, default in PUBLISHER_DEFAULTS.items()
    }
    publisher_params['reference_speed_mps'] = ParameterValue(speed, value_type=float)

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
        DeclareLaunchArgument('flight_height_m', default_value='10.0',
                              description='Cruise altitude (UAV climbs from ground during form stage)'),

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
                'formal_output_authorized': False, 'dry_run_only': True,
            }], condition=IfCondition(LaunchConfiguration('detour_preview_only'))),
        Node(
            package='hera_forward', executable='uav_formation_switch_publisher',
            name='uav_formation_switch_publisher', output='screen',
            parameters=[publisher_params]),
    ])
