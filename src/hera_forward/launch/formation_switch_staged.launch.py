"""Staged three-stage formation switch (earlier scheme).

Same geometry as the continuous scheme but published stage by stage with a
bounded speed profile: form wide_2x3 while advancing -> switch to single_column
-> switch back to wide_2x3.  Each stage is replaced once every vehicle is within
``stage_error_threshold_m`` of its slot.  The adapter runs in bounded-profile
mode, which re-derives and enforces the smooth acceleration/deceleration profile
and stops at each stage end.
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
    'pull_contents': '0_0_MiddleUGV_1, 0_0_MiddleUGV_2, 0_0_MiddleUGV_3, 0_0_MiddleUGV_4, 0_0_MiddleUGV_5, 0_0_MiddleUGV_6',
}

UGV_MAPPINGS = [
    f'{i}|4|0|0|{i}|/sigma/camp_0/squad_0/MiddleUGV_{i}/model_info|/sigma/camp_0/squad_0/MiddleUGV_{i}/motion_command'
    for i in range(1, 7)
]

ACCELERATION_MPS2 = 0.4
DECELERATION_MPS2 = 0.6
LATERAL_ACCELERATION_MPS2 = 0.6

ADAPTER_PARAMETERS = {
    'enable_motion': False, 'enable_reference_trajectory': True,
    'formal_output_authorized': True,
    'entity_mappings_confirmed': True, 'coordinate_transform_confirmed': True,
    'dry_run_only': False, 'require_complete_swarm': True,
    'state_freshness_s': 0.5,
    # Bounded-profile mode re-derives a smooth speed profile from the timed
    # paths and enforces the shared acceleration/deceleration limits (stops at
    # each stage end).
    'coordinated_arrival_enabled': True,
    'arrival_deceleration_enabled': True,
    'bounded_profile_enabled': True,
    'trajectory_acceleration_mps2': ACCELERATION_MPS2,
    'trajectory_deceleration_mps2': DECELERATION_MPS2,
    'trajectory_lateral_acceleration_mps2': LATERAL_ACCELERATION_MPS2,
    'trajectory_minimum_moving_speed_mps': 0.0,
    'reference_trajectory_input_topics': [
        f'{i}|/hera/algorithm/ugv{i}/path' for i in range(1, 7)],
    'map_translation_x': 0.0, 'map_translation_y': 0.0,
    'map_translation_z': 0.0, 'map_yaw_rad': 0.0,
    'ugv_mappings': UGV_MAPPINGS,
}

PUBLISHER_DEFAULTS = {
    'advance_distance_m': 100.0,
    'wide_spacing_m': 15.0,
    'column_spacing_m': 20.0,
    'max_segment_m': 2.0,
    'turning_radius_m': 4.5,
    'stage_error_threshold_m': 5.0,
    'acceleration_mps2': ACCELERATION_MPS2,
    'deceleration_mps2': DECELERATION_MPS2,
    'lateral_acceleration_mps2': LATERAL_ACCELERATION_MPS2,
}


def _numeric(name, default):
    if isinstance(default, bool):
        return ParameterValue(LaunchConfiguration(name), value_type=bool)
    return ParameterValue(LaunchConfiguration(name), value_type=type(default))


def generate_launch_description():
    speed = LaunchConfiguration('reference_speed_mps')

    adapter = dict(ADAPTER_PARAMETERS)
    adapter['reference_trajectory_speed_mps'] = ParameterValue(speed, value_type=float)
    adapter['trajectory_max_speed_mps'] = ParameterValue(speed, value_type=float)

    publisher_params = {
        name: _numeric(name, default) for name, default in PUBLISHER_DEFAULTS.items()
    }
    publisher_params['reference_speed_mps'] = ParameterValue(speed, value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('detour_preview_only', default_value='false',
                              description='Disable adapter formal output for candidate preview'),
        DeclareLaunchArgument('reference_speed_mps', default_value='5.0',
                              description='Cruise-speed cap used by the publisher and adapter'),
        DeclareLaunchArgument('advance_distance_m', default_value='100.0',
                              description='Eastward distance per form/switch segment'),
        DeclareLaunchArgument('wide_spacing_m', default_value='15.0',
                              description='Slot spacing of the wide_2x3 formation'),
        DeclareLaunchArgument('column_spacing_m', default_value='20.0',
                              description='Slot spacing of the single_column formation'),
        DeclareLaunchArgument('max_segment_m', default_value='2.0',
                              description='Dubins path sample spacing'),
        DeclareLaunchArgument('turning_radius_m', default_value='4.5',
                              description='Vehicle turn radius used to build Dubins paths'),
        DeclareLaunchArgument('stage_error_threshold_m', default_value='5.0',
                              description='Per-vehicle error below which the next stage is published'),
        DeclareLaunchArgument('acceleration_mps2', default_value=str(ACCELERATION_MPS2),
                              description='Longitudinal acceleration limit'),
        DeclareLaunchArgument('deceleration_mps2', default_value=str(DECELERATION_MPS2),
                              description='Longitudinal deceleration limit'),
        DeclareLaunchArgument('lateral_acceleration_mps2', default_value=str(LATERAL_ACCELERATION_MPS2),
                              description='Lateral acceleration limit'),

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
            package='hera_forward', executable='formation_switch_staged_publisher',
            name='formation_switch_staged_publisher', output='screen',
            parameters=[publisher_params]),
    ])
