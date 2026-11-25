"""Chapter-5 demo: environment-adaptive formation around a rectangular obstacle.

Six vehicles start from the config initial poses (a ring). The publisher emits
one timed ``nav_msgs/Path`` per vehicle; the adapter forwards them to UE.
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
        f'{i}|/hera/algorithm/ugv{i}/path' for i in range(1, 7)],
    'map_translation_x': 0.0, 'map_translation_y': 0.0,
    'map_translation_z': 0.0, 'map_yaw_rad': 0.0,
    'ugv_mappings': UGV_MAPPINGS,
}

PUBLISHER_DEFAULTS = {
    'obstacle_east_m': 75.0,
    'obstacle_half_width_y_m': 10.0,
    'obstacle_half_depth_x_m': 6.0,
    'clearance_margin_m': 5.0,
    'bypass_extra_m': 5.0,
    'scale_min': 0.7,
    'approach_m': 25.0,
    'shift_m': 40.0,
    'pass_m': 40.0,
    'return_m': 40.0,
    'final_m': 50.0,
    'step_m': 1.0,
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
        DeclareLaunchArgument('obstacle_east_m', default_value='75.0',
                              description='Obstacle distance east of fleet centre'),
        DeclareLaunchArgument('obstacle_half_width_y_m', default_value='10.0',
                              description='Obstacle half-width in y (total 20 m)'),
        DeclareLaunchArgument('obstacle_half_depth_x_m', default_value='6.0',
                              description='Obstacle half-depth in x'),
        DeclareLaunchArgument('clearance_margin_m', default_value='5.0',
                              description='Extra clearance beyond obstacle edge'),
        DeclareLaunchArgument('bypass_extra_m', default_value='5.0',
                              description='Additional north bypass buffer'),
        DeclareLaunchArgument('scale_min', default_value='0.7',
                              description='Contracted ring scale near obstacle'),
        DeclareLaunchArgument('approach_m', default_value='25.0',
                              description='Advance before the shift begins'),
        DeclareLaunchArgument('shift_m', default_value='40.0',
                              description='Advance while shifting north + contracting'),
        DeclareLaunchArgument('pass_m', default_value='40.0',
                              description='Advance while passing the obstacle'),
        DeclareLaunchArgument('return_m', default_value='40.0',
                              description='Advance while returning + restoring'),
        DeclareLaunchArgument('final_m', default_value='50.0',
                              description='Final advance after restore'),
        DeclareLaunchArgument('step_m', default_value='1.0',
                              description='Path sample spacing'),

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
            package='hera_forward', executable='obstacle_avoidance_publisher',
            name='obstacle_avoidance_publisher', output='screen',
            parameters=[publisher_params]),
    ])
