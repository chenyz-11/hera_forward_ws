"""Chapter-4 reconfiguration demonstration: node exit / local reconfigure / rejoin.

Six vehicles start from their recorded initial poses (that IS the initial
formation). One follower is confirmed withdrawn; the five remaining vehicles
reconfigure while advancing; the withdrawn vehicle rejoins and the initial
formation is restored -- all while moving.  The publisher emits timed
``nav_msgs/Path`` messages and the adapter forwards them into the UE simulator.
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
    'initial_advance_m': 50.0,
    'exit_drift_m': 30.0,
    'exit_offset_m': 20.0,
    'compact_hold_m': 30.0,
    'rejoin_drift_m': 30.0,
    'final_advance_m': 50.0,
    'step_m': 1.0,
}


def _numeric(name, default):
    if isinstance(default, bool):
        return ParameterValue(LaunchConfiguration(name), value_type=bool)
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
                              description='Cruise speed cap in m/s'),
        DeclareLaunchArgument('initial_advance_m', default_value='50.0',
                              description='Initial formation straight advance'),
        DeclareLaunchArgument('exit_drift_m', default_value='30.0',
                              description='Advance distance while the NW vehicle drifts out'),
        DeclareLaunchArgument('exit_offset_m', default_value='20.0',
                              description='Radial distance the withdrawn vehicle drifts off the ring'),
        DeclareLaunchArgument('compact_hold_m', default_value='30.0',
                              description='Advance while holding the 5-vehicle ring'),
        DeclareLaunchArgument('rejoin_drift_m', default_value='30.0',
                              description='Advance distance while the vehicle rejoins'),
        DeclareLaunchArgument('final_advance_m', default_value='50.0',
                              description='Final restored-formation advance'),
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
            package='hera_forward', executable='reconfiguration_publisher',
            name='reconfiguration_publisher', output='screen',
            parameters=[publisher_params]),
    ])
