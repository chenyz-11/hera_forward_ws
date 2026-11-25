"""Six vehicles advance ~100 m east in formation from their recorded poses.

Mirrors ``sigma_algorithm_trajectory_integration.launch.py``: the SDK bridge
pulls platform state, the adapter converts each timed ``nav_msgs/Path`` into a
``ReferenceTrajectory``, and ``six_forward_100m`` publishes the six formation
paths.  The adapter's arrival-deceleration parameters MUST match the publisher's
so the timed profile validates.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# --- SDK transport (same as the existing Sigma integration launch) ---
SDK_PARAMETERS = {
    'ue_ip': '192.168.1.130', 'ue_port': 9990, 'local_port': 26530,
    'pull_interval_ms': 100, 'pull_method': 1, 'home_timeout_ms': -1,
    'home_config_path': '/home/cyz/hera_swarm_ws/src/third_party/sigmainterface/config/home.json',
    'namespace': 'sigma',
    'pull_contents': '0_0_MiddleUGV_1, 0_0_MiddleUGV_2, 0_0_MiddleUGV_3, 0_0_MiddleUGV_4, 0_0_MiddleUGV_5, 0_0_MiddleUGV_6',
}

# --- Six MiddleUGV entity mappings (model_type 4) ---
UGV_MAPPINGS = [
    f'{i}|4|0|0|{i}|/sigma/camp_0/squad_0/MiddleUGV_{i}/model_info|/sigma/camp_0/squad_0/MiddleUGV_{i}/motion_command'
    for i in range(1, 7)
]

# Arrival-deceleration profile shared with the publisher (must match).
ARRIVAL_DECELERATION_DISTANCE_M = 40.0
ARRIVAL_MINIMUM_SPEED_MPS = 2.0

ADAPTER_PARAMETERS = {
    'enable_motion': False, 'enable_reference_trajectory': True,
    'formal_output_authorized': True,
    'entity_mappings_confirmed': True, 'coordinate_transform_confirmed': True,
    'dry_run_only': False, 'require_complete_swarm': True,
    'state_freshness_s': 0.5,
    'reference_trajectory_speed_mps': 5.0,
    'coordinated_arrival_enabled': True,
    'arrival_deceleration_enabled': True,
    'arrival_deceleration_distance_m': ARRIVAL_DECELERATION_DISTANCE_M,
    'arrival_minimum_speed_mps': ARRIVAL_MINIMUM_SPEED_MPS,
    'reference_trajectory_input_topics': [
        f'{i}|/hera/algorithm/ugv{i}/path' for i in range(1, 7)],
    'map_translation_x': 0.0, 'map_translation_y': 0.0,
    'map_translation_z': 0.0, 'map_yaw_rad': 0.0,
    'ugv_mappings': UGV_MAPPINGS,
}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'advance_distance_m', default_value='100.0',
            description='Eastward advance distance per vehicle, metres'),
        DeclareLaunchArgument(
            'reference_speed_mps', default_value='5.0',
            description='Cruise speed used by the publisher (m/s)'),
        Node(
            package='sigma2ros_bridge', executable='sigma2ros_bridge_node',
            name='sigma2ros_bridge', output='screen',
            parameters=[SDK_PARAMETERS]),
        Node(
            package='hera_sigma_bridge', executable='sigma_ros_adapter.py',
            name='sigma_ros_adapter', output='screen',
            parameters=[ADAPTER_PARAMETERS]),
        Node(
            package='hera_forward', executable='six_forward_100m',
            name='six_forward_100m', output='screen',
            parameters=[{
                'advance_distance_m': LaunchConfiguration('advance_distance_m'),
                'reference_speed_mps': LaunchConfiguration('reference_speed_mps'),
                'arrival_deceleration_distance_m': ARRIVAL_DECELERATION_DISTANCE_M,
                'arrival_minimum_speed_mps': ARRIVAL_MINIMUM_SPEED_MPS,
            }]),
    ])
