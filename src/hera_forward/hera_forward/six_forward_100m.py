#!/usr/bin/env python3
"""Publish six ~100 m formation-advance paths from the recorded initial poses.

Each vehicle advances from its recorded scene pose, keeping the formation, and
the six timed paths are handed to ``sigma_ros_adapter`` which converts them to
``ReferenceTrajectory`` messages for ``sigma2ros_bridge``.  The timing matches
the shared arrival-deceleration profile so the swarm slows and stops at the
advance distance instead of continuing past it.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hera_sigma_bridge.arrival_profile import coordinated_arrival_times

from hera_forward.forward_logic import advance_goals, line_samples, load_initial_poses


ROBOT_IDS = range(1, 7)
PATH_TOPICS = {robot: f'/hera/algorithm/ugv{robot}/path' for robot in ROBOT_IDS}

# The single source of truth for the platform's current six-vehicle poses.
DEFAULT_INITIAL_POSES_FILE = (
    '/home/cyz/hera_swarm_ws/src/hera_bringup/config/sigma_fixed_six_initial_poses.yaml')


class SixForwardPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('six_forward_100m', **kwargs)

        initial_poses_file = self.declare_parameter(
            'initial_poses_file', DEFAULT_INITIAL_POSES_FILE).value
        self.advance_distance_m = self.declare_parameter('advance_distance_m', 100.0).value
        self.reference_speed_mps = self.declare_parameter('reference_speed_mps', 5.0).value
        self.arrival_deceleration_distance_m = self.declare_parameter(
            'arrival_deceleration_distance_m', 40.0).value
        self.arrival_minimum_speed_mps = self.declare_parameter(
            'arrival_minimum_speed_mps', 2.0).value
        self.max_segment_m = self.declare_parameter('max_segment_m', 2.0).value
        self.republish_period_s = self.declare_parameter('republish_period_s', 5.0).value

        self._validate_parameters()

        self.initial_poses = load_initial_poses(initial_poses_file)
        self.goals = advance_goals(self.initial_poses, self.advance_distance_m)
        self.samples_by_robot = {
            robot: line_samples(self.initial_poses[robot][:3], self.goals[robot][:3],
                                self.max_segment_m)
            for robot in ROBOT_IDS}

        # All six vehicles share the same cruise speed and straight length, so
        # coordinated_arrival_times collapses to one profile; it is kept for
        # correctness if the distances ever diverge.
        self.path_times, self.common_duration = coordinated_arrival_times(
            self.samples_by_robot,
            {robot: self.reference_speed_mps for robot in ROBOT_IDS},
            self.arrival_deceleration_distance_m,
            self.arrival_minimum_speed_mps)

        self.path_publishers = {
            robot: self.create_publisher(Path, PATH_TOPICS[robot], 10)
            for robot in ROBOT_IDS}

        self.publish_paths()
        self.create_timer(self.republish_period_s, self.publish_paths)
        self.get_logger().info(
            f'Published six {self.advance_distance_m:.1f} m advance paths '
            f'(duration={self.common_duration:.3f}s, cruise={self.reference_speed_mps:.1f}m/s)')

    def _validate_parameters(self):
        if not math.isfinite(self.advance_distance_m) or self.advance_distance_m <= 0.0:
            raise ValueError('advance_distance_m must be finite and positive')
        if not math.isfinite(self.reference_speed_mps) or self.reference_speed_mps <= 0.0:
            raise ValueError('reference_speed_mps must be finite and positive')
        if not math.isfinite(self.arrival_deceleration_distance_m) or \
                self.arrival_deceleration_distance_m <= 0.0:
            raise ValueError('arrival_deceleration_distance_m must be finite and positive')
        if not math.isfinite(self.arrival_minimum_speed_mps) or \
                self.arrival_minimum_speed_mps <= 0.0:
            raise ValueError('arrival_minimum_speed_mps must be finite and positive')
        if self.arrival_minimum_speed_mps > self.reference_speed_mps:
            raise ValueError('arrival_minimum_speed_mps must not exceed reference_speed_mps')
        if not math.isfinite(self.max_segment_m) or self.max_segment_m <= 0.0:
            raise ValueError('max_segment_m must be finite and positive')
        if not math.isfinite(self.republish_period_s) or self.republish_period_s <= 0.0:
            raise ValueError('republish_period_s must be finite and positive')

    def publish_paths(self):
        stamp = self.get_clock().now().to_msg()
        start_ns = stamp.sec * 1000000000 + stamp.nanosec
        for robot in ROBOT_IDS:
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = 'map'
            z = self.initial_poses[robot][2]
            samples = self.samples_by_robot[robot]
            for index, (x, y, yaw) in enumerate(samples):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                point_ns = start_ns + round(self.path_times[robot][index] * 1000000000)
                pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(point_ns, 1000000000)
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                pose.pose.position.z = float(z)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path.poses.append(pose)
            self.path_publishers[robot].publish(path)


def main():
    rclpy.init()
    node = SixForwardPublisher()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
