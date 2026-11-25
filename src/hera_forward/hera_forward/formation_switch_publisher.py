#!/usr/bin/env python3
"""Continuous formation-switch trajectory source.

The swarm forms ``wide_2x3`` while advancing, switches to ``single_column``
while still moving, then switches back to ``wide_2x3`` -- all along a single
continuous eastward trajectory.  All six vehicles share the same anchor
progress, so they keep their nominal separation (>= 12 m for the default 15 m /
20 m spacings) while the shape changes, instead of stopping to reconfigure.

The node reads only ``/ugvN/world_odom`` and publishes one timed ``Path`` per
vehicle to the Sigma trajectory adapter topics.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hera_sigma_bridge.bounded_trajectory import speed_profile

from hera_forward.formation_switch_logic import (
    build_continuous_formation_trajectory, formation_templates)


ROBOT_IDS = range(1, 7)
PATH_TOPICS = {robot: f'/hera/algorithm/ugv{robot}/path' for robot in ROBOT_IDS}
ODOM_FRESHNESS_S = 2.0


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


class FormationSwitchPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('formation_switch_publisher', **kwargs)

        self.advance_distance_m = self._positive('advance_distance_m', 100.0)
        self.wide_spacing_m = self._positive('wide_spacing_m', 15.0)
        self.column_spacing_m = self._positive('column_spacing_m', 20.0)
        self.reference_speed_mps = self._positive('reference_speed_mps', 20.0)
        self.sample_step_m = self._positive('sample_step_m', 2.0)
        self.hold_distance_m = self._nonnegative('hold_distance_m', 50.0)
        self.acceleration_mps2 = self._positive('acceleration_mps2', 0.4)
        self.deceleration_mps2 = self._positive('deceleration_mps2', 0.6)
        self.lateral_acceleration_mps2 = self._positive('lateral_acceleration_mps2', 0.6)

        self.wide_slots, self.column_slots = formation_templates(
            self.wide_spacing_m, self.column_spacing_m)

        self.published = False
        self.states, self.last_receipt = {}, {}

        self.path_publishers = {
            robot: self.create_publisher(Path, PATH_TOPICS[robot], 10)
            for robot in ROBOT_IDS}
        self.create_timer(0.1, self.tick)
        for robot in ROBOT_IDS:
            self.create_subscription(
                Odometry, f'/ugv{robot}/world_odom',
                lambda message, robot=robot: self.odom_callback(robot, message), 10)
        self.get_logger().info(
            'Waiting for six fresh odometry streams; then publishing one '
            'continuous form/switch/switch-back trajectory')

    def _positive(self, name, default):
        value = self.declare_parameter(name, default).value
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def _nonnegative(self, name, default):
        value = self.declare_parameter(name, default).value
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f'{name} must be finite and nonnegative')
        return value

    # ------------------------------------------------------------------
    # Odometry
    # ------------------------------------------------------------------
    def odom_callback(self, robot, message):
        position, orientation = message.pose.pose.position, message.pose.pose.orientation
        values = (position.x, position.y, yaw_from_quaternion(orientation))
        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(f'Ignoring non-finite odometry for UGV{robot}')
            return
        self.states[robot], self.last_receipt[robot] = values, time.monotonic()

    def all_states_fresh(self):
        now = time.monotonic()
        return all(robot in self.states
                   and now - self.last_receipt.get(robot, 0.0) <= ODOM_FRESHNESS_S
                   for robot in ROBOT_IDS)

    # ------------------------------------------------------------------
    # Publication
    # ------------------------------------------------------------------
    def publish_continuous(self):
        trajectories, slot_ids = build_continuous_formation_trajectory(
            self.states, self.advance_distance_m,
            self.wide_slots, self.column_slots, self.sample_step_m,
            self.hold_distance_m)

        # The anchor advances due east; sampling it identically and running it
        # through the bounded speed profile gives the shared time axis.  Because
        # every vehicle is keyed to the same anchor progress, using this time
        # axis for every vehicle keeps them synchronized in shape.
        n = len(trajectories[1])
        anchor_x0, anchor_y0 = self.states[1][0], self.states[1][1]
        anchor_points = [
            (anchor_x0 + i * self.sample_step_m, anchor_y0, 0.0) for i in range(n)]
        _, times = speed_profile(
            anchor_points, self.reference_speed_mps, 0.0,
            acceleration=self.acceleration_mps2,
            deceleration=self.deceleration_mps2,
            lateral_acceleration=self.lateral_acceleration_mps2,
            minimum_moving_speed=0.0)

        stamp = self.get_clock().now().to_msg()
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        for robot in ROBOT_IDS:
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = 'map'
            for point_index, (x, y, yaw) in enumerate(trajectories[robot]):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                offset_ns = round(times[point_index] * 1000000000)
                pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(
                    stamp_ns + offset_ns, 1000000000)
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path.poses.append(pose)
            self.path_publishers[robot].publish(path)

        total = 3.0 * self.advance_distance_m + self.hold_distance_m
        self.get_logger().info(
            f'Published continuous trajectory: advance={self.advance_distance_m:.0f}m x3 '
            f'+hold={self.hold_distance_m:.0f}m total={total:.0f}m '
            f'duration={times[-1]:.1f}s slot_ids={slot_ids}')

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def tick(self):
        if self.published:
            return
        if not self.all_states_fresh():
            return
        try:
            self.publish_continuous()
            self.published = True
        except (ValueError, OSError) as exc:
            self.get_logger().error(f'Cannot build continuous trajectory: {exc}')


def main():
    rclpy.init()
    node = FormationSwitchPublisher()
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
