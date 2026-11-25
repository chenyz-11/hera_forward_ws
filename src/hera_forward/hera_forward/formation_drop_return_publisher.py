#!/usr/bin/env python3
"""Drop-return-rejoin formation trajectory source (staged, time-driven).

Six stages published one after another, each its own timed ``nav_msgs/Path``:

  1. form         -- advance 50 m while forming wide_2x3.
  2. straight1    -- hold formation, advance 50 m.
  3. drop         -- NW vehicle stops; the other five advance 20 m.
  4. slow_return  -- five vehicles crawl 20 m at 2 m/s; SW vehicle turns back
                     to the NW vehicle's side (original slot5/slot6 pose).
  5. rejoin       -- the two stragglers accelerate into their original slots.
  6. straight2    -- full formation restored, advance another 50 m.

Stages advance on a wall-clock deadline (no odometry gating), and point
timestamps are shared per stage by arc-length proportion, so the trajectory is
gap-free between stages.
"""

import math
import os
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hera_forward.formation_switch_logic import (
    formation_templates, formation_targets, assign_slots, normalize_angle)
from hera_forward.formation_drop_return_logic import build_drop_return_plan


ROBOT_IDS = range(1, 7)
PATH_TOPICS = {robot: f'/hera/algorithm/ugv{robot}/path' for robot in ROBOT_IDS}
ODOM_FRESHNESS_S = 2.0


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def _arc_length(points):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))


class FormationDropReturnPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('formation_drop_return_publisher', **kwargs)

        self.form_distance_m = self._positive('form_distance_m', 50.0)
        self.straight_distance_m = self._positive('straight_distance_m', 50.0)
        self.drop_distance_m = self._positive('drop_distance_m', 20.0)
        self.slow_distance_m = self._positive('slow_distance_m', 20.0)
        self.slow_speed_mps = self._positive('slow_speed_mps', 2.0)
        self.rejoin_speed_mps = self._positive('rejoin_speed_mps', 5.0)
        self.final_distance_m = self._positive('final_distance_m', 50.0)
        self.reference_speed_mps = self._positive('reference_speed_mps', 5.0)
        self.wide_spacing_m = self._positive('wide_spacing_m', 15.0)
        self.max_segment_m = self._positive('max_segment_m', 1.0)
        self.turning_radius_m = self._positive('turning_radius_m', 4.5)

        default_dubins = os.path.join(
            get_package_prefix('hera_bringup'), 'lib', 'hera_bringup', 'sigma_dubins_path')
        self.dubins_executable = self.declare_parameter(
            'dubins_executable', default_dubins).value

        self.wide_slots, _ = formation_templates(self.wide_spacing_m, self.wide_spacing_m)
        self.stages = None
        self.info = None
        self.stage_index = 0
        self.stage_deadline = None
        self.phase = 'WAIT_FOR_ODOMETRY'
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
            'Waiting for six fresh odometry streams; then running '
            'form -> straight -> drop -> slow_return -> rejoin -> straight2')

    def _positive(self, name, default):
        value = self.declare_parameter(name, default).value
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
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
    # Dubins (SW vehicle turning back)
    # ------------------------------------------------------------------
    def _dubins(self, start, goal):
        if (math.hypot(start[0] - goal[0], start[1] - goal[1]) < 1e-9 and
                abs(normalize_angle(start[2] - goal[2])) < 1e-9):
            return [start, goal]
        args = [self.dubins_executable] + [
            str(v) for v in (*start, *goal, self.turning_radius_m, self.max_segment_m)]
        result = subprocess.run(args, capture_output=True, text=True, timeout=10.0, check=True)
        points = [tuple(map(float, line.split())) for line in result.stdout.splitlines()]
        if len(points) < 2 or any(len(p) != 3 for p in points):
            raise ValueError('invalid Dubins path output')
        return points

    def build_plan(self):
        anchor = (self.states[1][0], self.states[1][1], 0.0)
        form_targets = formation_targets(anchor[0] + self.form_distance_m,
                                         anchor[1], 0.0, self.wide_slots)
        slot_ids = assign_slots(self.states, form_targets)
        params = dict(
            form_distance_m=self.form_distance_m,
            straight_distance_m=self.straight_distance_m,
            drop_distance_m=self.drop_distance_m,
            slow_distance_m=self.slow_distance_m,
            slow_speed_mps=self.slow_speed_mps,
            rejoin_speed_mps=self.rejoin_speed_mps,
            final_distance_m=self.final_distance_m,
            reference_speed_mps=self.reference_speed_mps,
            max_segment_m=self.max_segment_m)
        return build_drop_return_plan(self.states, slot_ids, self.wide_slots, params, self._dubins)

    # ------------------------------------------------------------------
    # Publication
    # ------------------------------------------------------------------
    def publish_stage(self, index):
        stage = self.stages[index]
        name = stage['name']
        duration = stage['duration']
        stamp = self.get_clock().now().to_msg()
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec

        for robot in ROBOT_IDS:
            points = stage['paths'][robot]
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = 'map'
            total = _arc_length(points)
            cumulative = 0.0
            for i, (x, y, yaw) in enumerate(points):
                if i > 0:
                    cumulative += math.dist(points[i - 1][:2], (x, y))
                offset = duration * cumulative / total if total > 0 else (
                    duration * i / (len(points) - 1) if len(points) > 1 else 0.0)
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                offset_ns = round(offset * 1000000000)
                pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(
                    stamp_ns + offset_ns, 1000000000)
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path.poses.append(pose)
            self.path_publishers[robot].publish(path)

        self.get_logger().info(
            f'Published stage {index + 1}/{len(self.stages)}: {name} '
            f'duration={duration:.1f}s nw_ugv={self.info["nw_robot"]} '
            f'sw_ugv={self.info["sw_robot"]}')
        return duration

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def tick(self):
        now = time.monotonic()
        if self.phase == 'WAIT_FOR_ODOMETRY':
            if not self.all_states_fresh():
                return
            try:
                self.stages, self.info = self.build_plan()
            except (ValueError, subprocess.SubprocessError, OSError) as exc:
                self.get_logger().error(f'Cannot build plan: {exc}')
                self.phase = 'SAFETY_REJECTED'
                return
            self.stage_index = 0
            duration = self.publish_stage(0)
            self.stage_deadline = now + duration
            self.phase = 'PUBLISHING'
            return

        if self.phase == 'PUBLISHING' and now >= self.stage_deadline:
            next_index = self.stage_index + 1
            if next_index >= len(self.stages):
                self.phase = 'COMPLETE'
                self.get_logger().info('final stage reached; no further trajectories')
                return
            duration = self.publish_stage(next_index)
            self.stage_index = next_index
            self.stage_deadline = now + duration


def main():
    rclpy.init()
    node = FormationDropReturnPublisher()
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
