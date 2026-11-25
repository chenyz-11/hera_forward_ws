#!/usr/bin/env python3
"""Three-stage (staged) Sigma formation-switch trajectory source.

This is the earlier staged scheme kept alongside the continuous one: the swarm
forms ``wide_2x3`` while advancing, then switches to ``single_column``, then
switches back to ``wide_2x3`` -- each stage anchored further east.  Each stage is
published as one timed ``Path`` with a smooth bounded speed profile; a stage is
replaced once every vehicle is within ``stage_error_threshold_m`` of its slot.

The node reads only ``/ugvN/world_odom`` and writes timed map-frame ``Path``
messages to the Sigma trajectory adapter topics.
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

from hera_sigma_bridge.bounded_trajectory import (
    add_boundary_knots, coordinated_profiles)

from hera_forward.formation_switch_logic import (
    assign_slots, build_stage_plan, formation_targets, formation_templates,
    normalize_angle, targets_for_assignment)


ROBOT_IDS = range(1, 7)
PATH_TOPICS = {robot: f'/hera/algorithm/ugv{robot}/path' for robot in ROBOT_IDS}
ODOM_FRESHNESS_S = 2.0


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


class FormationSwitchStagedPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('formation_switch_staged_publisher', **kwargs)

        self.advance_distance_m = self._positive('advance_distance_m', 100.0)
        self.wide_spacing_m = self._positive('wide_spacing_m', 15.0)
        self.column_spacing_m = self._positive('column_spacing_m', 20.0)
        self.reference_speed_mps = self._positive('reference_speed_mps', 5.0)
        self.max_segment_m = self._positive('max_segment_m', 2.0)
        self.turning_radius_m = self._positive('turning_radius_m', 4.5)
        self.stage_error_threshold_m = self._positive('stage_error_threshold_m', 5.0)
        self.acceleration_mps2 = self._positive('acceleration_mps2', 0.4)
        self.deceleration_mps2 = self._positive('deceleration_mps2', 0.6)
        self.lateral_acceleration_mps2 = self._positive('lateral_acceleration_mps2', 0.6)

        default_dubins = os.path.join(
            get_package_prefix('hera_bringup'), 'lib', 'hera_bringup', 'sigma_dubins_path')
        self.dubins_executable = self.declare_parameter(
            'dubins_executable', default_dubins).value

        self.wide_slots, self.column_slots = formation_templates(
            self.wide_spacing_m, self.column_spacing_m)

        self.stage_plan = None
        self.stage_index = -1
        self.targets = None
        self.phase = 'WAIT_FOR_ODOMETRY'
        self.states, self.last_receipt = {}, {}
        self.last_stage_log = None

        self.path_publishers = {
            robot: self.create_publisher(Path, PATH_TOPICS[robot], 10)
            for robot in ROBOT_IDS}
        self.create_timer(0.1, self.tick)
        for robot in ROBOT_IDS:
            self.create_subscription(
                Odometry, f'/ugv{robot}/world_odom',
                lambda message, robot=robot: self.odom_callback(robot, message), 10)
        self.get_logger().info(
            'Waiting for six fresh odometry streams; then forming wide_2x3 while '
            'advancing, switching to single_column, and switching back to wide_2x3')

    def _positive(self, name, default):
        value = self.declare_parameter(name, default).value
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    # ------------------------------------------------------------------
    # Odometry / arrival
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

    def formation_arrived(self):
        """True once every vehicle is within the stage error threshold of its slot."""
        if self.targets is None or not self.all_states_fresh():
            return False
        return all(
            math.hypot(self.states[robot][0] - self.targets[robot][0],
                       self.states[robot][1] - self.targets[robot][1])
            < self.stage_error_threshold_m
            for robot in ROBOT_IDS)

    # ------------------------------------------------------------------
    # Path building
    # ------------------------------------------------------------------
    def _build_dubins_paths(self, starts):
        samples_by_robot = {}
        for robot in ROBOT_IDS:
            start, goal = starts[robot], self.targets[robot]
            if (math.hypot(start[0] - goal[0], start[1] - goal[1]) < 1e-9 and
                    abs(normalize_angle(start[2] - goal[2])) < 1e-9):
                samples_by_robot[robot] = [start, goal]
                continue
            args = [self.dubins_executable] + [
                str(v) for v in (*start, *goal, self.turning_radius_m, self.max_segment_m)]
            result = subprocess.run(
                args, capture_output=True, text=True, timeout=10.0, check=True)
            points = [tuple(map(float, line.split()))
                      for line in result.stdout.splitlines()]
            if len(points) < 2 or any(len(p) != 3 for p in points):
                raise ValueError(f'UGV{robot}: invalid Dubins path output')
            samples_by_robot[robot] = points
        return samples_by_robot

    # ------------------------------------------------------------------
    # Stage publication
    # ------------------------------------------------------------------
    def publish_stage(self, index):
        stage = self.stage_plan[index]
        name = stage['name']
        if not self.all_states_fresh():
            self.get_logger().error(f'Not publishing {name}: odometry is stale')
            return False

        # Later stages start from the previous stage's exact target slots.
        starts = self.states if index == 0 else self.targets

        slot_positions = formation_targets(*stage['anchor'], stage['slots'])
        assignment = assign_slots(starts, slot_positions)
        self.targets = targets_for_assignment(slot_positions, assignment)

        try:
            samples_by_robot = self._build_dubins_paths(starts)
            moving = {r: p for r, p in samples_by_robot.items()
                      if any(math.dist(a[:2], b[:2]) > 1e-9
                             for a, b in zip(p, p[1:]))}
            shaped = {r: add_boundary_knots(
                          samples_by_robot[r], self.reference_speed_mps, 0.0,
                          acceleration=self.acceleration_mps2,
                          deceleration=self.deceleration_mps2,
                          minimum_moving_speed=0.0)
                      for r in moving}
            profiles, common_duration = coordinated_profiles(
                shaped,
                {r: self.reference_speed_mps for r in moving},
                {r: 0.0 for r in moving},
                synchronize_arrivals=True,
                acceleration=self.acceleration_mps2,
                deceleration=self.deceleration_mps2,
                lateral_acceleration=self.lateral_acceleration_mps2,
                minimum_moving_speed=0.0)
            path_times = {r: profiles[r][1] for r in moving}
        except (ValueError, subprocess.SubprocessError, OSError) as exc:
            self.get_logger().error(f'Cannot build {name} paths: {exc}')
            self.phase = 'SAFETY_REJECTED'
            return False
        if not self.all_states_fresh():
            self.get_logger().error(f'{name} paths rejected: odometry stale during planning')
            self.phase = 'SAFETY_REJECTED'
            return False

        stamp = self.get_clock().now().to_msg()
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        for robot in ROBOT_IDS:
            if robot not in moving:
                self.get_logger().warning(
                    f'UGV{robot}: stationary at its slot; path not sent '
                    '(platform hold is not confirmed)')
                continue
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = 'map'
            times = path_times[robot]
            for point_index, (x, y, yaw) in enumerate(shaped[robot]):
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

        self.get_logger().info(
            f'Published stage {index + 1}/{len(self.stage_plan)}: {name} '
            f'(anchor=({stage["anchor"][0]:.2f}, {stage["anchor"][1]:.2f})); '
            f'duration={common_duration:.1f}s; '
            f'stage_error_threshold={self.stage_error_threshold_m:.2f}m')
        return True

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def tick(self):
        if self.phase == 'WAIT_FOR_ODOMETRY':
            if not self.all_states_fresh():
                return
            self.stage_plan = build_stage_plan(
                self.states[1], self.advance_distance_m,
                self.wide_slots, self.column_slots)
            self.stage_index = 0
            self.targets = None
            if self.publish_stage(0):
                self.phase = 'WAIT_FOR_ARRIVAL'
            return

        if self.phase == 'WAIT_FOR_ARRIVAL' and self.formation_arrived():
            next_index = self.stage_index + 1
            if next_index >= len(self.stage_plan):
                self.phase = 'COMPLETE'
                self.get_logger().info(
                    'wide_2x3_return settled; no further trajectories will be published')
            elif self.publish_stage(next_index):
                self.stage_index = next_index

        self.log_stage()

    def log_stage(self):
        now = time.monotonic()
        if self.last_stage_log is not None and now - self.last_stage_log < 1.0:
            return
        self.last_stage_log = now
        name = (self.stage_plan[self.stage_index]['name']
                if self.stage_plan is not None and self.stage_index >= 0 else '-')
        self.get_logger().info(
            f'STAGED_STAGE phase={self.phase} '
            f'stage={self.stage_index + 1}/{len(self.stage_plan) if self.stage_plan else 0} '
            f'name={name}')


def main():
    rclpy.init()
    node = FormationSwitchStagedPublisher()
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
