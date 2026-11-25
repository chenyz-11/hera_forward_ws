#!/usr/bin/env python3
"""Formation advance-and-turn trajectory source (staged, time-driven hand-off).

Three stages are published one after another, each as its own timed
``nav_msgs/Path``:

  1. ``form``         -- form ``wide_2x3`` while advancing (Dubins paths).
  2. ``advance_turn`` -- hold the formation, advance straight 50 m, then turn
                         right 45 deg by default as a rigid body (outer vehicles faster,
                         inner slower).
  3. ``straight``     -- hold the turned formation and advance another 50 m.

The next stage is published once the previous stage's *planned* duration has
elapsed (a monotonic-clock deadline measured from publication). No
odometry-driven arrival check is used, which removes the earlier problem of
stages being skipped when odometry readback reported the vehicles as already
arrived.

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

from hera_forward.formation_switch_logic import (
    advance_turn_stage_targets, formation_templates, normalize_angle,
    rigid_turn_points)


ROBOT_IDS = range(1, 7)
PATH_TOPICS = {robot: f'/hera/algorithm/ugv{robot}/path' for robot in ROBOT_IDS}
ODOM_FRESHNESS_S = 2.0


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def _arc_length(points):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))


class FormationAdvanceTurnPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('formation_advance_turn_publisher', **kwargs)

        self.advance_distance_m = self._positive('advance_distance_m', 50.0)
        self.wide_spacing_m = self._positive('wide_spacing_m', 15.0)
        self.turn_angle_deg = self._positive('turn_angle_deg', 45.0)
        self.turn_radius_m = self._positive('turn_radius_m', 45.0)
        self.turning_radius_m = self._positive('turning_radius_m', 4.5)
        self.reference_speed_mps = self._positive('reference_speed_mps', 5.0)
        self.max_segment_m = self._positive('max_segment_m', 1.0)

        default_dubins = os.path.join(
            get_package_prefix('hera_bringup'), 'lib', 'hera_bringup', 'sigma_dubins_path')
        self.dubins_executable = self.declare_parameter(
            'dubins_executable', default_dubins).value

        self.wide_slots, _ = formation_templates(self.wide_spacing_m, self.wide_spacing_m)

        self.stages = None
        self.slot_ids = None
        self.turn_center = None
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
            'Waiting for six fresh odometry streams; then publishing staged '
            'form -> advance+turn -> straight trajectories')

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
    # Segment geometry
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

    def _line(self, start, goal):
        distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
        count = max(2, int(math.ceil(distance / self.max_segment_m)) + 1)
        return [(start[0] + (goal[0] - start[0]) * k / (count - 1),
                 start[1] + (goal[1] - start[1]) * k / (count - 1),
                 goal[2] if k == count - 1 else start[2])
                for k in range(count)]

    # ------------------------------------------------------------------
    # Timed sub-segments
    # ------------------------------------------------------------------
    @staticmethod
    def _push(pts, point, t):
        if pts and t <= pts[-1][3] + 1e-9:
            pts[-1] = (point[0], point[1], point[2], pts[-1][3])
        else:
            pts.append((point[0], point[1], point[2], t))

    @staticmethod
    def _append_arc(pts, segment, t0, duration):
        segment_len = _arc_length(segment)
        cumulative = 0.0
        for i, point in enumerate(segment):
            if i > 0:
                cumulative += math.dist(segment[i - 1][:2], point[:2])
            t = t0 + (duration * cumulative / segment_len if segment_len > 0 else 0.0)
            FormationAdvanceTurnPublisher._push(pts, point, t)

    @staticmethod
    def _append_angle(pts, segment, t0, duration):
        count = max(1, len(segment) - 1)
        for k, point in enumerate(segment):
            FormationAdvanceTurnPublisher._push(pts, point, t0 + duration * k / count)

    def build_sub_segments(self, index, starts):
        """Return ``[(paths_by_robot, time_mode, t0, duration), ...]`` for a stage."""
        stage = self.stages[index]
        name = stage['name']
        speed = self.reference_speed_mps
        if name == 'form':
            paths = {r: self._dubins(starts[r], stage['goals'][r]) for r in ROBOT_IDS}
            duration = max(_arc_length(paths[r]) for r in ROBOT_IDS) / speed
            return [(paths, 'arc', 0.0, duration)]
        if name == 'advance_turn':
            straight = {r: self._line(starts[r], stage['straight_goals'][r]) for r in ROBOT_IDS}
            d_straight = max(_arc_length(straight[r]) for r in ROBOT_IDS) / speed
            turn = rigid_turn_points(stage['straight_goals'], self.turn_center,
                                     math.radians(self.turn_angle_deg),
                                     self.max_segment_m)
            # Equal angular progress preserves the planned formation; the
            # longest arc sets the duration so no vehicle exceeds the cap.
            d_turn = max(_arc_length(turn[r]) for r in ROBOT_IDS) / speed
            return [(straight, 'arc', 0.0, d_straight),
                    (turn, 'angle', d_straight, d_turn)]
        if name == 'straight':
            paths = {r: self._line(starts[r], stage['goals'][r]) for r in ROBOT_IDS}
            duration = max(_arc_length(paths[r]) for r in ROBOT_IDS) / speed
            return [(paths, 'arc', 0.0, duration)]
        raise ValueError(f'unknown stage {name}')

    def publish_stage(self, index):
        stage = self.stages[index]
        name = stage['name']
        if not self.all_states_fresh():
            self.get_logger().error(f'Not publishing {name}: odometry is stale')
            return False
        starts = self.states if index == 0 else self.stages[index - 1]['goals']
        try:
            sub_segments = self.build_sub_segments(index, starts)
        except (ValueError, subprocess.SubprocessError, OSError) as exc:
            self.get_logger().error(f'Cannot build {name} paths: {exc}')
            self.phase = 'SAFETY_REJECTED'
            return False

        trajectories = {r: [] for r in ROBOT_IDS}
        total = 0.0
        for paths, mode, t0, duration in sub_segments:
            total = max(total, t0 + duration)
            for robot in ROBOT_IDS:
                if mode == 'arc':
                    self._append_arc(trajectories[robot], paths[robot], t0, duration)
                else:
                    self._append_angle(trajectories[robot], paths[robot], t0, duration)

        publication_time = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        for robot in ROBOT_IDS:
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = 'map'
            for (x, y, yaw, t) in trajectories[robot]:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                offset_ns = round(t * 1000000000)
                pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(
                    stamp_ns + offset_ns, 1000000000)
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path.poses.append(pose)
            self.path_publishers[robot].publish(path)

        self.get_logger().info(
            f'Published stage {index + 1}/{len(self.stages)}: {name} '
            f'duration={total:.1f}s slot_ids={self.slot_ids}')
        self.stage_deadline = publication_time + total
        return total

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def tick(self):
        now = time.monotonic()
        if self.phase == 'WAIT_FOR_ODOMETRY':
            if not self.all_states_fresh():
                return
            self.stages, self.slot_ids, self.turn_center = advance_turn_stage_targets(
                self.states, self.advance_distance_m,
                math.radians(self.turn_angle_deg), self.turn_radius_m,
                self.wide_slots)
            self.stage_index = 0
            duration = self.publish_stage(0)
            if duration:
                self.phase = 'PUBLISHING'
            return

        if self.phase == 'PUBLISHING' and now >= self.stage_deadline:
            next_index = self.stage_index + 1
            if next_index >= len(self.stages):
                self.phase = 'COMPLETE'
                self.get_logger().info('final stage reached; no further trajectories')
                return
            duration = self.publish_stage(next_index)
            if duration:
                self.stage_index = next_index


def main():
    rclpy.init()
    node = FormationAdvanceTurnPublisher()
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
