#!/usr/bin/env python3
"""UAV formation switch via motion_command (single waypoint navigation).

Same wide->column->wide geometry as the trajectory version, but emitted through
the adapter's motion_command channel: a stream of ``/ugvN/way_point``
(PointStamped) targets plus a constant ``/ugvN/aede_speed`` (Float32).  The
adapter converts each waypoint to a Sigma MotionCommand (controlMode=1, target
point navigation) and sends it to UE.  Used to test whether the platform
responds to motion_command where it ignores reference_trajectory for UAVs.
"""

import math
import time

import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

from hera_forward.formation_switch_logic import (
    formation_templates, build_continuous_formation_trajectory)
from hera_forward.reconfiguration_logic import dvs_quintic


ROBOT_IDS = range(1, 7)
ODOM_TOPICS = {r: f'/ugv{r}/world_odom' for r in ROBOT_IDS}
ODOM_FRESHNESS_S = 2.0
INITIAL_POSES_YAML = ('/home/cyz/hera_forward_ws/src/hera_forward/config/'
                      'sigma_uav_only_initial_poses.yaml')


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_initial_poses(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return {int(v['name'][3:]): (float(v['x_m']), float(v['y_m']), float(v['z_m']))
            for v in data['vehicles']}


class UavMotionPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('uav_motion_publisher', **kwargs)

        def pos(name, default):
            value = self.declare_parameter(name, default).value
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            return value

        self.advance_distance_m = pos('advance_distance_m', 50.0)
        self.wide_spacing_m = pos('wide_spacing_m', 15.0)
        self.column_spacing_m = pos('column_spacing_m', 20.0)
        self.hold_distance_m = pos('hold_distance_m', 50.0)
        self.waypoint_step_m = pos('waypoint_step_m', 5.0)
        self.waypoint_interval_s = pos('waypoint_interval_s', 0.4)
        self.reference_speed_mps = pos('reference_speed_mps', 5.0)
        self.flight_height_m = self.declare_parameter('flight_height_m', 10.0).value
        self.initial_poses_yaml = self.declare_parameter(
            'initial_poses_yaml', INITIAL_POSES_YAML).value

        self.phase = 'WAIT_FOR_ODOMETRY'
        self.last_receipt = {}
        self.waypoints = None
        self.wp_index = 0
        self.next_publish_at = 0.0

        self.waypoint_pubs = {r: self.create_publisher(
            PointStamped, f'/ugv{r}/way_point', 10) for r in ROBOT_IDS}
        self.speed_pubs = {r: self.create_publisher(
            Float32, f'/ugv{r}/aede_speed', 10) for r in ROBOT_IDS}
        self.create_timer(0.05, self.tick)
        for r in ROBOT_IDS:
            self.create_subscription(
                Odometry, ODOM_TOPICS[r],
                lambda message, robot=r: self.odom_callback(robot, message), 10)
        self.get_logger().info(
            'Waiting for six fresh UAV odometry streams; then streaming '
            'wide->column->wide waypoints via motion_command')

    def odom_callback(self, robot, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if all(math.isfinite(v) for v in (p.x, p.y, yaw_from_quaternion(q))):
            self.last_receipt[robot] = time.monotonic()

    def all_states_fresh(self):
        now = time.monotonic()
        return all(r in self.last_receipt and now - self.last_receipt[r] <= ODOM_FRESHNESS_S
                   for r in ROBOT_IDS)

    def build_waypoints(self):
        init = load_initial_poses(self.initial_poses_yaml)
        wide, column = formation_templates(self.wide_spacing_m, self.column_spacing_m)
        trajectories, slot_ids = build_continuous_formation_trajectory(
            init, self.advance_distance_m, wide, column,
            sample_step_m=self.waypoint_step_m, hold_distance_m=self.hold_distance_m)
        climb_end = int(self.advance_distance_m / self.waypoint_step_m)
        waypoints = {r: [] for r in ROBOT_IDS}
        for r in ROBOT_IDS:
            ground_z = init[r][2]
            for k, (x, y, _yaw) in enumerate(trajectories[r]):
                climb = dvs_quintic(min(1.0, k / climb_end)) if climb_end > 0 else 1.0
                z = ground_z + (self.flight_height_m - ground_z) * climb
                waypoints[r].append((x, y, z))
        return waypoints, init

    def publish_speeds(self):
        for r in ROBOT_IDS:
            self.speed_pubs[r].publish(Float32(data=float(self.reference_speed_mps)))

    def tick(self):
        now = time.monotonic()
        if self.phase == 'WAIT_FOR_ODOMETRY':
            if not self.all_states_fresh():
                return
            try:
                self.waypoints, init = self.build_waypoints()
            except (ValueError, OSError, yaml.YAMLError) as exc:
                self.get_logger().error(f'Cannot build waypoints: {exc}')
                self.phase = 'SAFETY_REJECTED'
                return
            self.publish_speeds()
            self.wp_index = 0
            self.next_publish_at = now
            self.phase = 'PUBLISHING'
            self.get_logger().info(
                f'Streaming {len(self.waypoints[1])} waypoints per UAV via '
                f'motion_command (step={self.waypoint_step_m:.0f}m '
                f'interval={self.waypoint_interval_s:.2f}s z->{self.flight_height_m}m)')
            return

        if self.phase == 'PUBLISHING' and now >= self.next_publish_at:
            idx = self.wp_index
            n = len(self.waypoints[1])
            stamp = self.get_clock().now().to_msg()
            for r in ROBOT_IDS:
                x, y, z = self.waypoints[r][idx]
                m = PointStamped()
                m.header.stamp = stamp
                m.header.frame_id = 'map'
                m.point.x, m.point.y, m.point.z = float(x), float(y), float(z)
                self.waypoint_pubs[r].publish(m)
            # keep speed command refreshed periodically (adapter caches it)
            self.publish_speeds()
            self.wp_index += 1
            self.next_publish_at = now + self.waypoint_interval_s
            if self.wp_index >= n:
                self.phase = 'COMPLETE'
                self.get_logger().info('waypoint stream complete')


def main():
    rclpy.init()
    node = UavMotionPublisher()
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
