#!/usr/bin/env python3
"""Build a dropout/recovery preview from six live poses; no motion output."""
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from hera_forward.recovery_scenario import make_scenario


class RecoveryPreview(Node):
    def __init__(self):
        super().__init__('recovery_preview')
        self.freshness = self._positive('odom_freshness_s', 1.0)
        self.options = {name: self._positive(name, default) for name, default in (
            ('spacing', 15.0), ('formation_advance', 50.0), ('initial_advance', 50.0),
            ('separation_advance', 20.0), ('final_advance', 50.0),
            ('cruise_speed', 3.0), ('slow_speed', 1.0), ('rescue_speed', 3.0),
            ('catchup_speed', 4.0), ('minimum_distance', 5.0))}
        self.states = {}
        self.receipts = {}
        self.done = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.paths = {r: self.create_publisher(Path, f'/hera/recovery/preview/ugv{r}/path', qos)
                      for r in range(1, 7)}
        self.status = self.create_publisher(String, '/hera/recovery/preview/status', qos)
        self.subscriptions_ = [self.create_subscription(
            Odometry, f'/ugv{r}/world_odom', lambda msg, robot=r: self.on_odom(robot, msg), 10)
            for r in range(1, 7)]
        self.create_timer(0.2, self.tick)
        self.get_logger().info('PREVIEW ONLY: waiting for six fresh odometry streams; no motion output')

    def _positive(self, name, default):
        value = float(self.declare_parameter(name, default).value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be finite and positive')
        return value

    def on_odom(self, robot, message):
        p = message.pose.pose.position
        if message.header.frame_id != 'map' or not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            return
        self.states[robot] = (p.x, p.y, p.z)
        self.receipts[robot] = time.monotonic()

    def emit_status(self, data):
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False, allow_nan=False)
        self.status.publish(msg)

    def tick(self):
        if self.done:
            return
        now = time.monotonic()
        if not all(r in self.states and now - self.receipts[r] <= self.freshness
                   for r in range(1, 7)):
            return
        try:
            plan = make_scenario(self.states, **self.options)
        except ValueError as exc:
            self.emit_status({'status': 'REJECTED', 'reason': str(exc), 'motion_enabled': False})
            self.get_logger().error(f'PREVIEW REJECTED: {exc}')
            self.done = True
            return
        stamp = self.get_clock().now().to_msg()
        epoch_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        for robot in range(1, 7):
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = stamp
            # Orientation here is display-only; this polyline has instantaneous
            # velocity reversals and must never be treated as a vehicle path.
            for index, t in enumerate(plan['times_s']):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(
                    epoch_ns + round(t * 1_000_000_000), 1_000_000_000)
                pose.pose.position.x, pose.pose.position.y = plan['positions'][index][robot]
                pose.pose.position.z = self.states[robot][2]
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.paths[robot].publish(path)
        result = {'status': 'PREVIEW_READY', 'motion_enabled': False,
                  'warning': 'geometric preview only; no platform stop or dynamic feasibility',
                  'dropout_robot': plan['dropout'], 'rescue_robot': plan['rescuer'],
                  'assignment': plan['assignment'], 'phases': plan['phases'],
                  'times_s': plan['times_s'], 'minimum_separation_m': plan['minimum_separation'][0],
                  'closest_pair': plan['minimum_separation'][1]}
        self.emit_status(result)
        self.get_logger().info('PREVIEW_READY ' + json.dumps(result, ensure_ascii=False))
        self.done = True


def main():
    rclpy.init()
    node = RecoveryPreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
