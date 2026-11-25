#!/usr/bin/env python3
"""Damaged-node topology reconstruction demo publisher.

One node is damaged and stops; the remaining five keep advancing and smoothly
redistribute into a uniform ring to fill the gap (Chapter 5 idea).  The damaged
node never rejoins.  One continuous timed path per vehicle, published once.
"""

import math
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hera_forward.reconfiguration_logic import build_damaged_node_trajectory


ROBOT_IDS = range(1, 7)
PATH_TOPICS = {r: f'/hera/algorithm/ugv{r}/path' for r in ROBOT_IDS}
ODOM_FRESHNESS_S = 2.0
INITIAL_POSES_YAML = ('/home/cyz/hera_swarm_ws/src/hera_bringup/config/'
                      'sigma_fixed_six_initial_poses.yaml')


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_initial_poses(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return {int(v['name'][3:]): (float(v['x_m']), float(v['y_m']), float(v['yaw_rad']))
            for v in data['vehicles']}


class DamagedNodePublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('damaged_node_publisher', **kwargs)

        def pos(name, default):
            value = self.declare_parameter(name, default).value
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            return value

        self.initial_advance_m = pos('initial_advance_m', 50.0)
        self.reconfig_advance_m = pos('reconfig_advance_m', 35.0)
        self.final_advance_m = pos('final_advance_m', 50.0)
        self.reference_speed_mps = pos('reference_speed_mps', 5.0)
        self.step_m = pos('step_m', 1.0)
        self.initial_poses_yaml = self.declare_parameter(
            'initial_poses_yaml', INITIAL_POSES_YAML).value

        self.phase = 'WAIT_FOR_ODOMETRY'
        self.last_receipt = {}
        self.path_publishers = {r: self.create_publisher(Path, PATH_TOPICS[r], 10)
                                for r in ROBOT_IDS}
        self.create_timer(0.1, self.tick)
        for r in ROBOT_IDS:
            self.create_subscription(
                Odometry, f'/ugv{r}/world_odom',
                lambda message, robot=r: self.odom_callback(robot, message), 10)
        self.get_logger().info(
            'Waiting for six fresh odometry streams; then publishing one '
            'continuous damaged-node reconstruction trajectory per vehicle')

    def odom_callback(self, robot, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if all(math.isfinite(v) for v in (p.x, p.y, yaw_from_quaternion(q))):
            self.last_receipt[robot] = time.monotonic()

    def all_states_fresh(self):
        now = time.monotonic()
        return all(r in self.last_receipt and now - self.last_receipt[r] <= ODOM_FRESHNESS_S
                   for r in ROBOT_IDS)

    def publish_all(self):
        init = load_initial_poses(self.initial_poses_yaml)
        params = dict(
            initial_advance_m=self.initial_advance_m,
            reconfig_advance_m=self.reconfig_advance_m,
            final_advance_m=self.final_advance_m,
            reference_speed_mps=self.reference_speed_mps,
            step_m=self.step_m)
        trajectories, info = build_damaged_node_trajectory(init, params)

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
            f'Published ONE continuous damaged-node trajectory per vehicle: '
            f'distance={info["total_m"]:.0f}m duration={info["total_m"]/self.reference_speed_mps:.0f}s '
            f'damaged_ugv={info["damaged_robot"]} stops_at={info["damaged_stop_m"]:.0f}m '
            f'new_ring={info["new_ring_angles_deg"]}')
        return True

    def tick(self):
        if self.phase != 'WAIT_FOR_ODOMETRY':
            return
        if not self.all_states_fresh():
            return
        try:
            self.publish_all()
        except (ValueError, OSError, yaml.YAMLError) as exc:
            self.get_logger().error(f'Cannot publish trajectory: {exc}')
            self.phase = 'SAFETY_REJECTED'
            return
        self.phase = 'COMPLETE'


def main():
    rclpy.init()
    node = DamagedNodePublisher()
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
