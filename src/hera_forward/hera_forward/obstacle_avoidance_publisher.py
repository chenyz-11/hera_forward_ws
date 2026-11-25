#!/usr/bin/env python3
"""Chapter-5 obstacle-avoidance demo publisher: ONE continuous path per vehicle.

Vehicles start from the config initial poses (a ring).  A rectangular obstacle
blocks the corridor 75 m east.  The plan shifts the ring's reference centre
north and contracts its scale before the obstacle, passes north of it, then
restores the initial ring -- all while advancing, stitched into one trajectory.
"""

import math
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hera_forward.obstacle_avoidance_logic import build_obstacle_avoidance


ROBOT_IDS = range(1, 7)
PATH_TOPICS = {r: f'/hera/algorithm/ugv{r}/path' for r in ROBOT_IDS}
ODOM_FRESHNESS_S = 2.0
INITIAL_POSES_YAML = ('/home/cyz/hera_forward_ws/src/hera_forward/config/'
                      'sigma_fixed_six_initial_poses.yaml')


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_initial_poses(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    return {int(v['name'][3:]): (float(v['x_m']), float(v['y_m']), float(v['yaw_rad']))
            for v in data['vehicles']}


class ObstacleAvoidancePublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('obstacle_avoidance_publisher', **kwargs)

        def pos(name, default):
            value = self.declare_parameter(name, default).value
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            return value

        def anyf(name, default):
            value = self.declare_parameter(name, default).value
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
            return value

        self.obstacle_east_m = pos('obstacle_east_m', 75.0)
        self.obstacle_half_width_y_m = pos('obstacle_half_width_y_m', 10.0)
        self.obstacle_half_depth_x_m = pos('obstacle_half_depth_x_m', 6.0)
        self.clearance_margin_m = pos('clearance_margin_m', 5.0)
        self.bypass_extra_m = pos('bypass_extra_m', 5.0)
        self.scale_min = pos('scale_min', 0.7)
        self.approach_m = pos('approach_m', 25.0)
        self.shift_m = pos('shift_m', 40.0)
        self.pass_m = pos('pass_m', 40.0)
        self.return_m = pos('return_m', 40.0)
        self.final_m = pos('final_m', 50.0)
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
            'continuous Chapter-5 obstacle-avoidance trajectory per vehicle')

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
            obstacle_east_m=self.obstacle_east_m,
            obstacle_half_width_y_m=self.obstacle_half_width_y_m,
            obstacle_half_depth_x_m=self.obstacle_half_depth_x_m,
            clearance_margin_m=self.clearance_margin_m,
            bypass_extra_m=self.bypass_extra_m,
            scale_min=self.scale_min,
            approach_m=self.approach_m,
            shift_m=self.shift_m,
            pass_m=self.pass_m,
            return_m=self.return_m,
            final_m=self.final_m,
            reference_speed_mps=self.reference_speed_mps,
            step_m=self.step_m)
        trajectories, info = build_obstacle_avoidance(init, params)

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
            f'Published ONE continuous obstacle-avoidance trajectory per vehicle: '
            f'distance={info["total_m"]:.0f}m duration={info["total_m"]/self.reference_speed_mps:.0f}s '
            f'bypass_y_offset={info["bypass_y_offset_m"]:.1f}m scale_min={info["scale_min"]:.2f} '
            f'obstacle_x={info["obstacle_center_x"]:.1f}m')
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
    node = ObstacleAvoidancePublisher()
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
