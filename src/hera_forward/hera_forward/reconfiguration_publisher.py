#!/usr/bin/env python3
"""Chapter-4 reconfiguration demo publisher: ONE continuous trajectory per vehicle.

The full exit / reconfigure / rejoin / restore sequence is stitched into a single
timed ``nav_msgs/Path`` per vehicle and published once.  This avoids the staged
hand-off problem where the next stage overwrote a still-running stage and DVS
zero-endpoint speed made vehicles look frozen.

Vehicles start from their recorded poses (the initial formation).  The whole
sequence runs while advancing.
"""

import math
import os
import subprocess
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hera_forward.formation_switch_logic import normalize_angle
from hera_forward.reconfiguration_logic import build_flow_trajectory


ROBOT_IDS = range(1, 7)
PATH_TOPICS = {robot: f'/hera/algorithm/ugv{robot}/path' for robot in ROBOT_IDS}
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


class ReconfigurationPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('reconfiguration_publisher', **kwargs)

        def pos(name, default):
            value = self.declare_parameter(name, default).value
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            return value

        def pos_any(name, default):
            value = self.declare_parameter(name, default).value
            if not math.isfinite(value):
                raise ValueError(f'{name} must be finite')
            return value

        # Flow parameters (shared anchor progress -> always advancing).
        self.initial_advance_m = pos('initial_advance_m', 50.0)
        self.exit_drift_m = pos('exit_drift_m', 30.0)
        self.exit_offset_m = pos('exit_offset_m', 20.0)
        self.exit_long_lag_m = pos_any('exit_long_lag_m', 10.0)
        self.compact_hold_m = pos('compact_hold_m', 30.0)
        self.rejoin_drift_m = pos('rejoin_drift_m', 30.0)
        self.final_advance_m = pos('final_advance_m', 50.0)
        self.reference_speed_mps = pos('reference_speed_mps', 5.0)
        self.step_m = pos('step_m', 1.0)
        self.initial_poses_yaml = self.declare_parameter(
            'initial_poses_yaml', INITIAL_POSES_YAML).value

        self.phase = 'WAIT_FOR_ODOMETRY'
        self.last_receipt = {}
        self.path_publishers = {
            r: self.create_publisher(Path, PATH_TOPICS[r], 10) for r in ROBOT_IDS}
        self.create_timer(0.1, self.tick)
        for r in ROBOT_IDS:
            self.create_subscription(
                Odometry, f'/ugv{r}/world_odom',
                lambda message, robot=r: self.odom_callback(robot, message), 10)
        self.get_logger().info(
            'Waiting for six fresh odometry streams; then publishing one '
            'continuous Chapter-4 exit/reconfigure/rejoin trajectory per vehicle')

    def _dubins(self, start, goal):
        """Turn-radius-limited path via the external OMPL Dubins solver."""
        if (math.hypot(start[0] - goal[0], start[1] - goal[1]) < 1e-9 and
                abs(normalize_angle(start[2] - goal[2])) < 1e-9):
            return [start, goal]
        args = [self.dubins_executable] + [
            str(v) for v in (*start, *goal, self.turning_radius_m, self.step_m)]
        result = subprocess.run(args, capture_output=True, text=True, timeout=10.0, check=True)
        points = [tuple(map(float, line.split())) for line in result.stdout.splitlines()]
        if len(points) < 2 or any(len(p) != 3 for p in points):
            raise ValueError('invalid Dubins path output')
        return points

    def odom_callback(self, robot, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if all(math.isfinite(v) for v in (p.x, p.y, yaw_from_quaternion(q))):
            self.last_receipt[robot] = time.monotonic()

    def all_states_fresh(self):
        now = time.monotonic()
        return all(r in self.last_receipt and now - self.last_receipt[r] <= ODOM_FRESHNESS_S
                   for r in ROBOT_IDS)

    def publish_all(self):
        """Build the full plan from recorded initial poses and publish once."""
        init = load_initial_poses(self.initial_poses_yaml)
        params = dict(
            initial_advance_m=self.initial_advance_m,
            exit_drift_m=self.exit_drift_m,
            exit_offset_m=self.exit_offset_m,
            compact_hold_m=self.compact_hold_m,
            rejoin_drift_m=self.rejoin_drift_m,
            final_advance_m=self.final_advance_m,
            reference_speed_mps=self.reference_speed_mps,
            step_m=self.step_m)
        trajectories, info = build_flow_trajectory(init, params)
        total = info['total']

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
            f'Published ONE continuous advancing trajectory per vehicle: '
            f'distance={total:.1f}m duration={total / self.reference_speed_mps:.1f}s '
            f'npts_ugv1={len(trajectories[1])} exit_ugv={info["exit_robot"]} '
            f'ring_R={info["ring_radius"]:.1f}m exit_offset={info["exit_offset_m"]:.1f}m')
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
    node = ReconfigurationPublisher()
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
