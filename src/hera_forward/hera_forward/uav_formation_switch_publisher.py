#!/usr/bin/env python3
"""UAV formation-switch publisher (direct precomputed paths, one-shot).

Six UAVs start from the config initial poses (a ring) at their fixed altitude
(~0.05 m; no height control).  The swarm forms wide_2x3 while advancing, then
switches to single_column, back to wide_2x3, and holds in wide_2x3 -- exactly
the staged UAV formation-switch process.  The geometry is computed offline from
the saved initial poses and emitted as ONE continuous timed nav_msgs/Path per
vehicle (no stage hand-off / odom gating), avoiding stage-overwrite stalls.

The adapter maps each UAV to a MiddleUAV entity and relays the paths to UE.
"""

import math
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hera_forward.formation_switch_logic import (
    formation_templates, build_continuous_formation_trajectory)
from hera_forward.reconfiguration_logic import dvs_quintic


ROBOT_IDS = range(1, 7)
PATH_TOPICS = {r: f'/hera/algorithm/uav{r}/path' for r in ROBOT_IDS}
ODOM_TOPICS = {r: f'/ugv{r}/world_odom' for r in ROBOT_IDS}  # adapter relays UAV state here
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


class UavFormationSwitchPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('uav_formation_switch_publisher', **kwargs)

        def pos(name, default):
            value = self.declare_parameter(name, default).value
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            return value

        def nonnegative(name, default):
            value = self.declare_parameter(name, default).value
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} must be finite and nonnegative')
            return value

        self.advance_distance_m = pos('advance_distance_m', 50.0)
        self.wide_spacing_m = pos('wide_spacing_m', 15.0)
        self.column_spacing_m = pos('column_spacing_m', 20.0)
        self.hold_distance_m = pos('hold_distance_m', 50.0)
        self.sample_step_m = pos('sample_step_m', 1.0)
        self.reference_speed_mps = pos('reference_speed_mps', 5.0)
        # Flight altitude for the trajectory (the config z ~=0.05 m is only the
        # ground spawn pose; UAV must climb to a cruise height to execute).
        self.flight_height_m = nonnegative('flight_height_m', 10.0)
        self.publish_period_s = pos('publish_period_s', 3.0)
        self.initial_poses_yaml = self.declare_parameter(
            'initial_poses_yaml', INITIAL_POSES_YAML).value

        self.phase = 'WAIT_FOR_ODOMETRY'
        self.last_receipt = {}
        self.publish_attempts = 0
        self.next_publish_at = 0.0
        self.path_publishers = {r: self.create_publisher(Path, PATH_TOPICS[r], 10)
                                for r in ROBOT_IDS}
        self.create_timer(0.1, self.tick)
        for r in ROBOT_IDS:
            self.create_subscription(
                Odometry, ODOM_TOPICS[r],
                lambda message, robot=r: self.odom_callback(robot, message), 10)
        self.get_logger().info(
            'Waiting for six fresh UAV odometry streams; then publishing one '
            'continuous wide->column->wide formation trajectory per UAV')

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
        wide, column = formation_templates(self.wide_spacing_m, self.column_spacing_m)
        trajectories, slot_ids = build_continuous_formation_trajectory(
            init, self.advance_distance_m, wide, column,
            sample_step_m=self.sample_step_m, hold_distance_m=self.hold_distance_m)

        stamp = self.get_clock().now().to_msg()
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        # Each trajectory point is sampled every sample_step_m of anchor progress;
        # time = accumulated distance / speed (x is the anchor advance).
        climb_end_idx = int(self.advance_distance_m / self.sample_step_m)
        for robot in ROBOT_IDS:
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = 'map'
            ground_z = init[robot][2]
            for k, (x, y, _yaw) in enumerate(trajectories[robot]):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                # Shared anchor timebase: sample k is at anchor s = k*step.
                t = k * self.sample_step_m / self.reference_speed_mps
                offset_ns = round(t * 1000000000)
                pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(
                    stamp_ns + offset_ns, 1000000000)
                pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                # Smooth climb from ground spawn to cruise altitude during form
                # stage (quintic S so vertical speed is zero at both ends).
                climb = dvs_quintic(min(1.0, k / climb_end_idx)) if climb_end_idx > 0 else 1.0
                pose.pose.position.z = float(ground_z + (self.flight_height_m - ground_z) * climb)
                pose.pose.orientation.w = 1.0      # ENU yaw 0 -> quat (0,0,0,1)
                path.poses.append(pose)
            self.path_publishers[robot].publish(path)

        total = self.advance_distance_m * 3 + self.hold_distance_m
        self.get_logger().info(
            f'Published one continuous wide->column->wide UAV trajectory per vehicle: '
            f'distance={total:.0f}m duration={total/self.reference_speed_mps:.0f}s '
            f'climb {init[1][2]:.3f}m->{self.flight_height_m:.1f}m slot_ids={slot_ids}')
        return True

    def tick(self):
        now = time.monotonic()
        if self.phase == 'WAIT_FOR_ODOMETRY':
            if not self.all_states_fresh():
                return
            # A one-shot VOLATILE publish can be lost if the adapter's
            # subscription is not matched yet.  Wait until every path topic has
            # at least one subscriber, then publish a short burst (3x) to be safe.
            if not all(self.path_publishers[r].get_subscription_count() > 0
                       for r in ROBOT_IDS):
                return
            try:
                self.publish_all()
            except (ValueError, OSError, yaml.YAMLError) as exc:
                self.get_logger().error(f'Cannot publish trajectory: {exc}')
                self.phase = 'SAFETY_REJECTED'
                return
            self.publish_attempts = 1
            self.next_publish_at = now + 1.5
            self.phase = 'REPUBLISHING'
            return

        if self.phase == 'REPUBLISHING' and now >= self.next_publish_at:
            # Keep republishing periodically forever.  The adapter may start
            # its path subscription late; a bounded burst can all be missed.
            # Repeated identical trajectories are idempotent for UE (each fully
            # replaces the previous one), so this guarantees eventual delivery.
            self.publish_all()
            self.publish_attempts += 1
            self.next_publish_at = now + self.publish_period_s


def main():
    rclpy.init()
    node = UavFormationSwitchPublisher()
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
