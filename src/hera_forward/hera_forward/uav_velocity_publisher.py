#!/usr/bin/env python3
"""Feedback-based BigUAV velocity mission with persistent position/height hold.

Uses existing formation geometry and the existing Sigma adapter. No direct UDP
sender and no change to ground-vehicle algorithms.
"""
import bisect
import csv
import json
import math
from pathlib import Path
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from hera_forward.formation_switch_logic import formation_templates, build_continuous_formation_trajectory
from hera_forward.uav_feedback_logic import segment_clearance, tracking_velocity


class UavVelocityPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__('uav_velocity_publisher', **kwargs)
        def number(name, default):
            value = float(self.declare_parameter(name, default).value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
            return value
        self.ids = list(self.declare_parameter('robot_ids', [1, 2, 3, 4, 5, 6]).value)
        if not self.ids or len(set(self.ids)) != len(self.ids) or any(i not in range(1, 7) for i in self.ids):
            raise ValueError('robot_ids must be a nonempty unique subset of 1..6')
        self.mode = self.declare_parameter('mission_mode', 'formation').value
        if self.mode not in ('translate', 'formation') or (self.mode == 'formation' and len(self.ids) != 6):
            raise ValueError('formation needs all six; mission_mode must be translate or formation')
        self.hz = number('control_hz', 20.0)
        self.speed = number('cruise_speed_mps', 0.6)
        self.max_xy = number('max_horizontal_speed_mps', 1.0)
        self.max_z = number('max_vertical_speed_mps', 0.4)
        self.kp_xy = number('position_gain', 0.7)
        self.kp_z = number('height_gain', 0.35)
        self.height_damping = number('height_damping', 0.8)
        self.height = number('flight_height_m', 5.0)
        self.floor = number('min_height_m', 2.0)
        self.ceiling = number('max_height_m', 8.0)
        if not self.floor < self.height < self.ceiling or self.speed > self.max_xy:
            raise ValueError('inconsistent height bounds or cruise speed exceeds speed limit')
        self.fresh_s = number('state_freshness_s', 0.5)
        self.separation = number('min_separation_m', 3.0)
        self.stop_hold_s = number('stop_hold_s', 3.0)
        self.autostart = bool(self.declare_parameter('autostart', True).value)
        self.offset = tuple(float(self.declare_parameter(k, v).value) for k, v in
                            [('translate_x_m', 6.0), ('translate_y_m', 0.0)])
        if not all(math.isfinite(v) for v in self.offset) or math.hypot(*self.offset) > 200:
            raise ValueError('invalid translation')
        self.advance = number('advance_distance_m', 50.0)
        self.wide = number('wide_spacing_m', 15.0)
        self.column = number('column_spacing_m', 20.0)
        self.hold_distance = number('hold_distance_m', 50.0)
        # Accepted for old launches; flight now always uses measured z feedback.
        self.declare_parameter('auto_altitude', True)
        self.declare_parameter('climb_distance_m', 50.0)
        self.declare_parameter('initial_poses_yaml', '')
        path = self.declare_parameter('record_directory', '').value
        self.out = Path(path or '/home/cyz/hera_forward_ws/results/uav_motion_' + time.strftime('%Y%m%d_%H%M%S'))
        self.out.mkdir(parents=True, exist_ok=False)
        self.feedback_file = (self.out / 'feedback.csv').open('w', buffering=1)
        self.command_file = (self.out / 'commands.csv').open('w', buffering=1)
        self.feedback_writer = csv.writer(self.feedback_file)
        self.command_writer = csv.writer(self.command_file)
        self.feedback_writer.writerow(['receipt_s', 'robot', 'phase', 'x', 'y', 'z', 'reported_vx', 'reported_vy', 'reported_vz'])
        self.command_writer.writerow(['receipt_s', 'robot', 'phase', 'vx', 'vy', 'vz', 'target_x', 'target_y', 'target_z'])
        self.started = time.monotonic()
        self.last_tick = self.started
        self.states, self.receipts, self.targets = {}, {}, {}
        self.vertical_windows = {i: deque() for i in range(1, 7)}
        self.vertical_velocities = {i: 0.0 for i in range(1, 7)}
        self.before = {}
        self.phase, self.reason = 'WAIT_FOR_ODOMETRY', ''
        self.ready_since, self.settle_since = None, None
        self.plan_time = 0.0
        self.reference_index = 0
        self.last_status = 0.0
        self.pubs = {i: self.create_publisher(Twist, f'/hera/algorithm/uav{i}/velocity', 10) for i in self.ids}
        self.status_pub = self.create_publisher(String, '/hera/uav/status', 10)
        self.subs = [self.create_subscription(Odometry, f'/ugv{i}/world_odom',
                     lambda m, i=i: self.observe(i, m), 50) for i in range(1, 7)]
        self.control_services = [self.create_service(Trigger, '/hera/uav/' + name, callback) for name, callback in
                         [('start', self.start_service), ('hold', self.hold_service), ('land', self.land_service)]]
        self.create_timer(1.0 / self.hz, self.tick)
        self.get_logger().info(f'BigUAV feedback control: ids={self.ids}, mode={self.mode}, z={self.height}m; records={self.out}')

    def observe(self, robot, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        if not all(math.isfinite(x) for x in (p.x, p.y, p.z)):
            self.abort('nonfinite feedback')
            return
        now = time.monotonic()
        window = self.vertical_windows[robot]
        window.append((now, p.z))
        while len(window) > 2 and window[1][0] < now-.35:
            window.popleft()
        if len(window) > 1 and now-window[0][0] > .1:
            self.vertical_velocities[robot] = (p.z-window[0][1])/(now-window[0][0])
        self.states[robot] = (p.x, p.y, p.z)
        self.receipts[robot] = now
        self.feedback_writer.writerow([time.monotonic()-self.started, robot, self.phase, p.x, p.y, p.z, v.x, v.y, v.z])

    def fresh(self):
        now = time.monotonic()
        return len(self.states) == 6 and all(now-self.receipts[i] < self.fresh_s for i in range(1, 7))

    def capture_hold(self):
        self.targets = {i: (self.states[i][0], self.states[i][1], self.height) for i in self.ids if i in self.states}

    def abort(self, reason):
        if self.phase != 'SAFETY_HOLD':
            self.reason = reason
            self.capture_hold()
            self.phase = 'SAFETY_HOLD'
            self.get_logger().error(reason + '; mission latched off; bounded hold continues with fresh feedback')

    def build_plan(self):
        self.before = dict(self.states)
        if self.mode == 'translate':
            count = max(1, math.ceil(math.hypot(*self.offset)/0.05))
            self.paths = {i: [(self.states[i][0]+self.offset[0]*k/count,
                               self.states[i][1]+self.offset[1]*k/count, self.height)
                              for k in range(count+1)] for i in self.ids}
        else:
            wide, column = formation_templates(self.wide, self.column)
            paths, self.slot_ids = build_continuous_formation_trajectory(
                self.states, self.advance, wide, column, sample_step_m=0.25, hold_distance_m=self.hold_distance)
            self.paths = {i: [(x, y, self.height) for x, y, _ in paths[i]] for i in self.ids}
        count = len(self.paths[self.ids[0]])
        for k in range(count-1):
            for a in range(1, 7):
                for b in range(a+1, 7):
                    a0, a1 = (self.paths[a][k], self.paths[a][k+1]) if a in self.paths else (self.states[a], self.states[a])
                    b0, b1 = (self.paths[b][k], self.paths[b][k+1]) if b in self.paths else (self.states[b], self.states[b])
                    if segment_clearance(a0, a1, b0, b1) < self.separation:
                        raise ValueError('candidate path violates separation')
        self.times = [0.0]
        for k in range(count-1):
            length = max(math.dist(self.paths[i][k], self.paths[i][k+1]) for i in self.ids)
            self.times.append(self.times[-1]+max(1/self.hz, length/self.speed))
        self.plan_time = 0.0
        self.reference_index = 0
        (self.out/'mission_plan.json').write_text(json.dumps({
            'mode': self.mode, 'before': self.before, 'robot_ids': self.ids,
            'paths': self.paths, 'times_s': self.times,
            'slot_ids': getattr(self, 'slot_ids', {}), 'sample_step_m': .25,
            'advance_distance_m': self.advance, 'wide_spacing_m': self.wide,
            'column_spacing_m': self.column, 'hold_distance_m': self.hold_distance,
            'flight_height_m': self.height}, indent=2)+'\n')
        self.phase = 'PUBLISHING'
        self.get_logger().info(f'Mission started: duration >= {self.times[-1]:.1f}s; final position/height hold persists')

    def start_service(self, request, response):
        if self.phase not in ('READY', 'HOLD', 'COMPLETE') or not self.fresh():
            response.success, response.message = False, 'Need fresh six feedback streams and READY/HOLD/COMPLETE'
        else:
            try:
                self.build_plan()
                response.success, response.message = True, 'started'
            except ValueError as exc:
                self.abort(str(exc))
                response.success, response.message = False, str(exc)
        return response

    def hold_service(self, request, response):
        response.success = self.fresh() and self.phase not in ('LANDING_DISARM', 'LANDED')
        if response.success:
            self.capture_hold()
            self.phase = 'HOLD'
        response.message = 'holding position/height; keep bridge running' if response.success else 'feedback stale'
        return response

    def land_service(self, request, response):
        response.success = self.fresh() and bool(self.targets) and self.phase not in ('LANDING_DISARM', 'LANDED')
        if response.success:
            self.capture_hold()
            # The native BigUAV root rests at about 0.284m. A target below the
            # contact plane maintains a slow descent until actual contact.
            self.targets = {i: (v[0], v[1], 0.10) for i, v in self.targets.items()}
            self.phase = 'LANDING'
            self.settle_since = None
        response.message = 'controlled descent; wait for LANDED before stopping launch' if response.success else 'feedback/targets missing'
        return response

    def publish(self, robot, reference, feedforward=(0.0, 0.0, 0.0), zero=False):
        v = (0.0, 0.0, 0.0) if zero else tracking_velocity(
            self.states[robot], reference, feedforward, self.kp_xy, self.kp_z,
            self.max_xy, min(self.max_z, .3) if self.phase == 'LANDING' else self.max_z,
            self.vertical_velocities[robot], self.height_damping)
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = v
        self.pubs[robot].publish(msg)
        self.command_writer.writerow([time.monotonic()-self.started, robot, self.phase, *v, *reference])

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(.1, now-self.last_tick), now
        if not self.fresh():
            if self.phase in ('LANDING_DISARM', 'LANDED'):
                # Loss of feedback after ground contact must never turn the
                # output back on or command a climb to the cruise height.
                self.reason = 'grounded feedback stale; output remains off'
                self.write_status(now)
                return
            if self.targets:
                self.abort('one or more of six feedback streams became stale')
                for i in self.ids:
                    self.publish(i, self.targets.get(i, (0, 0, self.height)), zero=True)
            self.write_status(now)
            return
        if self.phase == 'WAIT_FOR_ODOMETRY':
            if any(not self.floor <= self.states[i][2] <= self.ceiling for i in self.ids):
                self.reason = 'waiting for valid airborne height in metres; check units/armed state'
                self.write_status(now)
                return
            self.capture_hold()
            self.phase, self.reason = 'PREPARE', ''
        if self.phase not in ('LANDING', 'LANDING_DISARM', 'LANDED', 'SAFETY_HOLD'):
            if any(not self.floor <= self.states[i][2] <= self.ceiling for i in self.ids):
                self.abort('aircraft exceeded height bounds')
            if any(math.dist(self.states[a][:2], self.states[b][:2]) < self.separation
                   for a in range(1, 7) for b in range(a+1, 7)):
                self.abort('live separation below minimum')
            if self.before and any(math.dist(self.states[i][:2], self.before[i][:2]) > .2
                                   for i in range(1, 7) if i not in self.ids):
                self.abort('uncommanded aircraft moved more than 0.2m')
        if self.phase == 'PREPARE':
            ready = all(abs(self.states[i][2]-self.height) < .12 for i in self.ids)
            self.ready_since = (self.ready_since or now) if ready else None
            if self.ready_since and now-self.ready_since >= 2:
                self.phase = 'READY'
                if self.autostart:
                    try:
                        self.build_plan()
                    except ValueError as exc:
                        self.abort(str(exc))
        feedforward = {}
        if self.phase == 'PUBLISHING':
            # Follow the precomputed time axis as-is: no tracking-error latch
            # and no error-dependent throttling of the reference progress.
            self.plan_time = min(self.times[-1], self.plan_time+dt)
            k = min(len(self.times)-2, bisect.bisect_right(self.times, self.plan_time)-1)
            self.reference_index = k
            duration = self.times[k+1]-self.times[k]
            fraction = (self.plan_time-self.times[k])/duration
            for i in self.ids:
                a, b = self.paths[i][k], self.paths[i][k+1]
                self.targets[i] = tuple(a[j]+fraction*(b[j]-a[j]) for j in range(3))
                feedforward[i] = tuple((b[j]-a[j])/duration for j in range(3))
            if self.plan_time >= self.times[-1]:
                self.phase, self.settle_since = 'STOPPING', None
                feedforward.clear()
        if self.phase == 'STOPPING':
            settled = all(math.dist(self.states[i][:2], self.targets[i][:2]) < .12
                          and abs(self.states[i][2]-self.height) < .12 for i in self.ids)
            self.settle_since = (self.settle_since or now) if settled else None
            if self.settle_since and now-self.settle_since >= self.stop_hold_s:
                self.phase = 'COMPLETE'
                self.get_logger().info('COMPLETE: position/height hold continues; no command timeout')
        if self.phase == 'LANDING':
            contact = all(self.states[i][2] < .34 for i in self.ids)
            self.settle_since = (self.settle_since or now) if contact else None
            if self.settle_since and now-self.settle_since >= 3:
                # Only after ground contact, let the platform's unchanged
                # command watchdog disarm. Sending zero would keep Offboard
                # armed and can lift the model off its contact plane again.
                self.phase, self.settle_since = 'LANDING_DISARM', now
                self.get_logger().info('Ground contact verified; releasing commands for platform disarm')
        if self.phase == 'LANDING_DISARM' and now-self.settle_since >= 5:
            if all(self.states[i][2] < .34 for i in self.ids):
                self.phase = 'LANDED'
                self.get_logger().info('LANDED: aircraft remain on ground; output is off; launch can be stopped')
            else:
                self.reason = 'ground contact was not maintained after command release'
        if self.phase in ('LANDING_DISARM', 'LANDED'):
            self.write_status(now)
            return
        for i in self.ids:
            if i in self.targets:
                self.publish(i, self.targets[i], feedforward.get(i, (0., 0., 0.)))
        self.write_status(now)

    def write_status(self, now):
        if now-self.last_status < .5:
            return
        self.last_status = now
        result = {'phase': self.phase, 'reason': self.reason, 'robot_ids': self.ids,
                  'elapsed_s': now-self.started, 'plan_time_s': self.plan_time,
                  'before': self.before, 'targets': self.targets, 'states': self.states,
                  'fresh': self.fresh(), 'flight_height_m': self.height,
                  'mission_mode': self.mode, 'record_directory': str(self.out)}
        progress = self.reference_index*.25
        result['reference_index'] = self.reference_index
        result['formation_stage'] = ('FORM_WIDE' if progress < self.advance else
                                     'TO_COLUMN' if progress < 2*self.advance else
                                     'RETURN_WIDE' if progress < 3*self.advance else 'ADVANCE_WIDE') if self.mode == 'formation' else 'TRANSLATE'
        temp = self.out/'status.tmp'
        temp.write_text(json.dumps(result, indent=2)+'\n')
        temp.replace(self.out/'status.json')
        msg = String()
        msg.data = json.dumps(result)
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.feedback_file.close()
        self.command_file.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = UavVelocityPublisher()
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
