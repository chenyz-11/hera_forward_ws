"""Offline checks for the synchronized 45 degree turn."""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from builtin_interfaces.msg import Time
from hera_forward.formation_advance_turn_publisher import (
    FormationAdvanceTurnPublisher as Publisher, ROBOT_IDS, _arc_length)
from hera_forward.formation_switch_logic import advance_turn_stage_targets, formation_templates
from hera_sigma_bridge.sigma_adapter_logic import Transform
from hera_sigma_bridge.synchronized_trajectory import convert_synchronized_path


class Sink:
    def publish(self, message):
        self.message = message


class PublisherStub:
    _line = Publisher._line
    _push = staticmethod(Publisher._push)
    _append_arc = staticmethod(Publisher._append_arc)
    _append_angle = staticmethod(Publisher._append_angle)
    build_sub_segments = Publisher.build_sub_segments
    publish_stage = Publisher.publish_stage

    def all_states_fresh(self):
        return True

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: Time(sec=100, nanosec=0)))

    def get_logger(self):
        return SimpleNamespace(info=lambda message: None)


class AdvanceTurnTest(unittest.TestCase):
    def test_rigid_turn_speed_and_adapter_conversion(self):
        wide, _ = formation_templates(15.0, 20.0)
        poses = {i: (*wide[i], 0.0) for i in ROBOT_IDS}
        stages, slots, center = advance_turn_stage_targets(
            poses, 50.0, math.radians(45), 45.0, wide)
        node = PublisherStub()
        node.stages, node.slot_ids, node.turn_center = stages, slots, center
        node.turn_angle_deg, node.max_segment_m, node.reference_speed_mps = 45.0, 1.0, 5.0
        node.path_publishers = {r: Sink() for r in ROBOT_IDS}

        segments = node.build_sub_segments(1, stages[0]['goals'])
        turn, _, _, duration = segments[1]
        self.assertAlmostEqual(max(_arc_length(p) for p in turn.values()) / duration, 5.0)
        for k in range(len(turn[1])):
            for a in ROBOT_IDS:
                for b in range(a + 1, 7):
                    self.assertAlmostEqual(math.dist(turn[a][k][:2], turn[b][k][:2]),
                                           math.dist(turn[a][0][:2], turn[b][0][:2]))
        for r in ROBOT_IDS:
            self.assertAlmostEqual(math.dist(turn[r][-1][:2], stages[1]['goals'][r][:2]), 0.0)
            self.assertAlmostEqual(turn[r][-1][2], -math.pi / 4)

        with patch('hera_forward.formation_advance_turn_publisher.time.monotonic', return_value=250.0):
            total = node.publish_stage(1)
        self.assertAlmostEqual(node.stage_deadline, 250.0 + total)
        for r in ROBOT_IDS:
            message = node.path_publishers[r].message
            times = [p.header.stamp.sec * 10**9 + p.header.stamp.nanosec for p in message.poses]
            self.assertTrue(all(a < b for a, b in zip(times, times[1:])))
            candidate = convert_synchronized_path(message, Transform(0, 0, 0, 0), None)
            self.assertLessEqual(max(p.speed_ms for p in candidate.points), 5.00001)


if __name__ == '__main__':
    unittest.main()
