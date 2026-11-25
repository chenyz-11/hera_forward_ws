import json
import unittest

import rclpy
from nav_msgs.msg import Odometry

from hera_forward.formation_switch_logic import formation_templates
from hera_forward.recovery_preview_node import RecoveryPreview


class Sink:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class PreviewTest(unittest.TestCase):
    def test_waits_for_six_and_publishes_preview_only(self):
        rclpy.init()
        node = RecoveryPreview()
        try:
            node.paths = {r: Sink() for r in range(1, 7)}
            node.status = Sink()
            slots, _ = formation_templates(15.0, 15.0)
            for r in range(1, 6):
                msg = Odometry()
                msg.header.frame_id = 'map'
                msg.pose.pose.position.x, msg.pose.pose.position.y = slots[r]
                node.on_odom(r, msg)
            node.tick()
            self.assertFalse(node.done)
            msg.pose.pose.position.x, msg.pose.pose.position.y = slots[6]
            node.on_odom(6, msg)
            node.tick()
            status = json.loads(node.status.messages[-1].data)
            self.assertEqual(status['status'], 'PREVIEW_READY')
            self.assertFalse(status['motion_enabled'])
            self.assertEqual((status['dropout_robot'], status['rescue_robot']), (5, 6))
            for sink in node.paths.values():
                self.assertEqual(len(sink.messages), 1)
                self.assertEqual(len(sink.messages[0].poses), 7)
                self.assertEqual(sink.messages[0].header.frame_id, 'map')
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
