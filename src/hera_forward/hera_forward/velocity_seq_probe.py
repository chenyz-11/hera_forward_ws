#!/usr/bin/env python3
"""Probe: send raw DXFD VELOCITY_COMMAND with incrementing seq to UE, read z.

Bypasses bridge entirely for SENDING (bridge still runs for the model_info pull
pump).  Tests whether UE needs an incrementing seq to accept repeated commands.
"""
import argparse
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from sigma2ros_bridge.msg import ModelInfo

HEAD = 0xDF
CTRL = 255
LOWER = 1
MSG_VEL = 37
# int64 entity_id, uint8 flags, uint16 seq, 9 floats
FMT = '<qBH8f'


def build(entity_id, flags, seq, vx, vy, vz, yaw, yaw_rate, ax, ay, az):
    payload = struct.pack(FMT, entity_id, flags, seq, vx, vy, vz,
                          yaw, yaw_rate, ax, ay, az)
    cs = sum(payload) & 0xFF
    frame = struct.pack('<BBHBH', HEAD, CTRL, LOWER, MSG_VEL, len(payload))
    return frame + payload + bytes([cs])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--flags', type=int, default=3)
    ap.add_argument('--vz', type=float, default=2.0)
    ap.add_argument('--secs', type=float, default=8.0)
    ap.add_argument('--entity-id', type=int, default=281474976710657)  # uav1
    a = ap.parse_args()

    rclpy.init()
    node = Node('velprobe')
    zs = []
    node.create_subscription(
        ModelInfo, '/sigma/camp_0/squad_0/MiddleUAV_1/model_info',
        lambda m: zs.append(m.z), 10)

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    seq = 0
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.secs:
        pkt = build(a.entity_id, a.flags, seq, 0.0, 0.0, a.vz,
                    0.0, 0.0, 0.0, 0.0, 0.0)
        out.sendto(pkt, ('192.168.1.130', 9990))
        seq += 1
        rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(0.1)
    out.close()
    node.destroy_node()
    rclpy.shutdown()
    print(f'flags={a.flags} vz={a.vz} sent {seq} cmds, seq 0..{seq-1}')
    if zs:
        print(f'UE z: first={zs[0]:.3f} max={max(zs):.3f} last={zs[-1]:.3f} samples={len(zs)}')
    else:
        print('no model_info received')


if __name__ == '__main__':
    main()
