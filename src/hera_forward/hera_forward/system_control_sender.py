#!/usr/bin/env python3
"""Send a raw DXFD SYSTEM_CONTROL_CMD to UE over UDP (bypasses SDK wrapper).

The SDK wrapper (sigmaSDK) does not expose SendSystemControlCmd, so this tool
encodes the DXFD protocol frame directly.  Used to put UAVs into offboard+armed
so they will execute motion_command / reference_trajectory.

Frame: head=0xDF, upperID=255(CONTROLLER_ID), lowerID=1, msgID=26,
       length=4, payload=[command, offboard, armed, algo_running],
       checksum=sum(payload) & 0xFF.
"""

import argparse
import socket
import struct
import time

HEAD = 0xDF
CONTROLLER_ID = 255
LOWER_ID = 1
MSG_ID_SYSTEM_CONTROL_CMD = 26


def encode(command, offboard, armed, algo_running):
    payload = bytes([command & 0xFF, offboard & 0xFF, armed & 0xFF, algo_running & 0xFF])
    checksum = sum(payload) & 0xFF
    # head, upperID, lowerID(2 LE), msgID, length(2 LE) + payload + checksum
    frame = struct.pack('<BBHBH', HEAD, CONTROLLER_ID, LOWER_ID,
                        MSG_ID_SYSTEM_CONTROL_CMD, len(payload))
    return frame + payload + bytes([checksum])


def send(ip, port, command, offboard, armed, algo_running, repeat):
    pkt = encode(command, offboard, armed, algo_running)
    print(f'SYSTEM_CONTROL_CMD: cmd={command} offboard={offboard} armed={armed} '
          f'algo={algo_running} frame={pkt.hex()}')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for i in range(repeat):
        sock.sendto(pkt, (ip, port))
        print(f'  sent {i+1}/{repeat} to {ip}:{port}')
        time.sleep(0.3)
    sock.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', default='192.168.1.130')
    ap.add_argument('--port', type=int, default=9990)
    ap.add_argument('--command', type=int, default=1,
                    help='0 none, 1 start, 2 stop, 3 restart')
    ap.add_argument('--offboard', type=int, default=1)
    ap.add_argument('--armed', type=int, default=1)
    ap.add_argument('--algo', type=int, default=1)
    ap.add_argument('--repeat', type=int, default=3)
    a = ap.parse_args()
    send(a.ip, a.port, a.command, a.offboard, a.armed, a.algo, a.repeat)


if __name__ == '__main__':
    main()
