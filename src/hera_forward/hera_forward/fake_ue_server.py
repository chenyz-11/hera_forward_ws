#!/usr/bin/env python3
"""Minimal fake UE UDP peer for verifying bridge -> UE command emission.

Listens on LOCAL_PORT (default 9990). Replies to REQUEST_SIM_DATA (msgID=10)
with six MODEL_INFO (msgID=9) packets so the publisher's odom becomes fresh and
it starts streaming. Records all msgIDs received from the bridge.
"""
import argparse
import socket
import struct
import time

HEAD = 0xDF
MSG_REQUEST_SIM_DATA = 10
MSG_MODEL_INFO = 9
INIT = [(-3.0, 34.0), (-17.0, 30.0), (-27.0, 42.0),
        (-22.0, 57.0), (-7.0, 60.0), (3.0, 49.0)]


def model_info_packet(entity_id, x, y):
    # dxfd_model_info_t: entityID(int64), seq(u8), lat/lon(int32), alt(f),
    # roll/pitch/yaw(f), vx/vy/vz(f), sizeX/Y/Z(f), health(f) = 11 floats
    payload = struct.pack('<qBii11f', entity_id, 0,
                          int(x * 1e6), int(y * 1e6), 0.0,
                          0.0, 0.0, 0.0,          # roll/pitch/yaw
                          0.0, 0.0, 0.0,          # vx/vy/vz
                          1.0, 1.0, 1.0, 0.0)      # sizeX/Y/Z, health
    cs = sum(payload) & 0xFF
    frame = struct.pack('<BBHBH', HEAD, 255, 1, MSG_MODEL_INFO, len(payload))
    return frame + payload + bytes([cs])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=9990)
    ap.add_argument('--secs', type=float, default=30.0)
    ap.add_argument('--peer', default='127.0.0.1')
    ap.add_argument('--model-type', type=int, default=0,
                    help='model_type in fake model_info: 0=BigUAV, 1=MiddleUAV')
    a = ap.parse_args()
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('0.0.0.0', a.port))
    srv.settimeout(0.5)
    seen = {}
    t0 = time.monotonic()
    while time.monotonic() - t0 < a.secs:
        try:
            data, addr = srv.recvfrom(65535)
            if len(data) >= 5 and data[0] == HEAD:
                mid = data[4]
                seen[mid] = seen.get(mid, 0) + 1
                if mid == 37 and seen[37] == 1:  # parse first velocity packet
                    payload = data[7:7 + struct.unpack('<H', data[5:7])[0]]
                    v = struct.unpack('<qBH8f', payload[:43])
                    eid, flags, seq = v[0], v[1], v[2]
                    vx, vy, vz, yaw, yaw_rate = v[3], v[4], v[5], v[6], v[7]
                    print(f'VEL PKT: entity_id=0x{eid:016X} flags={flags} seq={seq} '
                          f'vx={vx:.2f} vy={vy:.2f} vz={vz:.2f} yaw_deg={yaw:.2f} '
                          f'yaw_rate={yaw_rate:.2f}', flush=True)
                if mid == 31 and seen[31] == 1:  # parse first trajectory packet
                    payload = data[7:7 + struct.unpack('<H', data[5:7])[0]]
                    hdr = struct.unpack('<qBBHBH', payload[:15])
                    eid, cmd, flags, seq, has_more, count = hdr
                    print(f'TRAJ PKT: entity_id=0x{eid:016X} cmd={cmd} flags={flags} '
                          f'seq={seq} has_more={has_more} count={count}', flush=True)
                    if count >= 1:
                        pt = struct.unpack('<7f', payload[15:15 + 28])
                        print(f'  point0: x_cm={pt[0]:.1f} y_cm={pt[1]:.1f} z_cm={pt[2]:.1f} '
                              f'yaw_deg={pt[3]:.2f} speed={pt[4]:.2f} time={pt[5]:.2f}',
                              flush=True)
                if mid == MSG_REQUEST_SIM_DATA:
                    for i, (x, y) in enumerate(INIT):
                        eid = (a.model_type << 48) | (i + 1)
                        srv.sendto(model_info_packet(eid, x, y), addr)
        except socket.timeout:
            pass
    srv.close()
    print('received msgIDs from bridge:', dict(sorted(seen.items())))


if __name__ == '__main__':
    main()
