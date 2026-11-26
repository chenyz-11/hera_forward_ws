#!/usr/bin/env python3
"""Send a raw DXFD SIMULATION_CONTROL (msgID=4) to UE over UDP.

The SDK wrapper (sigmaSDK) does NOT expose SendSimulationControl, so this tool
encodes the DXFD frame directly.  Hypothesis: MiddleUAV needs its PX4 flight
component started / the algorithm registered before it will consume velocity or
trajectory commands.  SIMULATION_CONTROL carries controlType=RUN_COMPONENT_
SOFTWARE, componentID=COMPONENT_PX4 and algorithmIP/algorithmPort, which look
exactly like that mechanism.

Frame: head=0xDF, upperID=255(CONTROLLER_ID), lowerID=1, msgID=4,
       length=sizeof(dxfd_simulation_control_t)=107, payload=<packed struct>,
       checksum=sum(payload) & 0xFF.

C struct (dxfd_msg_simulation_control.h, #pragma pack(1)) - 107 bytes:
    uint16 dynamicModelID; uint8 seq,controlType,componentID,simMode,simSpeed,
    renderDataOutRate; int32 longitude,latitude; float altitude,roll,pitch,yaw;
    uint8 nameLength; char modelName[64]; uint64 algorithmIP; uint16 algorithmPort.
"""

import argparse
import socket
import struct
import time

HEAD = 0xDF
CONTROLLER_ID = 255
LOWER_ID = 1
MSG_ID_SIMULATION_CONTROL = 4

# '<' = little-endian, no padding (matches #pragma pack(1))
STRUCT_FMT = '<H6BiiffffB64sQH'
PAYLOAD_LEN = struct.calcsize(STRUCT_FMT)  # expect 107

# DXFD_SIM_CONTROL_CMD_TYPE
IDLE = 0
RUN_COMPONENT_SOFTWARE = 1
STOP_COMPONENT_SOFTWARE = 2
SET_PARAMS = 3
REBOOT_ALL_SOFTWARE = 4
STOP_ALL_SOFTWARE = 5
# COMPONENT_ID (SIM_COMPONENT_ID)
COMPONENT_ALL = 0
COMPONENT_PX4 = 1
COMPONENT_JSB = 2
COMPONENT_RC = 3
# DXFD_SIM_MODE
SITL_SENSOR = 0
SITL_STATE = 1
HITL_STATE = 2

# UAV config home_origin (sigma_uav_only_initial_poses.yaml) in micro-degrees
DEFAULT_LON = 121876533
DEFAULT_LAT = 22565657


def ip_to_uint64(value):
    """Accept a dotted IPv4 or an integer and return a uint64."""
    if value is None:
        return 0
    value = str(value)
    if '.' in value:
        packed = socket.inet_aton(value)             # 4 bytes, network order
        return struct.unpack('!I', packed)[0]
    return int(value, 0) & 0xFFFFFFFFFFFFFFFF


def encode(dynamic_model_id=0, seq=0, control_type=RUN_COMPONENT_SOFTWARE,
           component=COMPONENT_PX4, sim_mode=SITL_SENSOR, sim_speed=1,
           render_rate=20, longitude=DEFAULT_LON, latitude=DEFAULT_LAT,
           altitude=0.0, roll=0.0, pitch=0.0, yaw=0.0, model_name='',
           algorithm_ip=0, algorithm_port=26530):
    name = model_name.encode('utf-8')[:64]
    name_len = len(name)
    name_field = name + b'\x00' * (64 - name_len)
    payload = struct.pack(
        STRUCT_FMT,
        int(dynamic_model_id) & 0xFFFF,
        int(seq) & 0xFF,
        int(control_type) & 0xFF,
        int(component) & 0xFF,
        int(sim_mode) & 0xFF,
        int(sim_speed) & 0xFF,
        int(render_rate) & 0xFF,
        int(longitude),
        int(latitude),
        float(altitude), float(roll), float(pitch), float(yaw),
        name_len & 0xFF,
        name_field,
        int(algorithm_ip) & 0xFFFFFFFFFFFFFFFF,
        int(algorithm_port) & 0xFFFF,
    )
    assert len(payload) == PAYLOAD_LEN
    checksum = sum(payload) & 0xFF
    frame = struct.pack('<BBHBH', HEAD, CONTROLLER_ID, LOWER_ID,
                        MSG_ID_SIMULATION_CONTROL, len(payload))
    return frame + payload + bytes([checksum])


def decode_fields(frame):
    """Decode just the payload fields from a frame produced by encode()."""
    payload = frame[7:7 + PAYLOAD_LEN]
    return struct.unpack(STRUCT_FMT, payload)


def send(ip, port, repeat, interval, **kwargs):
    pkt = encode(**kwargs)
    f = decode_fields(pkt)
    print(f'SIMULATION_CONTROL: frame_len={len(pkt)} payload_len={PAYLOAD_LEN} '
          f'checksum=0x{pkt[-1]:02X}')
    print(f'  dynamicModelID={f[0]} seq={f[1]} controlType={f[2]} '
          f'component={f[3]} simMode={f[4]} simSpeed={f[5]} renderRate={f[6]}')
    print(f'  lon={f[7]} lat={f[8]} alt/roll/pitch/yaw={f[9:13]}')
    print(f'  nameLength={f[13]} modelName={f[14].rstrip(bytes([0]))!r} '
          f'algorithmIP=0x{f[15]:016X} algorithmPort={f[16]}')
    print(f'  hex={pkt.hex()}')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for i in range(repeat):
        sock.sendto(pkt, (ip, port))
        print(f'  sent {i + 1}/{repeat} to {ip}:{port}')
        time.sleep(interval)
    sock.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ip', default='192.168.1.130')
    ap.add_argument('--port', type=int, default=9990)
    ap.add_argument('--dynamic-model-id', type=int, default=0)
    ap.add_argument('--seq', type=int, default=0)
    ap.add_argument('--control-type', type=int, default=RUN_COMPONENT_SOFTWARE,
                    help='0 idle, 1 run component, 2 stop component, 3 set params, 4 reboot all, 5 stop all')
    ap.add_argument('--component', type=int, default=COMPONENT_PX4,
                    help='0 all, 1 PX4, 2 JSB, 3 RC')
    ap.add_argument('--sim-mode', type=int, default=SITL_SENSOR,
                    help='0 SITL_SENSOR, 1 SITL_STATE, 2 HITL_STATE')
    ap.add_argument('--sim-speed', type=int, default=1)
    ap.add_argument('--render-rate', type=int, default=20)
    ap.add_argument('--longitude', type=int, default=DEFAULT_LON)
    ap.add_argument('--latitude', type=int, default=DEFAULT_LAT)
    ap.add_argument('--altitude', type=float, default=0.0)
    ap.add_argument('--model-name', default='')
    ap.add_argument('--algorithm-ip', default='192.168.1.130',
                    help='dotted IPv4 or integer (uint64)')
    ap.add_argument('--algorithm-port', type=int, default=26530)
    ap.add_argument('--repeat', type=int, default=3)
    ap.add_argument('--interval', type=float, default=0.3)
    a = ap.parse_args()
    send(a.ip, a.port, a.repeat, a.interval,
         dynamic_model_id=a.dynamic_model_id, seq=a.seq,
         control_type=a.control_type, component=a.component,
         sim_mode=a.sim_mode, sim_speed=a.sim_speed, render_rate=a.render_rate,
         longitude=a.longitude, latitude=a.latitude, altitude=a.altitude,
         model_name=a.model_name, algorithm_ip=ip_to_uint64(a.algorithm_ip),
         algorithm_port=a.algorithm_port)


if __name__ == '__main__':
    main()
