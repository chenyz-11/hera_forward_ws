"""Bounded velocity tracking and continuous segment clearance, in ENU metres."""
import math


def tracking_velocity(position, reference, feedforward, kp_xy, kp_z, max_xy, max_z,
                      vertical_velocity=0.0, height_damping=0.0):
    if not all(math.isfinite(v) for v in (*position, *reference, *feedforward, vertical_velocity)):
        raise ValueError('nonfinite position or reference')
    vx = feedforward[0] + kp_xy * (reference[0] - position[0])
    vy = feedforward[1] + kp_xy * (reference[1] - position[1])
    length = math.hypot(vx, vy)
    scale = min(1.0, max_xy / length) if length else 1.0
    vz = max(-max_z, min(max_z, feedforward[2] + kp_z * (reference[2] - position[2])
                         - height_damping * vertical_velocity))
    return vx * scale, vy * scale, vz


def segment_clearance(a0, a1, b0, b1):
    """Minimum horizontal separation over synchronous linear interpolation."""
    r = (a0[0] - b0[0], a0[1] - b0[1])
    d = (a1[0] - b1[0] - r[0], a1[1] - b1[1] - r[1])
    denominator = d[0] ** 2 + d[1] ** 2
    t = max(0.0, min(1.0, -(r[0] * d[0] + r[1] * d[1]) / denominator)) if denominator else 0.0
    return math.hypot(r[0] + t * d[0], r[1] + t * d[1])
