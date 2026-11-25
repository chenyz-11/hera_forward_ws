"""Chapter-5 demonstration: environment-adaptive formation around an obstacle.

Six vehicles form their ring from the recorded initial poses.  A rectangular
obstacle blocks the corridor ~75 m east of the fleet (y-span ~20 m).  Following
Chapter 5, the plan:

  1. pre-view detects the current ring cannot pass the obstacle;
  2. BEFORE reaching it, the fleet shifts its reference centre north and
     contracts its scale (class-preserving scale adjustment) with a quintic
     S-curve -- the switch completes ahead of the obstacle (Sec. 5.4);
  3. the fleet passes north of the obstacle in the compact ring;
  4. after clearing it, the centre returns and the scale restores to the
     original ring (Restore as a positive-benefit replan, Sec. 5.5).

Everything is stitched into ONE per-vehicle timed trajectory with a shared
eastward anchor progress, so vehicles keep moving forward the whole time.
"""

import math

from hera_forward.reconfiguration_logic import dvs_quintic, _fit_circle


def build_obstacle_avoidance(init_poses, params):
    """Return ``({robot: [(x, y, yaw, t), ...]}, info)``."""
    need = ('obstacle_east_m', 'obstacle_half_width_y_m', 'obstacle_half_depth_x_m',
            'clearance_margin_m', 'bypass_extra_m', 'scale_min',
            'approach_m', 'shift_m', 'pass_m', 'return_m', 'final_m',
            'reference_speed_mps', 'step_m')
    if any(p not in params for p in need):
        raise ValueError('params missing: ' + ', '.join(p for p in need if p not in params))

    east = params['obstacle_east_m']
    half_y = params['obstacle_half_width_y_m']
    half_x = params['obstacle_half_depth_x_m']
    margin = params['clearance_margin_m']
    extra = params['bypass_extra_m']
    smin = params['scale_min']
    a_pre = params['approach_m']
    a_shift = params['shift_m']
    a_pass = params['pass_m']
    a_ret = params['return_m']
    a_final = params['final_m']
    v = params['reference_speed_mps']
    step = params['step_m']
    total = a_pre + a_shift + a_pass + a_ret + a_final

    ids = sorted(init_poses)
    cx, cy, R = _fit_circle(init_poses)
    ang0 = {r: math.atan2(init_poses[r][1] - cy, init_poses[r][0] - cx) for r in ids}
    radius = {r: math.hypot(init_poses[r][0] - cx, init_poses[r][1] - cy) for r in ids}
    obs_x = cx + east
    obs_north = cy + half_y

    # Centre offset needed to clear the obstacle's north edge with margin, plus
    # an extra buffer so the outermost vehicle in the (contracted) ring clears.
    y_off = (obs_north - cy) + margin + max(radius.values()) * smin + extra

    def phase(s):
        if s <= a_pre:
            return 0.0, 0.0
        if s <= a_pre + a_shift:
            return dvs_quintic((s - a_pre) / a_shift), 0.0
        if s <= a_pre + a_shift + a_pass:
            return 1.0, 0.0
        if s <= a_pre + a_shift + a_pass + a_ret:
            return 1.0, dvs_quintic((s - a_pre - a_shift - a_pass) / a_ret)
        return 1.0, 1.0

    def pose_at(r, s):
        pu, pd = phase(s)
        scale = 1.0 - (1.0 - smin) * pu + (1.0 - smin) * pd
        centre_y = cy + y_off * (pu - pd)
        a = ang0[r]
        rr = radius[r] * scale
        return (cx + s + rr * math.cos(a),
                centre_y + rr * math.sin(a), 0.0)

    s_values = [step * k for k in range(int(total / step) + 1)]
    if s_values[-1] < total - 1e-9:
        s_values.append(total)
    trajectories = {r: [(pose_at(r, s)[0], pose_at(r, s)[1],
                         pose_at(r, s)[2], s / v) for s in s_values]
                    for r in ids}
    info = {'ring_center': (cx, cy), 'ring_radius': R,
            'obstacle_center_x': obs_x, 'obstacle_north_y': obs_north,
            'bypass_y_offset_m': y_off, 'scale_min': smin, 'total_m': total,
            'switch_start_m': a_pre, 'obstacle_at_m': east}
    return trajectories, info
