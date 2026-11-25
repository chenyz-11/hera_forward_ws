"""Chapter-4 demonstration: reconfiguration after one node exits and rejoins.

The six vehicles' *initial relative positions* are the initial formation (no
wide_2x3 forming stage).  The whole sequence is executed while advancing:

  1. initial_advance : all six hold the initial formation, advance straight.
  2. exit_reconfig   : one normal follower (NW) is confirmed withdrawn and
                       drifts off-formation while still moving forward; the
                       five remaining vehicles run a Local reconfigure --
                       template regeneration (chain/wedge template), Hungarian
                       node-slot re-assignment, DVS quintic transition -- while
                       advancing.
  3. compact_advance : five vehicles hold the compact formation, advance.
  4. rejoin          : the withdrawn vehicle finishes recovery confirmation +
                       state sync + role check and re-enters as a NEW candidate;
                       Hungarian matches all six into the initial template, DVS
                       smoothly restores the original formation, all advancing.
  5. final_advance   : six vehicles hold the restored formation, advance.

DVS uses S(tau)=10tau^3-15tau^4+6tau^5 (zero endpoint velocity/acceleration),
with transition time adapted to remaining speed/acceleration margins.
"""

import math

from scipy.optimize import linear_sum_assignment

from hera_forward.formation_switch_logic import normalize_angle


# ---------------------------------------------------------------------------
# DVS quintic (Chapter 4, Sec. 4.5)
# ---------------------------------------------------------------------------
def dvs_quintic(tau):
    t = min(1.0, max(0.0, tau))
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


def dvs_duration(max_delta_m, v_margin, a_margin, t_min, safety, t_max):
    """T >= 1.875*d/v_margin ; T >= sqrt(5.77*d/a_margin)."""
    if max_delta_m <= 1e-9:
        return t_min
    t_speed = 1.875 * max_delta_m / v_margin
    t_accel = math.sqrt(5.77 * max_delta_m / a_margin)
    return min(t_max, max(t_speed, t_accel, t_min) * safety)


# ---------------------------------------------------------------------------
# Hungarian assignment with composite cost (Chapter 4, Sec. 4.4)
# ---------------------------------------------------------------------------
def hungarian_assign(robots, slots, robot_pose, slot_world, old_slot_of):
    robots, slots = list(robots), list(slots)
    dists = [[math.dist(robot_pose[r][:2], slot_world[s][:2]) for s in slots]
             for r in robots]
    dmax = max((max(row) for row in dists), default=1.0) or 1.0
    cost = []
    for i, r in enumerate(robots):
        row = []
        for j, s in enumerate(slots):
            path = dists[i][j] / dmax
            comm = path * 0.5
            struct = 0.0 if old_slot_of.get(r) == s else 1.0
            row.append(1.0 * path + 0.2 * comm + 0.15 * struct)
        cost.append(row)
    rows, cols = linear_sum_assignment(cost)
    return {robots[i]: slots[j] for i, j in zip(rows, cols)}, cost


# ---------------------------------------------------------------------------
# Path helpers
# ---------------------------------------------------------------------------
def _line(start, goal, step_m, yaw_end=None):
    d = math.hypot(goal[0] - start[0], goal[1] - start[1])
    n = max(2, int(math.ceil(d / step_m)) + 1)
    ye = start[2] if yaw_end is None else yaw_end
    return [(start[0] + (goal[0] - start[0]) * k / (n - 1),
             start[1] + (goal[1] - start[1]) * k / (n - 1),
             ye if k == n - 1 else start[2]) for k in range(n)]


def _dvs_arc(start, goal, n_points, yaw_start, yaw_end):
    pts = []
    for k in range(n_points):
        tau = k / (n_points - 1)
        s = dvs_quintic(tau)
        pts.append((start[0] + (goal[0] - start[0]) * s,
                    start[1] + (goal[1] - start[1]) * s,
                    normalize_angle(yaw_start + (yaw_end - yaw_start) * s)))
    return pts


def _initial_offsets(init_poses, ref=1):
    """Initial-formation template = recorded poses relative to vehicle ``ref``."""
    rx, ry = init_poses[ref][0], init_poses[ref][1]
    return {r: (init_poses[r][0] - rx, init_poses[r][1] - ry)
            for r in init_poses}


def _compact_template(offsets, keep_slots):
    """Regenerated compact template for the remaining vehicles: shift every
    kept relative pose toward the group centre by 35% (Scale action in Ch.4)."""
    cx = sum(o[0] for o in offsets.values()) / len(offsets)
    cy = sum(o[1] for o in offsets.values()) / len(offsets)
    return {s: (cx + (offsets[s][0] - cx) * 0.65,
                cy + (offsets[s][1] - cy) * 0.65) for s in keep_slots}


# ---------------------------------------------------------------------------
# Main plan
# ---------------------------------------------------------------------------
def build_reconfiguration_plan(init_poses, params, dubins_fn=None):
    """``dubins_fn(start, goal)`` -> [(x,y,yaw),...] honoring turn radius.

    The withdrawing vehicle's exit and rejoin paths use Dubins so its lateral
    departure/return respects the 4.5 m turn-radius limit.
    """
    need = ('initial_advance_m', 'exit_advance_m', 'exit_offset_m',
            'reconfig_advance_m', 'compact_advance_m', 'rejoin_advance_m',
            'final_advance_m', 'reference_speed_mps', 'step_m',
            'dvs_v_margin', 'dvs_a_margin', 'dvs_t_min', 'dvs_safety', 'dvs_t_max')
    if any(p not in params for p in need):
        raise ValueError('params missing: ' + ', '.join(p for p in need if p not in params))

    a1 = params['initial_advance_m']
    ex_adv = params['exit_advance_m']
    ex_off = params['exit_offset_m']
    rec_adv = params['reconfig_advance_m']
    comp_adv = params['compact_advance_m']
    rej_adv = params['rejoin_advance_m']
    fin_adv = params['final_advance_m']
    v = params['reference_speed_mps']
    step = params['step_m']

    ids = sorted(init_poses)
    offsets = _initial_offsets(init_poses)          # slot id == robot id
    slots_all = sorted(offsets)

    # NW follower = west column, north-most (confirmed normal follower -> Local).
    xs = sorted(init_poses[r][0] for r in ids)
    west_x = xs[1]                                   # second-smallest x
    west = [r for r in ids if abs(init_poses[r][0] - west_x) < 1e-9]
    west = west if west else [min(ids, key=lambda r: init_poses[r][0])]
    exit_robot = max(west, key=lambda r: init_poses[r][1])

    def world(template, ax, ay):
        return {s: (ax + ox, ay + oy, 0.0) for s, (ox, oy) in template.items()}

    ax0, ay0 = init_poses[1][0], init_poses[1][1]
    cur_pose = dict(init_poses)
    cur_slot = {r: r for r in ids}
    remaining = [r for r in ids if r != exit_robot]
    stages = []
    info = {'exit_robot': exit_robot, 'initial_offsets': offsets}

    # ---- 1. initial_advance ------------------------------------------------
    ax1 = ax0 + a1
    wg1 = world(offsets, ax1, ay0)
    p1 = {r: _line(cur_pose[r], wg1[r], step) for r in ids}
    stages.append({'name': 'initial_advance', 'paths': p1, 'time_mode': 'arc',
                   'duration': a1 / v})
    cur_pose = dict(wg1)

    # ---- 2. exit_reconfig (Local) ------------------------------------------
    hold = cur_pose[exit_robot]
    # Depart toward the OUTSIDE of the formation (west + north for the NW
    # vehicle) so the Dubins arc never cuts through neighbouring vehicles.
    exit_goal = (hold[0] - ex_adv, hold[1] + ex_off, 0.0)
    # Dubins exit: turn-radius-limited lateral departure (more visible).
    if dubins_fn is not None:
        exit_path = dubins_fn(hold, exit_goal)
    else:
        exit_path = _line(hold, exit_goal, step)
    info['exit_path_len'] = sum(math.dist(a[:2], b[:2])
                                for a, b in zip(exit_path, exit_path[1:]))

    compact = _compact_template(offsets, keep_slots=remaining)
    ax2 = ax1 + rec_adv
    cw = world(compact, ax2, ay0)
    assign2, _ = hungarian_assign(remaining, sorted(compact), cur_pose, cw, cur_slot)
    info['assign_exit'] = assign2

    delta2 = [math.dist(cur_pose[r][:2], cw[assign2[r]][:2]) for r in remaining]
    t2 = dvs_duration(max(delta2), params['dvs_v_margin'], params['dvs_a_margin'],
                      params['dvs_t_min'], params['dvs_safety'], params['dvs_t_max'])
    info['dvs_exit_s'] = t2
    info['dvs_exit_max_delta_m'] = max(delta2)

    p2 = {exit_robot: exit_path}
    for r in remaining:
        p2[r] = _dvs_arc(cur_pose[r], cw[assign2[r]],
                         max(30, int(t2 * 10)), cur_pose[r][2], 0.0)
    stages.append({'name': 'exit_reconfig', 'paths': p2, 'time_mode': 'uniform',
                   'duration': max(t2, ex_adv / v)})
    for r in remaining:
        cur_pose[r] = cw[assign2[r]]
        cur_slot[r] = assign2[r]
    cur_pose[exit_robot] = exit_goal

    # ---- 3. compact_advance ------------------------------------------------
    ax3 = ax2 + comp_adv
    cw2 = world(compact, ax3, ay0)
    p3 = {r: _line(cur_pose[r], cw2[cur_slot[r]], step) for r in remaining}
    p3[exit_robot] = _line(cur_pose[exit_robot],
                           (cur_pose[exit_robot][0] + comp_adv,
                            cur_pose[exit_robot][1], 0.0), step)
    stages.append({'name': 'compact_advance', 'paths': p3, 'time_mode': 'arc',
                   'duration': comp_adv / v})
    for r in remaining:
        cur_pose[r] = cw2[cur_slot[r]]
    cur_pose[exit_robot] = (cur_pose[exit_robot][0] + comp_adv,
                            cur_pose[exit_robot][1], 0.0)

    # ---- 4. rejoin (recovering node re-enters the CURRENT formation) -------
    ax4 = ax3 + rej_adv
    wg4 = world(offsets, ax4, ay0)                   # restore initial template
    old_slot = dict(cur_slot)
    old_slot[exit_robot] = exit_robot
    assign4, _ = hungarian_assign(ids, slots_all, cur_pose, wg4, old_slot)
    info['assign_rejoin'] = assign4

    delta4 = [math.dist(cur_pose[r][:2], wg4[assign4[r]][:2]) for r in ids]
    t4 = dvs_duration(max(delta4), params['dvs_v_margin'], params['dvs_a_margin'],
                      params['dvs_t_min'], params['dvs_safety'], params['dvs_t_max'])
    info['dvs_rejoin_s'] = t4
    info['dvs_rejoin_max_delta_m'] = max(delta4)

    p4 = {}
    for r in ids:
        goal = wg4[assign4[r]]
        if r == exit_robot and dubins_fn is not None:
            # Recovering vehicle returns on a turn-radius-limited Dubins arc.
            p4[r] = dubins_fn(cur_pose[r], goal)
        else:
            p4[r] = _dvs_arc(cur_pose[r], goal, max(30, int(t4 * 10)),
                             cur_pose[r][2], 0.0)
    stages.append({'name': 'rejoin', 'paths': p4, 'time_mode': 'uniform',
                   'duration': t4})
    for r in ids:
        cur_pose[r] = wg4[assign4[r]]
        cur_slot[r] = assign4[r]

    # ---- 5. final_advance --------------------------------------------------
    ax5 = ax4 + fin_adv
    wg5 = world(offsets, ax5, ay0)
    p5 = {r: _line(cur_pose[r], wg5[cur_slot[r]], step) for r in ids}
    stages.append({'name': 'final_advance', 'paths': p5, 'time_mode': 'arc',
                   'duration': fin_adv / v})

    info['final_goals'] = {r: wg5[cur_slot[r]] for r in ids}
    return stages, info


def stitch_continuous_trajectory(stages, robot_ids):
    """Concatenate all stages into ONE timeline per vehicle.

    Returns ``{robot: [(x, y, yaw, t), ...]}`` with a single absolute time base
    from 0 to the total duration.  Duplicate junction points (same time) are
    merged so timestamps stay strictly increasing.  Stages marked ``uniform``
    keep their evenly-spaced DVS timing; ``arc`` stages use arc-length timing.
    """
    trajectories = {r: [] for r in robot_ids}
    cursor = 0.0
    for stage in stages:
        duration = stage['duration']
        mode = stage['time_mode']
        for r in robot_ids:
            points = stage['paths'][r]
            n = len(points)
            if n == 0:
                continue
            total = sum(math.dist(a[:2], b[:2]) for a, b in zip(points, points[1:]))
            cumulative = 0.0
            for i, (x, y, yaw) in enumerate(points):
                if i > 0:
                    cumulative += math.dist(points[i - 1][:2], (x, y))
                if mode == 'uniform' and n > 1:
                    local_t = duration * i / (n - 1)
                else:
                    local_t = duration * cumulative / total if total > 0 else (
                        duration * i / (n - 1) if n > 1 else 0.0)
                t = cursor + local_t
                traj = trajectories[r]
                if traj and t <= traj[-1][3] + 1e-9:
                    # Junction: keep the later point's position, same timestamp.
                    traj[-1] = (x, y, yaw, traj[-1][3])
                else:
                    traj.append((x, y, yaw, t))
        cursor += duration
    return trajectories, cursor


# ---------------------------------------------------------------------------
# Flow-based continuous trajectory (final smooth scheme)
# ---------------------------------------------------------------------------
def _fit_circle(points):
    """Least-squares circle centre/radius from {id:(x,y)}."""
    import numpy as np
    ids = sorted(points)
    pts = np.array([[points[r][0], points[r][1]] for r in ids])
    A = np.c_[2 * pts[:, 0], 2 * pts[:, 1], np.ones(len(ids))]
    b = (pts ** 2).sum(axis=1)
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    return cx, cy, math.sqrt(max(c + cx * cx + cy * cy, 0.0))


def _angle_diff(a, b):
    """Signed shortest angular difference a-b in (-pi, pi]."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def build_flow_trajectory(init_poses, params):
    """Ring-preserving reconfiguration with a shared advancing anchor.

    The six initial poses lie on a circle (a near-uniform 6-vehicle ring).  The
    NW vehicle exits; the five remaining vehicles redistribute to a *uniform
    5-vehicle ring* (72 deg spacing) -- a genuine reconfiguration that keeps the
    ring shape.  The exiting vehicle drifts radially outward off the ring.  When
    it recovers, all six return to the original 6-vehicle ring.  The ring centre
    advances east throughout, so every vehicle keeps moving forward; ring motion
    follows circular arcs (radius ~15 m > 4.5 m turn limit).

    Returns ``({robot: [(x, y, yaw, t), ...]}, info)``.
    """
    need = ('initial_advance_m', 'exit_drift_m', 'exit_offset_m',
            'compact_hold_m', 'rejoin_drift_m', 'final_advance_m',
            'reference_speed_mps', 'step_m')
    if any(p not in params for p in need):
        raise ValueError('params missing: ' + ', '.join(p for p in need if p not in params))

    d1 = params['initial_advance_m']
    d2 = params['exit_drift_m']
    d3 = params['compact_hold_m']
    d4 = params['rejoin_drift_m']
    d5 = params['final_advance_m']
    exit_off = params['exit_offset_m']
    v = params['reference_speed_mps']
    step = params['step_m']
    total = d1 + d2 + d3 + d4 + d5

    ids = sorted(init_poses)
    cx, cy, R = _fit_circle(init_poses)
    ang0 = {r: math.atan2(init_poses[r][1] - cy, init_poses[r][0] - cx) % (2 * math.pi)
            for r in ids}
    order = sorted(ids, key=lambda r: ang0[r])
    xs = sorted(init_poses[r][0] for r in ids)
    west = [r for r in ids if abs(init_poses[r][0] - xs[1]) < 1e-9]
    west = west or [min(ids, key=lambda r: init_poses[r][0])]
    exit_robot = max(west, key=lambda r: init_poses[r][1])
    keep = [r for r in order if r != exit_robot]

    # Uniform 5-ring angles: assign in angular order, choose the phase that
    # minimizes total angular travel (keeps the ring organization contiguous).
    n = len(keep)
    best = None
    for k in range(n):
        phase = ang0[keep[k]] - 2.0 * math.pi * k / n
        new_angles = {keep[i]: (phase + 2.0 * math.pi * i / n) % (2.0 * math.pi)
                      for i in range(n)}
        cost = sum(abs(_angle_diff(new_angles[r], ang0[r])) for r in keep)
        if best is None or cost < best[0]:
            best = (cost, new_angles)
    new_ring = best[1]

    def progress(s):
        if s <= d1:
            return 0.0
        if s <= d1 + d2:
            return dvs_quintic((s - d1) / d2)
        if s <= d1 + d2 + d3:
            return 1.0
        if s <= d1 + d2 + d3 + d4:
            return 1.0 - dvs_quintic((s - d1 - d2 - d3) / d4)
        return 0.0

    def pose_at(r, s):
        f = progress(s)
        if r == exit_robot:
            # Drift radially outward off the ring; keep the same angle so the
            # radial excursion stays x-monotonic with the advancing centre.
            rad = R + f * exit_off
            a = ang0[r]
            return (cx + s + rad * math.cos(a), cy + rad * math.sin(a), 0.0)
        a = ang0[r] + f * _angle_diff(new_ring[r], ang0[r])
        return (cx + s + R * math.cos(a), cy + R * math.sin(a), 0.0)

    s_values = [step * k for k in range(int(total / step) + 1)]
    if s_values[-1] < total - 1e-9:
        s_values.append(total)
    trajectories = {r: [(pose_at(r, s)[0], pose_at(r, s)[1],
                         pose_at(r, s)[2], s / v) for s in s_values]
                    for r in ids}
    info = {'exit_robot': exit_robot, 'total': total,
            'ring_center': (cx, cy), 'ring_radius': R,
            'exit_offset_m': exit_off, 'new_ring_angles_deg':
                {r: round(math.degrees(new_ring[r]), 1) for r in keep}}
    return trajectories, info


# ---------------------------------------------------------------------------
# Chapter-5 style: damaged node stops, survivors refill the ring uniformly
# ---------------------------------------------------------------------------
def build_damaged_node_trajectory(init_poses, params):
    """One node is damaged (stops); survivors reconfigure into a uniform ring.

    Different from the exit/rejoin scheme: the damaged node NEVER rejoins.  It
    halts at its last pose, while the remaining vehicles keep advancing and
    redistribute on the ring to fill the gap (uniform spacing), following the
    Chapter-5 'locate gap -> select redundant node -> virtual-force guided
    refill -> verify' idea, realized here as a smooth quintic reconfiguration
    on a shared advancing anchor.

    Returns ``({robot: [(x, y, yaw, t), ...]}, info)``.
    """
    need = ('initial_advance_m', 'reconfig_advance_m', 'final_advance_m',
            'reference_speed_mps', 'step_m')
    if any(p not in params for p in need):
        raise ValueError('params missing: ' + ', '.join(p for p in need if p not in params))

    d1 = params['initial_advance_m']
    d2 = params['reconfig_advance_m']
    d3 = params['final_advance_m']
    v = params['reference_speed_mps']
    step = params['step_m']
    total = d1 + d2 + d3

    ids = sorted(init_poses)
    cx, cy, R = _fit_circle(init_poses)
    radius = {r: math.hypot(init_poses[r][0] - cx, init_poses[r][1] - cy) for r in ids}
    ang0 = {r: math.atan2(init_poses[r][1] - cy, init_poses[r][0] - cx) % (2 * math.pi)
            for r in ids}
    # Damaged node = NW follower (west column, north-most)
    xs = sorted(init_poses[r][0] for r in ids)
    west = [r for r in ids if abs(init_poses[r][0] - xs[1]) < 1e-9]
    west = west or [min(ids, key=lambda r: init_poses[r][0])]
    damaged = max(west, key=lambda r: init_poses[r][1])
    order_angular = sorted([r for r in ids if r != damaged], key=lambda r: ang0[r])

    # Uniform ring for survivors: angular order preserved, phase minimizes travel.
    n = len(order_angular)
    best = None
    for k in range(n):
        phase = ang0[order_angular[k]] - 2.0 * math.pi * k / n
        new_angles = {order_angular[i]: (phase + 2.0 * math.pi * i / n) % (2.0 * math.pi)
                      for i in range(n)}
        cost = sum(abs(_angle_diff(new_angles[r], ang0[r])) for r in order_angular)
        if best is None or cost < best[0]:
            best = (cost, new_angles)
    new_ring = best[1]

    def progress(s):
        if s <= d1:
            return 0.0
        if s <= d1 + d2:
            return dvs_quintic((s - d1) / d2)
        return 1.0

    def pose_at(r, s):
        f = progress(s) if r in new_ring else 0.0   # damaged keeps original angle
        a = ang0[r] + f * _angle_diff(new_ring[r], ang0[r]) if r in new_ring else ang0[r]
        return (cx + s + radius[r] * math.cos(a),
                cy + radius[r] * math.sin(a), 0.0, s / v)

    s_values = [step * k for k in range(int((d1 + d2 + d3) / step) + 1)]
    trajectories = {r: [] for r in ids}
    for s in s_values:
        for r in ids:
            if r == damaged and s > d1 + 1e-9:
                # Damaged vehicle advances with everyone for d1, then stops:
                # its path ENDS at s=d1 (no further points) -> platform holds.
                continue
            trajectories[r].append(pose_at(r, s))
    info = {'damaged_robot': damaged, 'total_m': total,
            'damaged_stop_m': d1,
            'ring_center': (cx, cy), 'ring_radius': R,
            'new_ring_angles_deg': {r: round(math.degrees(new_ring[r]), 1)
                                    for r in order_angular}}
    return trajectories, info
