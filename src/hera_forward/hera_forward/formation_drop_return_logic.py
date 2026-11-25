"""Geometry for the drop-return-rejoin formation task.

Six staged paths:
  1. ``form``        -- advance 50 m while forming wide_2x3.
  2. ``straight1``   -- hold the formation, advance 50 m.
  3. ``drop``        -- NW vehicle stops (falls out); the other five advance 20 m.
  4. ``slow_return`` -- five vehicles slow to 2 m/s and advance 20 m; the SW
                        vehicle turns around (Dubins) back to the NW vehicle's
                        side, keeping the original slot-5/slot-6 relative pose.
  5. ``rejoin``      -- the two stragglers accelerate and catch up to the column
                        into their original slots (slot5/slot6).
  6. ``straight2``   -- full formation restored, advance another 50 m.

Each stage returns per-vehicle point lists; within a stage time is distributed
by arc length with a shared duration, so a vehicle's speed is automatically its
own path length / stage duration.  This gives the slower/faster split without
any per-vehicle speed parameter.
"""

import math

from hera_forward.formation_switch_logic import (
    formation_targets, normalize_angle)


def _line_points(start, goal, max_segment_m):
    """Straight map-frame samples from ``start`` to ``goal`` (x, y, yaw)."""
    distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
    count = max(2, int(math.ceil(distance / max_segment_m)) + 1)
    return [(start[0] + (goal[0] - start[0]) * k / (count - 1),
             start[1] + (goal[1] - start[1]) * k / (count - 1),
             goal[2] if k == count - 1 else start[2])
            for k in range(count)]


def corner_slots(wide_slots):
    """Return (NW slot, SW slot) -- west column, north-most / south-most."""
    xs = sorted({o[0] for o in wide_slots.values()})
    west_x = xs[0]
    west = [s for s, o in wide_slots.items() if abs(o[0] - west_x) < 1e-9]
    nw = min(west, key=lambda s: -wide_slots[s][1])
    sw = max(west, key=lambda s: -wide_slots[s][1])
    return nw, sw


def build_drop_return_plan(init_poses, slot_ids, wide_slots, params, dubins_fn):
    """Build the six-stage plan.

    ``dubins_fn(start_xyz, goal_xyz)`` returns a list of ``(x, y, yaw)`` points.

    Returns ``(stages, info)`` where each stage is
    ``{'name', 'paths': {robot: [(x, y, yaw), ...]}, 'duration': float}``.
    """
    required = ('form_distance_m', 'straight_distance_m', 'drop_distance_m',
                'slow_distance_m', 'slow_speed_mps', 'rejoin_speed_mps',
                'final_distance_m', 'reference_speed_mps', 'max_segment_m')
    if any(p not in params for p in required):
        raise ValueError(f'params missing one of {required}')

    form_d = params['form_distance_m']
    straight_d = params['straight_distance_m']
    drop_d = params['drop_distance_m']
    slow_d = params['slow_distance_m']
    slow_v = params['slow_speed_mps']
    rejoin_v = params['rejoin_speed_mps']
    final_d = params['final_distance_m']
    ref_v = params['reference_speed_mps']
    step = params['max_segment_m']

    anchor0 = (init_poses[1][0], init_poses[1][1])

    def wide_at(ax):
        return formation_targets(ax, anchor0[1], 0.0, wide_slots)

    nw_slot, sw_slot = corner_slots(wide_slots)
    robot_by_slot = {s: r for r, s in slot_ids.items()}
    nw_robot = robot_by_slot[nw_slot]
    sw_robot = robot_by_slot[sw_slot]
    # Original relative pose of SW slot w.r.t. NW slot (same column, one row
    # pitch south) -- used so the two vehicles rejoin in formation shape.
    rel_sw = (wide_slots[sw_slot][0] - wide_slots[nw_slot][0],
              wide_slots[sw_slot][1] - wide_slots[nw_slot][1])

    stages = []

    # ---- Stage 1: form while advancing -------------------------------------
    form_ax = anchor0[0] + form_d
    g1 = wide_at(form_ax)
    paths = {r: _line_points(init_poses[r],
                             (g1[slot_ids[r]][0], g1[slot_ids[r]][1], 0.0), step)
             for r in init_poses}
    stages.append({'name': 'form', 'paths': paths, 'duration': form_d / ref_v})

    # ---- Stage 2: straight advance -----------------------------------------
    straight_ax = form_ax + straight_d
    g2 = wide_at(straight_ax)
    paths = {r: _line_points((g1[slot_ids[r]][0], g1[slot_ids[r]][1], 0.0),
                             (g2[slot_ids[r]][0], g2[slot_ids[r]][1], 0.0), step)
             for r in init_poses}
    stages.append({'name': 'straight1', 'paths': paths, 'duration': straight_d / ref_v})

    # ---- Stage 3: NW stops, other five advance drop_d ----------------------
    drop_ax = straight_ax + drop_d
    g3 = wide_at(drop_ax)
    paths = {}
    for r in init_poses:
        if r == nw_robot:
            p = (g2[slot_ids[r]][0], g2[slot_ids[r]][1], 0.0)
            paths[r] = [p, p]          # stationary hold
        else:
            paths[r] = _line_points((g2[slot_ids[r]][0], g2[slot_ids[r]][1], 0.0),
                                    (g3[slot_ids[r]][0], g3[slot_ids[r]][1], 0.0), step)
    stages.append({'name': 'drop', 'paths': paths, 'duration': drop_d / ref_v})

    # ---- Stage 4: five crawl 20 m; SW turns back to NW's side --------------
    slow_ax = drop_ax + slow_d
    g4 = wide_at(slow_ax)
    nw_hold = (g2[nw_slot][0], g2[nw_slot][1], 0.0)
    meeting_sw = (nw_hold[0] + rel_sw[0], nw_hold[1] + rel_sw[1], 0.0)
    paths = {}
    for r in init_poses:
        if r == nw_robot:
            paths[r] = [nw_hold, nw_hold]
        elif r == sw_robot:
            start = (g3[slot_ids[r]][0], g3[slot_ids[r]][1], 0.0)
            paths[r] = dubins_fn(start, meeting_sw)
        else:
            paths[r] = _line_points((g3[slot_ids[r]][0], g3[slot_ids[r]][1], 0.0),
                                    (g4[slot_ids[r]][0], g4[slot_ids[r]][1], 0.0), step)
    stages.append({'name': 'slow_return', 'paths': paths, 'duration': slow_d / slow_v})

    # ---- Stage 5: stragglers catch up into original slots ------------------
    # Catch-up time solves: rejoin_v * T = (gap) + slow_v * T, where the gap
    # from the held NW position to its slot in the column at g4 is
    # slow_ax - straight_ax = drop_d + slow_d.
    gap = drop_d + slow_d
    rejoin_t = gap / (rejoin_v - slow_v)
    rejoin_ax = slow_ax + slow_v * rejoin_t
    g5 = wide_at(rejoin_ax)
    paths = {}
    for r in init_poses:
        if r == nw_robot:
            paths[r] = _line_points(nw_hold,
                                    (g5[nw_slot][0], g5[nw_slot][1], 0.0), step)
        elif r == sw_robot:
            paths[r] = _line_points(meeting_sw,
                                    (g5[sw_slot][0], g5[sw_slot][1], 0.0), step)
        else:
            paths[r] = _line_points((g4[slot_ids[r]][0], g4[slot_ids[r]][1], 0.0),
                                    (g5[slot_ids[r]][0], g5[slot_ids[r]][1], 0.0), step)
    stages.append({'name': 'rejoin', 'paths': paths, 'duration': rejoin_t})

    # ---- Stage 6: full formation advances final_d --------------------------
    final_ax = rejoin_ax + final_d
    g6 = wide_at(final_ax)
    paths = {r: _line_points((g5[slot_ids[r]][0], g5[slot_ids[r]][1], 0.0),
                             (g6[slot_ids[r]][0], g6[slot_ids[r]][1], 0.0), step)
             for r in init_poses}
    stages.append({'name': 'straight2', 'paths': paths, 'duration': final_d / ref_v})

    info = {'nw_robot': nw_robot, 'sw_robot': sw_robot,
            'nw_slot': nw_slot, 'sw_slot': sw_slot,
            'meeting_sw': meeting_sw, 'rejoin_t': rejoin_t,
            'rejoin_ax': rejoin_ax}
    return stages, info
