"""Pure geometry for the three-stage formation-switch demonstration.

Stages (all built around the advanced, east-shifted anchor, so the swarm forms
its shape *while* advancing instead of forming in place first):

  1. ``wide_2x3``       -- advance east and form the wide formation in one motion.
  2. ``single_column``  -- switch to a single file around the advanced anchor.
  3. ``wide_2x3_return``-- switch back to the wide formation.

There is deliberately no live collision monitor: the slot spacings (wide 15 m /
column 20 m) and the globally optimal slot assignment keep the offline paths
from crossing.
"""

import math

from scipy.optimize import linear_sum_assignment


FIRST_FORMATION = 'wide_2x3'
SWITCH_FORMATION = 'single_column'
RETURN_FORMATION = 'wide_2x3_return'


def normalize_angle(angle):
    """Return an angle in [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def formation_templates(wide_spacing_m, column_spacing_m):
    """Return the wide (2x3) and single-column slot offsets in metres.

    Slots are leader-relative ``(offset_x, offset_y)`` in the leader frame:
    x is forward, y is left.  ``wide`` is two columns by three rows; ``column``
    is a single file behind the leader.
    """
    if not all(math.isfinite(v) and v > 0.0 for v in (wide_spacing_m, column_spacing_m)):
        raise ValueError('template spacings must be finite positive metres')
    w, c = wide_spacing_m, column_spacing_m
    wide = {1: (0.0, 0.0), 2: (0.0, -w), 3: (-w, 0.0),
            4: (-w, -w), 5: (-2 * w, 0.0), 6: (-2 * w, -w)}
    column = {i: (-(i - 1) * c, 0.0) for i in range(1, 7)}
    return wide, column


def formation_targets(anchor_x, anchor_y, heading, slots):
    """Rotate the leader-relative slot offsets about the anchor."""
    cosine, sine = math.cos(heading), math.sin(heading)
    return {
        slot: (anchor_x + cosine * ox - sine * oy,
               anchor_y + sine * ox + cosine * oy, heading)
        for slot, (ox, oy) in slots.items()
    }
# 返回的内容：
# {
#     1: (x1, y1, heading),
#     2: (x2, y2, heading),
#     ...
#     6: (x6, y6, heading),
# }


def advance_anchor(anchor, distance_m):
    """Translate a leader anchor ``(x, y, yaw)`` due east by ``distance_m``.

    The advance is a fixed +x translation; it does not follow the leader's yaw.
    """
    if not all(math.isfinite(v) for v in (*anchor, distance_m)):
        raise ValueError('anchor components and advance distance must be finite')
    if distance_m < 0.0:
        raise ValueError('advance distance must be nonnegative')
    x, y, yaw = anchor
    return (x + distance_m, y, yaw)


def build_stage_plan(anchor, advance_distance_m, wide_slots, column_slots):
    """Return the three ordered stage descriptors.

    Each descriptor is ``{'name', 'anchor', 'slots'}``.  Every stage anchors at
    a position shifted further east, so the swarm forms and switches shape
    *while* advancing instead of stopping to reconfigure in place.
    """
    if not wide_slots or not column_slots:
        raise ValueError('both formation templates must contain slots')
    # Formation orientation is fixed due east (heading 0), independent of the
    # leader's measured yaw.
    anchor = (anchor[0], anchor[1], 0.0)
    first = advance_anchor(anchor, advance_distance_m)
    second = advance_anchor(first, advance_distance_m)
    third = advance_anchor(second, advance_distance_m)
    return [
        {'name': FIRST_FORMATION, 'anchor': first, 'slots': wide_slots},
        {'name': SWITCH_FORMATION, 'anchor': second, 'slots': column_slots},
        {'name': RETURN_FORMATION, 'anchor': third, 'slots': wide_slots},
    ]


def assign_slots(states, slot_targets):
    """Map robots to slots with a globally optimal (Hungarian) assignment.

    The cost is the squared robot-to-slot distance, minimised in total so no
    vehicle is left with a far slot merely because it was considered last.
    ``states`` and ``slot_targets`` are keyed by robot id and slot id and must
    hold the same six ids.
    """
    if set(states) != set(slot_targets):
        raise ValueError('robots and slot ids must be the same set')
    robots = sorted(states)
    slots = sorted(slot_targets)
    cost = [
        [(states[r][0] - slot_targets[s][0]) ** 2 +
         (states[r][1] - slot_targets[s][1]) ** 2 for s in slots]
        for r in robots]
    rows, cols = linear_sum_assignment(cost)
    return {robots[i]: slots[j] for i, j in zip(rows, cols)}


def targets_for_assignment(slot_targets, assignment):
    """Re-key slot world positions by robot id using the chosen assignment."""
    return {robot: slot_targets[slot] for robot, slot in assignment.items()}


def build_continuous_formation_trajectory(init_poses, advance_distance_m,
                                          wide_slots, column_slots,
                                          sample_step_m=2.0, hold_distance_m=0.0):
    """Build six continuous trajectories synchronized by a shared anchor.

    The swarm forms ``wide_2x3`` while advancing the first ``advance_distance_m``,
    switches to ``single_column`` over the second advance, switches back to
    ``wide_2x3`` over the third, then keeps the wide formation and advances a
    final ``hold_distance_m`` before stopping (so it comes to rest clearly in
    the wide shape instead of right as the last switch completes).  Every
    vehicle is indexed by the same anchor progress ``s``, so at any shared ``s``
    the vehicles occupy the interpolated slots and keep their nominal
    separation (>= 12 m for the default 15 m / 20 m spacings).

    Returns ``(trajectories, slot_ids)`` where ``trajectories`` is
    ``{robot: [(x, y, yaw), ...]}`` sampled every ``sample_step_m`` metres of
    anchor progress, and ``slot_ids`` records each robot's fixed slot index.
    """
    if not init_poses or not wide_slots or not column_slots:
        raise ValueError('initial poses and both templates are required')
    if not math.isfinite(advance_distance_m) or advance_distance_m <= 0.0:
        raise ValueError('advance distance must be finite and positive')
    if not math.isfinite(sample_step_m) or sample_step_m <= 0.0:
        raise ValueError('sample step must be finite and positive')
    if not math.isfinite(hold_distance_m) or hold_distance_m < 0.0:
        raise ValueError('hold distance must be finite and nonnegative')

    anchor = (init_poses[1][0], init_poses[1][1], 0.0)
    wide_targets = formation_targets(
        anchor[0] + advance_distance_m, anchor[1], 0.0, wide_slots)
    slot_ids = assign_slots(init_poses, wide_targets)
    switch_end = 3.0 * advance_distance_m
    total = switch_end + hold_distance_m

    def slot_position(robot, s):
        sid = slot_ids[robot]
        w, c = wide_slots[sid], column_slots[sid]
        if s <= advance_distance_m:
            f = s / advance_distance_m
            wx = anchor[0] + advance_distance_m + w[0]
            wy = anchor[1] + w[1]
            return (init_poses[robot][0] * (1.0 - f) + wx * f,
                    init_poses[robot][1] * (1.0 - f) + wy * f, 0.0)
        if s <= 2.0 * advance_distance_m:
            p = (s - advance_distance_m) / advance_distance_m
            return (anchor[0] + s + (1.0 - p) * w[0] + p * c[0],
                    anchor[1] + (1.0 - p) * w[1] + p * c[1], 0.0)
        if s <= switch_end:
            p = (s - 2.0 * advance_distance_m) / advance_distance_m
            return (anchor[0] + s + (1.0 - p) * c[0] + p * w[0],
                    anchor[1] + (1.0 - p) * c[1] + p * w[1], 0.0)
        # Hold the wide formation and keep advancing east.
        return (anchor[0] + s + w[0], anchor[1] + w[1], 0.0)

    trajectories = {}
    for robot in init_poses:
        points = []
        s = 0.0
        while s <= total + 1e-9:
            points.append(slot_position(robot, min(s, total)))
            s += sample_step_m
        trajectories[robot] = points
    return trajectories, slot_ids


def build_formation_advance_turn_trajectory(init_poses, advance_distance_m,
                                            turn_angle_rad, turn_radius_m,
                                            wide_slots, sample_step_m=2.0):
    """Build six continuous trajectories: form wide_2x3, advance straight, turn
    right as a rigid body, then keep advancing.

    Timeline (shared anchor progress ``s``):
      1. ``[0, advance]``           -- form wide_2x3 while advancing east.
      2. ``[advance, 2*advance]``   -- hold wide_2x3 and advance east.
      3. ``[2*advance, +turn_arc]`` -- turn right by ``turn_angle_rad`` about the
                                      formation's right (inner) side.  Every
                                      vehicle rotates about the same centre, so
                                      outer vehicles travel a longer arc (faster)
                                      and inner ones a shorter arc (slower).
      4. remainder                  -- keep advancing west another ``advance``.

    The turn centre sits ``turn_radius_m`` to the right (south, for an eastward
    heading) of the formation centre, so a right turn is a clockwise rotation
    about that centre.
    """
    if not init_poses or not wide_slots:
        raise ValueError('initial poses and wide template are required')
    if not math.isfinite(advance_distance_m) or advance_distance_m <= 0.0:
        raise ValueError('advance distance must be finite and positive')
    if not math.isfinite(turn_angle_rad) or turn_angle_rad <= 0.0:
        raise ValueError('turn angle must be finite and positive')
    if not math.isfinite(turn_radius_m) or turn_radius_m <= 0.0:
        raise ValueError('turn radius must be finite and positive')
    if not math.isfinite(sample_step_m) or sample_step_m <= 0.0:
        raise ValueError('sample step must be finite and positive')

    anchor = (init_poses[1][0], init_poses[1][1], 0.0)
    form_targets = formation_targets(
        anchor[0] + advance_distance_m, anchor[1], 0.0, wide_slots)
    slot_ids = assign_slots(init_poses, form_targets)

    straight_end = 2.0 * advance_distance_m
    turn_arc = turn_radius_m * turn_angle_rad
    turn_start = (anchor[0] + straight_end, anchor[1])
    center = (turn_start[0], turn_start[1] - turn_radius_m)
    total = straight_end + turn_arc + advance_distance_m

    def position(robot, s):
        sid = slot_ids[robot]
        w = wide_slots[sid]
        if s <= advance_distance_m:
            f = s / advance_distance_m
            wx = anchor[0] + advance_distance_m + w[0]
            wy = anchor[1] + w[1]
            return (init_poses[robot][0] * (1.0 - f) + wx * f,
                    init_poses[robot][1] * (1.0 - f) + wy * f, 0.0)
        if s <= straight_end:
            return (anchor[0] + s + w[0], anchor[1] + w[1], 0.0)
        if s <= straight_end + turn_arc:
            theta = (s - straight_end) / turn_radius_m
            vx = w[0]
            vy = w[1] + turn_radius_m
            rx = vx * math.cos(theta) + vy * math.sin(theta)
            ry = -vx * math.sin(theta) + vy * math.cos(theta)
            return (center[0] + rx, center[1] + ry, -theta)
        # Turn complete: heading is -turn_angle (west for a 180 deg turn); keep
        # advancing along that heading.
        distance = s - (straight_end + turn_arc)
        theta = turn_angle_rad
        vx = w[0]
        vy = w[1] + turn_radius_m
        rx = vx * math.cos(theta) + vy * math.sin(theta)
        ry = -vx * math.sin(theta) + vy * math.cos(theta)
        end = (center[0] + rx, center[1] + ry)
        heading = -theta
        return (end[0] + distance * math.cos(heading),
                end[1] + distance * math.sin(heading), heading)

    trajectories = {}
    for robot in init_poses:
        points = []
        s = 0.0
        while s <= total + 1e-9:
            points.append(position(robot, min(s, total)))
            s += sample_step_m
        trajectories[robot] = points
    return trajectories, slot_ids


def advance_turn_stage_targets(init_poses, advance_distance_m, turn_angle_rad,
                               turn_radius_m, wide_slots):
    """Return the three stage descriptors for form -> advance+turn -> straight.

    Stage goals are keyed by robot id and expressed as ``(x, y, yaw)``:
      1. ``form``         -- form wide_2x3 *while* advancing east: the vehicles
                             move from their scattered starts to the wide slots
                             anchored ``advance_distance_m`` ahead of the leader.
      2. ``advance_turn`` -- hold the formation, advance another
                             ``advance_distance_m``, then turn right
                             ``turn_angle_rad`` about the turn centre.  The stage
                             also carries ``straight_goals`` (the formation pose
                             right after the straight advance, where the rigid
                             turn begins).
      3. ``straight``     -- the turned formation shifted ``advance_distance_m``
                             along its new heading.

    Also returns ``slot_ids`` (robot -> fixed slot index) and the turn centre, so
    the publisher can build a rigid-rotation path for the turn.
    """
    if not init_poses or not wide_slots:
        raise ValueError('initial poses and wide template are required')
    anchor = (init_poses[1][0], init_poses[1][1], 0.0)
    form_anchor = (anchor[0] + advance_distance_m, anchor[1])
    form_targets = formation_targets(form_anchor[0], form_anchor[1], 0.0, wide_slots)
    slot_ids = assign_slots(init_poses, form_targets)

    def slot_goal(robot, ax, ay, heading):
        w = wide_slots[slot_ids[robot]]
        return (ax + w[0], ay + w[1], heading)

    form_goals = {r: slot_goal(r, form_anchor[0], form_anchor[1], 0.0) for r in init_poses}
    straight_goals = {r: slot_goal(r, anchor[0] + 2.0 * advance_distance_m, anchor[1], 0.0)
                      for r in init_poses}

    # Turn centre directly below the formation's GEOMETRIC centre (x aligned).
    # This keeps front/back rows symmetric about the turn centre. Their radii
    # still depend on both longitudinal and lateral slot offsets.
    turn_start = (anchor[0] + 2.0 * advance_distance_m, anchor[1])
    slot_offsets = list(wide_slots.values())
    center_offset_x = sum(o[0] for o in slot_offsets) / len(slot_offsets)
    center_offset_y = sum(o[1] for o in slot_offsets) / len(slot_offsets)
    center = (turn_start[0] + center_offset_x,
              turn_start[1] + center_offset_y - turn_radius_m)
    # Rigid-rotation endpoint so stage 3 starts exactly where the turn ends.
    turn_goals = {}
    for robot in init_poses:
        w = wide_slots[slot_ids[robot]]
        vx, vy = w[0] - center_offset_x, w[1] - center_offset_y + turn_radius_m
        rx = vx * math.cos(turn_angle_rad) + vy * math.sin(turn_angle_rad)
        ry = -vx * math.sin(turn_angle_rad) + vy * math.cos(turn_angle_rad)
        turn_goals[robot] = (center[0] + rx, center[1] + ry, -turn_angle_rad)

    heading = -turn_angle_rad
    straight2_goals = {
        r: (turn_goals[r][0] + advance_distance_m * math.cos(heading),
            turn_goals[r][1] + advance_distance_m * math.sin(heading), heading)
        for r in init_poses}

    stages = [
        {'name': 'form', 'goals': form_goals},
        {'name': 'advance_turn', 'goals': turn_goals, 'straight_goals': straight_goals},
        {'name': 'straight', 'goals': straight2_goals},
    ]
    return stages, slot_ids, center


def two_lane_turn_points(start_poses, slot_ids, center, turn_angle_rad,
                         wide_slots, lane_spacing_m, sample_step_m=1.0):
    """Two shared arcs: the inner column and outer column each follow ONE arc.

    Vehicles are split by their slot's lateral offset.  All vehicles of a column
    ride the same geometric arc (same radius), so within a column every vehicle
    has the same speed.  The outer column's radius is ``lane_spacing_m`` larger,
    hence faster by that radius ratio at the same angular rate.

    Because the wide formation's three vehicles per column are spread along a
    straight line before the turn, they are not exactly on one circle.  The
    shared radius is the *mean* of the three actual radii (smallest entry jump),
    and the first few samples blend from each vehicle's true start pose onto the
    shared arc so the trajectory is continuous.

    ``slot_ids`` maps robot -> slot.  Returns ``{robot: [(x, y, yaw), ...]}``
    with a common point count; same index = same rotation progress for all.
    """
    if not start_poses or not wide_slots:
        raise ValueError('start poses and wide template are required')
    leader_slot = min(wide_slots, key=lambda s: abs(wide_slots[s][0]) + abs(wide_slots[s][1]))
    base_lateral = wide_slots[leader_slot][1]
    outer_slots = {s for s, o in wide_slots.items() if abs(o[1] - base_lateral) < 1e-9}
    inner_slots = set(wide_slots) - outer_slots

    # Mean radius of each column from the three vehicles' actual start poses.
    def column_robots(slots):
        return [r for r in start_poses if slot_ids[r] in slots]

    inner_robots = column_robots(inner_slots)
    outer_robots = column_robots(outer_slots)
    r_inner = sum(math.hypot(start_poses[r][0] - center[0],
                             start_poses[r][1] - center[1]) for r in inner_robots) / len(inner_robots)
    # Keep both arcs concentric and a fixed column-spacing apart.
    r_outer = r_inner + lane_spacing_m
    radius_by_robot = {r: (r_outer if slot_ids[r] in outer_slots else r_inner)
                       for r in start_poses}

    max_radius = max(r_inner, r_outer)
    angle_step = sample_step_m / max_radius
    count = max(2, int(math.ceil(turn_angle_rad / angle_step)) + 1)
    transition = max(1, count // 10)  # first 10% blends onto the shared arc

    points_by_robot = {}
    for r in start_poses:
        x, y, yaw = start_poses[r]
        radius = radius_by_robot[r]
        theta_start = math.atan2(y - center[1], x - center[0])
        arc_start = (center[0] + radius * math.cos(theta_start),
                     center[1] + radius * math.sin(theta_start))
        points = []
        for k in range(count):
            progress = turn_angle_rad * k / (count - 1)
            theta = theta_start - progress
            ax = center[0] + radius * math.cos(theta)
            ay = center[1] + radius * math.sin(theta)
            if k < transition:
                f = (k + 1) / transition
                px = x + (ax - arc_start[0]) * f + (arc_start[0] - x) * (1 - f)
                py = y + (ay - arc_start[1]) * f + (arc_start[1] - y) * (1 - f)
            else:
                px, py = ax, ay
            points.append((px, py, yaw - progress))
        points_by_robot[r] = points
    return points_by_robot


def rigid_turn_points(start_poses, center, turn_angle_rad, sample_step_m=2.0):
    """Return per-robot arc points for a clockwise rigid rotation about a centre.

    ``start_poses`` are the ``{robot: (x, y, yaw)}`` poses at the start of the
    turn; each vehicle rotates clockwise by ``turn_angle_rad`` about ``center``.
    All vehicles share the same angular sampling (same number of points), so the
    same point index always means the same rotation angle and the rigid
    separation is preserved.  The arc length differs per vehicle (outer longer,
    inner shorter), which the caller turns into the faster-outer / slower-inner
    speed split.
    """
    max_radius = max(
        math.hypot(x - center[0], y - center[1]) for x, y, _ in start_poses.values())
    angle_step = sample_step_m / max_radius
    count = max(2, int(math.ceil(turn_angle_rad / angle_step)) + 1)
    points_by_robot = {}
    for robot, (x, y, yaw) in start_poses.items():
        vx, vy = x - center[0], y - center[1]
        points = []
        for k in range(count):
            theta = turn_angle_rad * k / (count - 1)
            rx = vx * math.cos(theta) + vy * math.sin(theta)
            ry = -vx * math.sin(theta) + vy * math.cos(theta)
            points.append((center[0] + rx, center[1] + ry, yaw - theta))
        points_by_robot[robot] = points
    return points_by_robot
