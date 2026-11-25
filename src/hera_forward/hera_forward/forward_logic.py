"""Pure helpers for the six-vehicle forward advance demonstration.

These functions are offline-testable: they only read a recorded pose snapshot
and turn it into per-vehicle straight reference geometry.  They intentionally
do not talk to the platform, claim obstacle avoidance, or perform any motion.
"""

import math

import yaml


def load_initial_poses(path):
    """Read the recorded six-vehicle scene snapshot.

    Returns ``{robot: (x_m, y_m, z_m, yaw_rad)}`` for robots 1..6.  The source
    file is the live measured snapshot, so these are the platform's current
    poses and are treated as the fixed start of every advance.
    """
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    vehicles = data.get('vehicles') if isinstance(data, dict) else None
    if not vehicles:
        raise ValueError(f'{path}: missing vehicles list')
    poses = {}
    for vehicle in vehicles:
        name = str(vehicle.get('name', ''))
        if not name.startswith('ugv'):
            continue
        robot = int(name[3:])
        poses[robot] = (float(vehicle['x_m']), float(vehicle['y_m']),
                        float(vehicle['z_m']), float(vehicle['yaw_rad']))
    if set(poses) != set(range(1, 7)):
        raise ValueError(f'{path}: expected ugv1..ugv6, got {sorted(poses)}')
    return poses


def advance_goals(poses, distance_m):
    """Translate every pose east by ``distance_m``, preserving the formation.

    All six vehicles receive the same (+distance_m, 0) offset, so their
    relative geometry (the formation) is unchanged.  Yaw is kept as recorded.
    """
    if not math.isfinite(distance_m) or distance_m <= 0.0:
        raise ValueError('advance distance must be finite and positive')
    return {robot: (x + distance_m, y, z, yaw)
            for robot, (x, y, z, yaw) in poses.items()}


def line_samples(start, goal, max_segment_m):
    """Sample a straight map-frame path, preserving the terminal yaw.

    ``start`` and ``goal`` are ``(x, y, yaw)`` tuples.  Returns a list of
    ``(x, y, yaw)`` points spaced at most ``max_segment_m`` apart.
    """
    if not math.isfinite(max_segment_m) or max_segment_m <= 0.0:
        raise ValueError('max_segment_m must be finite and positive')
    start_x, start_y, start_yaw = start
    goal_x, goal_y, goal_yaw = goal
    distance = math.hypot(goal_x - start_x, goal_y - start_y)
    if distance <= 1e-9:
        return [(start_x, start_y, start_yaw)]
    count = max(2, int(math.ceil(distance / max_segment_m)) + 1)
    travel_yaw = math.atan2(goal_y - start_y, goal_x - start_x)
    points = []
    for index in range(count):
        fraction = index / (count - 1)
        yaw = travel_yaw if index < count - 1 else goal_yaw
        points.append((start_x + fraction * (goal_x - start_x),
                       start_y + fraction * (goal_y - start_y), yaw))
    return points
