"""Offline geometric candidate for a six-vehicle dropout and rendezvous.

This is not a trajectory publisher or a vehicle controller.  Its piecewise
linear kinematics have instantaneous velocity changes and require a separate
feasibility and platform execution design before use as motion commands.
"""
import math

from hera_forward.formation_switch_logic import assign_slots, formation_targets, formation_templates


def make_scenario(initial, spacing=15.0, formation_advance=50.0,
                  initial_advance=50.0, separation_advance=20.0,
                  final_advance=50.0, cruise_speed=3.0, slow_speed=1.0,
                  rescue_speed=3.0, catchup_speed=4.0, minimum_distance=5.0):
    """Return timed XY knots, assigned roles and exact linear separation audit.

    Map coordinates are ENU and the formation travels east.  The northern and
    southern slots in the rear (western) row are respectively the dropout and
    rescuer.  The rescue vehicle returns along its own lane until abreast of the
    stopped vehicle, then both catch the four moving vehicles in their own lanes.
    """
    if set(initial) != set(range(1, 7)):
        raise ValueError('initial states must contain exactly ugv1..ugv6')
    vals = (spacing, formation_advance, initial_advance, separation_advance,
            final_advance, cruise_speed, slow_speed, rescue_speed, catchup_speed,
            minimum_distance)
    if not all(math.isfinite(v) and v > 0 for v in vals):
        raise ValueError('distances and speeds must be finite and positive')
    if slow_speed >= cruise_speed or catchup_speed <= slow_speed:
        raise ValueError('slow speed must be below cruise; catchup must exceed slow')
    if not all(len(p) >= 2 and all(math.isfinite(v) for v in p[:2]) for p in initial.values()):
        raise ValueError('initial positions must be finite')
    wide, _ = formation_templates(spacing, spacing)
    ax, ay = initial[1][:2]
    form = formation_targets(ax + formation_advance, ay, 0.0, wide)
    assignment = assign_slots(initial, form)
    roles = {slot: robot for robot, slot in assignment.items()}
    dropout, rescuer = roles[5], roles[6]
    formed = {r: form[assignment[r]][:2] for r in initial}
    duration_form = max(math.dist(initial[r][:2], formed[r]) for r in initial) / cruise_speed
    if duration_form <= 0:
        raise ValueError('formation stage has zero duration')
    times = [0.0, duration_form]
    positions = [{r: tuple(initial[r][:2]) for r in initial}, formed]

    def add(duration, points):
        if duration <= 0 or not math.isfinite(duration):
            raise ValueError('phase duration must be positive and finite')
        times.append(times[-1] + duration)
        positions.append(points)

    def advance(points, distance, robots):
        return {r: (p[0] + (distance if r in robots else 0.0), p[1])
                for r, p in points.items()}

    everyone = set(initial)
    add(initial_advance / cruise_speed, advance(positions[-1], initial_advance, everyone))
    add(separation_advance / cruise_speed,
        advance(positions[-1], separation_advance, everyone - {dropout}))
    # Rescuer retreats in its original lane; four vehicles continue slowly.
    t_return = separation_advance / rescue_speed
    prev = positions[-1]
    add(t_return, {r: (p[0] + (-separation_advance if r == rescuer else
                                0.0 if r == dropout else slow_speed * t_return), p[1])
                   for r, p in prev.items()})
    # Close the longitudinal gap at positive relative speed in each original lane.
    gap = separation_advance + slow_speed * t_return
    t_catch = gap / (catchup_speed - slow_speed)
    prev = positions[-1]
    add(t_catch, {r: (p[0] + (catchup_speed if r in {dropout, rescuer}
                                else slow_speed) * t_catch, p[1])
                  for r, p in prev.items()})
    add(final_advance / cruise_speed, advance(positions[-1], final_advance, everyone))

    closest = (math.inf, None, None)
    for k, (a, b) in enumerate(zip(positions, positions[1:])):
        for r in sorted(initial):
            for s in range(r + 1, 7):
                px, py = a[r][0] - a[s][0], a[r][1] - a[s][1]
                vx = (b[r][0] - b[s][0]) - px
                vy = (b[r][1] - b[s][1]) - py
                vv = vx * vx + vy * vy
                u = max(0.0, min(1.0, -(px * vx + py * vy) / vv)) if vv else 0.0
                d = math.hypot(px + u * vx, py + u * vy)
                if d < closest[0]:
                    closest = (d, (r, s), times[k] + u * (times[k + 1] - times[k]))
    if closest[0] < minimum_distance:
        raise ValueError(f'candidate separation {closest[0]:.3f} m below {minimum_distance:.3f} m')
    return {'times_s': times, 'positions': positions, 'assignment': assignment,
            'dropout': dropout, 'rescuer': rescuer, 'minimum_separation': closest,
            'phases': ['form', 'advance_50m', 'dropout_20m', 'rescue_return',
                       'catch_up', 'advance_50m_final']}
