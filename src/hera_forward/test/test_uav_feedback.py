import math
import pytest
from hera_forward.uav_feedback_logic import segment_clearance, tracking_velocity


def test_height_error_uses_metres_and_vertical_limit():
    assert tracking_velocity((0, 0, 4.5), (0, 0, 5), (0, 0, 0), .7, .8, 1, .4) == (0, 0, .4)
    assert tracking_velocity((0, 0, 6), (0, 0, 5), (0, 0, 0), .7, .8, 1, .4)[2] == -.4


def test_horizontal_vector_limit_preserves_direction():
    v = tracking_velocity((0, 0, 5), (10, 10, 5), (0, 0, 0), .7, .8, 1, .4)
    assert math.hypot(*v[:2]) == pytest.approx(1)
    assert v[0] == v[1]


def test_segments_cannot_cross_between_safe_endpoints():
    assert segment_clearance((-5, 0), (5, 0), (0, -5), (0, 5)) == 0
    assert segment_clearance((0, 0), (6, 0), (0, 10), (6, 10)) == 10


def test_nonfinite_feedback_is_rejected():
    with pytest.raises(ValueError):
        tracking_velocity((0, 0, float('nan')), (0, 0, 5), (0, 0, 0), .7, .8, 1, .4)


def test_height_damping_brakes_an_upward_overshoot():
    v = tracking_velocity((0, 0, 5.5), (0, 0, 6), (0, 0, 0), .7, .35, 1, .4,
                          vertical_velocity=.3, height_damping=.8)
    assert v[2] < 0
