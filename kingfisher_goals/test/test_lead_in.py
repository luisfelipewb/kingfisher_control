"""The lead-in controller and the heading helpers, without ROS."""

from math import pi

from geometry_msgs.msg import Pose
from kingfisher_goals.geometry import heading_error, wrap
from kingfisher_goals.lead_in import LeadIn
import pytest


def controller(**gains):
    defaults = {'k_heading': 1.0, 'k_yaw_rate': 1.0, 'max_turn': 0.5, 'max_accel': 100.0,
                'feedforward': 1.0, 'k_speed': 1.0, 'k_integral': 0.5, 'max_integral': 0.5}
    return LeadIn(**{**defaults, **gains})


def test_straight_at_speed_is_feedforward():
    assert controller().update(0.0, 0.5, 0.5, 0.0, 0.1) == pytest.approx((0.5, 0.5))


def test_turns_counter_clockwise_with_more_right_thrust():
    left, right = controller().update(0.3, 0.5, 0.5, 0.0, 0.1)
    assert right > left


def test_yaw_rate_damps_the_turn():
    left, right = controller().update(0.0, 0.5, 0.5, 0.4, 0.1)
    assert left > right


def test_turn_and_thrust_saturate():
    left, right = controller().update(pi, 0.0, 0.0, 0.0, 0.1)
    assert (left, right) == pytest.approx((-0.5, 0.5))
    left, right = controller().update(0.0, 5.0, 0.0, 0.0, 0.1)
    assert (left, right) == pytest.approx((1.0, 1.0))


def test_no_reverse_thrust_when_too_fast():
    assert controller().update(0.0, 0.5, 2.0, 0.0, 0.1) == pytest.approx((0.0, 0.0))


def test_integral_is_clamped_and_reset():
    lead_in = controller(feedforward=0.0, k_speed=0.0)
    for _ in range(1000):
        lead_in.update(0.0, 0.5, 0.0, 0.0, 0.1)
    assert lead_in.update(0.0, 0.5, 0.0, 0.0, 0.1) == pytest.approx((0.5, 0.5))
    lead_in.reset(0.5)
    assert lead_in.update(0.0, 0.5, 0.5, 0.0, 0.1) == pytest.approx((0.0, 0.0))


def test_setpoint_ramps_from_the_reset_speed():
    lead_in = controller(max_accel=0.1, k_speed=0.0, k_integral=0.0)
    lead_in.reset(0.2)
    assert lead_in.update(0.0, 0.5, 0.2, 0.0, 1.0) == pytest.approx((0.3, 0.3))
    for _ in range(10):
        lead_in.update(0.0, 0.5, 0.2, 0.0, 1.0)
    assert lead_in.update(0.0, 0.5, 0.2, 0.0, 1.0) == pytest.approx((0.5, 0.5))


def test_setpoint_drops_at_once():
    lead_in = controller(max_accel=0.1, k_speed=0.0, k_integral=0.0)
    lead_in.reset(0.5)
    assert lead_in.update(0.0, 0.0, 0.5, 0.0, 0.1) == pytest.approx((0.0, 0.0))


def test_no_integral_while_ramping():
    lead_in = controller(max_accel=0.1, feedforward=0.0, k_speed=0.0)
    lead_in.reset(0.0)
    for _ in range(10):
        lead_in.update(0.0, 0.5, 0.0, 0.0, 0.1)
    assert lead_in.update(0.0, 0.5, 0.0, 0.0, 0.1) == pytest.approx((0.0, 0.0))


def test_wrap():
    assert wrap(3 * pi / 2) == pytest.approx(-pi / 2)
    assert wrap(-3 * pi / 2) == pytest.approx(pi / 2)


def test_heading_error_across_pi():
    pose = Pose()
    pose.orientation.z, pose.orientation.w = 1.0, 0.0     # facing -x
    assert heading_error(pose, [-1.0, -0.1]) == pytest.approx(0.0997, abs=1e-3)
    assert heading_error(pose, [-1.0, 0.1]) == pytest.approx(-0.0997, abs=1e-3)
