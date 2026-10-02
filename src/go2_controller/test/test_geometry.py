import math

import pytest

from go2_controller.core.geometry import (
    CourseFrame, Pose, bearing_to, distance, to_local, to_world, wrap, yaw_from_quaternion)


@pytest.mark.parametrize("angle, expected", [
    (0.0, 0.0),
    (math.pi / 2, math.pi / 2),
    (3 * math.pi / 2, -math.pi / 2),
    (-3 * math.pi / 2, math.pi / 2),
    (4 * math.pi + 0.1, 0.1),
])
def test_wrap(angle, expected):
    assert wrap(angle) == pytest.approx(expected, abs=1e-12)


def test_wrap_stays_in_range():
    for k in range(-50, 50):
        a = wrap(0.37 * k)
        assert -math.pi <= a <= math.pi


def test_local_world_roundtrip():
    pose = Pose(1.5, -2.0, 0.7)
    for wx, wy in [(0.0, 0.0), (3.0, 1.0), (-4.2, 7.7)]:
        lx, ly = to_local(pose, wx, wy)
        bx, by = to_world(pose, lx, ly)
        assert (bx, by) == pytest.approx((wx, wy), abs=1e-12)


def test_to_local_axes():
    pose = Pose(1.0, 1.0, math.pi / 2)
    assert to_local(pose, 1.0, 3.0) == pytest.approx((2.0, 0.0), abs=1e-12)
    assert to_local(pose, 0.0, 1.0) == pytest.approx((0.0, 1.0), abs=1e-12)


def test_bearing_and_distance():
    pose = Pose(0.0, 0.0, math.pi / 2)
    assert bearing_to(pose, 0.0, 5.0) == pytest.approx(0.0, abs=1e-12)
    assert bearing_to(pose, -1.0, 0.0) == pytest.approx(math.pi / 2, abs=1e-12)
    assert bearing_to(pose, 1.0, 0.0) == pytest.approx(-math.pi / 2, abs=1e-12)
    assert distance(pose, 3.0, 4.0) == pytest.approx(5.0)


@pytest.mark.parametrize("yaw", [0.0, 0.5, -2.0, math.pi - 1e-6])
def test_yaw_from_quaternion(yaw):
    q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    assert yaw_from_quaternion(*q) == pytest.approx(yaw, abs=1e-9)


def test_course_frame():
    course = CourseFrame(Pose(2.0, 3.0, math.pi / 2))
    assert course.to_course(2.0, 5.0) == pytest.approx((2.0, 0.0), abs=1e-12)
    assert course.to_course(1.0, 3.0) == pytest.approx((0.0, 1.0), abs=1e-12)
    assert course.to_world(2.0, 0.0) == pytest.approx((2.0, 5.0), abs=1e-12)
    assert course.to_world(0.0, 1.0) == pytest.approx((1.0, 3.0), abs=1e-12)
