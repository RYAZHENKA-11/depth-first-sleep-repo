import math

import numpy as np
import pytest

from go2_controller.core.geometry import Pose
from go2_controller.core.scan_utils import (
    TimeWindowMean, corridor_free_distance, make_scan, scan_points)
from synthetic import World

ORIGIN = Pose(0.0, 0.0, 0.0)


def empty_scan(n=360):
    return make_scan(np.full(n, np.inf), -math.pi, 2 * math.pi / n, 0.05, 10.0)


def test_scan_points_filters_invalid():
    r = np.full(360, np.inf)
    r[180] = 2.0
    r[90] = np.nan
    r[270] = 0.01
    r[0] = 10.0
    r[45] = 3.0
    xs, ys = scan_points(make_scan(r, -math.pi, 2 * math.pi / 360, 0.05, 10.0))
    assert xs.size == 2
    pts = sorted(zip(xs.round(6), ys.round(6)))
    a45 = -math.pi + 45 * 2 * math.pi / 360
    assert pts[0] == pytest.approx((3 * math.cos(a45), 3 * math.sin(a45)), abs=1e-6)
    assert pts[1] == pytest.approx((2.0, 0.0), abs=1e-6)


def test_corridor_free_distance_wall_ahead():
    world = World(boxes=[(2.0, -1.0, 2.2, 1.0)])
    xs, ys = scan_points(world.raycast(ORIGIN))
    free = corridor_free_distance(xs, ys, [0.0, math.pi / 2, math.pi], 0.3, 5.0)
    assert free[0] == pytest.approx(2.0, abs=0.02)
    assert free[1] == pytest.approx(5.0)
    assert free[2] == pytest.approx(5.0)


def test_corridor_ignores_points_behind():
    world = World(circles=[(-1.0, 0.0, 0.2)])
    xs, ys = scan_points(world.raycast(ORIGIN))
    assert corridor_free_distance(xs, ys, 0.0, 0.3, 5.0)[0] == pytest.approx(5.0)


def test_corridor_width_decides_gap():
    world = World(circles=[(2.0, 0.45, 0.1), (2.0, -0.45, 0.1)])
    xs, ys = scan_points(world.raycast(ORIGIN, n=720))
    assert corridor_free_distance(xs, ys, 0.0, 0.25, 5.0)[0] == pytest.approx(5.0)
    assert corridor_free_distance(xs, ys, 0.0, 0.40, 5.0)[0] < 2.1


def test_corridor_empty_scan():
    xs, ys = scan_points(empty_scan())
    assert corridor_free_distance(xs, ys, [0.0, 1.0], 0.3, 4.0).tolist() == [4.0, 4.0]


def test_time_window_mean():
    m = TimeWindowMean(0.3)
    assert m.update(0.0, 1.0) == pytest.approx(1.0)
    assert m.update(0.1, 3.0) == pytest.approx(2.0)
    assert m.update(0.2, 5.0) == pytest.approx(3.0)
    assert m.update(0.5, 7.0) == pytest.approx(6.0)
    m.reset()
    assert m.update(1.0, 0.5) == pytest.approx(0.5)
