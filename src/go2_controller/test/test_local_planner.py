import numpy as np
import pytest

from go2_controller.core.geometry import Pose
from go2_controller.core.local_planner import (
    BLOCKED, DRIVE, TURN, LocalPlanner, PlannerParams)
from synthetic import World, footprint_clearance, run_planner

START = Pose(0.0, 0.0, 0.0)
DT = 0.05


def min_clearance(world, log):
    return min(footprint_clearance(world, pose) for _, pose, _ in log)


def test_open_field_reaches_goal():
    world = World()
    robot, log = run_planner(world, LocalPlanner(), START, (5.0, 0.0))
    assert log[-1][2].goal_dist < 0.25
    assert abs(robot.pose.y) < 0.05
    assert all(o.status == DRIVE for _, _, o in log)


def test_limits_respected():
    p = PlannerParams()
    _, log = run_planner(World(), LocalPlanner(p), START, (3.0, 3.0))
    vx = np.array([o.vx for _, _, o in log])
    wz = np.array([o.wz for _, _, o in log])
    assert vx.max() <= p.cruise_vx + 1e-9
    assert vx.min() >= 0.0
    assert np.abs(wz).max() <= p.max_wz + 1e-9
    assert np.abs(np.diff(vx)).max() <= max(p.max_accel, p.max_decel) * DT + 1e-9
    assert np.abs(np.diff(wz)).max() <= max(p.max_alpha, p.max_alpha_decel) * DT + 1e-9


def test_first_command_is_zero():
    out = LocalPlanner().compute(0.0, START, World().raycast(START), (5.0, 0.0))
    assert out.vx == 0.0 and out.wz == 0.0


def test_goal_behind_turns_in_place_first():
    _, log = run_planner(World(), LocalPlanner(), START, (-3.0, 0.0), t_max=2.0)
    assert any(o.status == TURN for _, _, o in log)
    turn_phase = [o for _, _, o in log if o.status == TURN]
    assert max(o.vx for o in turn_phase) == pytest.approx(0.0, abs=0.05)


@pytest.mark.parametrize("obstacle", [
    ("circle", (3.0, 0.0, 0.3)),
    ("circle", (3.0, 0.15, 0.3)),
    ("box", (2.8, -0.4, 3.4, 0.4)),
    ("box", (2.5, -0.6, 3.0, 0.2)),
])
def test_avoids_single_obstacle(obstacle):
    kind, shape = obstacle
    world = World(circles=[shape]) if kind == "circle" else World(boxes=[shape])
    robot, log = run_planner(world, LocalPlanner(), START, (6.0, 0.0), t_max=60.0)
    assert log[-1][2].goal_dist < 0.25, f"не доехал: {robot.pose}"
    assert min_clearance(world, log) > 0.05


def test_symmetric_obstacle_no_dithering():
    world = World(circles=[(3.0, 0.0, 0.4)])
    _, log = run_planner(world, LocalPlanner(), START, (6.0, 0.0), t_max=60.0)
    assert log[-1][2].goal_dist < 0.25
    sides = [np.sign(pose.y) for _, pose, _ in log if 2.0 < pose.x < 4.0 and abs(pose.y) > 0.05]
    assert len(set(sides)) == 1


def test_side_hint_selects_side():
    world = World(circles=[(3.0, 0.0, 0.4)])
    for hint in (1, -1):
        _, log = run_planner(world, LocalPlanner(), START, (6.0, 0.0), side_hint=hint)
        y_at_obstacle = [pose.y for _, pose, _ in log if 2.8 < pose.x < 3.2]
        assert y_at_obstacle and np.sign(np.mean(y_at_obstacle)) == hint


def test_wall_without_gap_blocks_and_does_not_collide():
    world = World(boxes=[(2.0, -5.0, 2.2, 5.0)])
    _, log = run_planner(world, LocalPlanner(), START, (6.0, 0.0), t_max=20.0)
    assert log[-1][2].goal_dist > 3.0
    assert any(o.status == BLOCKED for _, _, o in log[-20:])
    assert min_clearance(world, log) > 0.1


def test_gap_narrower_than_robot_not_entered():
    world = World(boxes=[(2.0, -5.0, 2.2, -0.15), (2.0, 0.15, 2.2, 5.0)])
    robot, log = run_planner(world, LocalPlanner(), START, (6.0, 0.0), t_max=20.0)
    assert robot.pose.x < 2.0
    assert min_clearance(world, log) > 0.1


def test_gap_wide_enough_is_passed():
    world = World(boxes=[(2.0, -5.0, 2.2, -0.45), (2.0, 0.45, 2.2, 5.0)])
    _, log = run_planner(world, LocalPlanner(), START, (6.0, 0.0), t_max=40.0)
    assert log[-1][2].goal_dist < 0.25
    assert min_clearance(world, log) > 0.05


def test_slows_down_near_obstacle():
    p = PlannerParams()
    world = World(boxes=[(2.0, -5.0, 2.2, 5.0)])
    _, log = run_planner(world, LocalPlanner(p), START, (6.0, 0.0), t_max=20.0)
    near = [o.vx for _, pose, o in log if 2.0 - pose.x < p.slow_dist - 0.2 and o.status == DRIVE]
    assert near and max(near) < p.cruise_vx
