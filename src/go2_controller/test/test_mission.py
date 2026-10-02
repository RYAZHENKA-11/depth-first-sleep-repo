import math

import pytest

from go2_controller.core.geometry import CourseFrame, Pose
from go2_controller.core.mission import (
    ABORT, ARRIVED, AVOID, IDLE, INIT, OUTBOUND, RECOVERY, RETURN, TURNAROUND, Mission,
    MissionParams)
from synthetic import World, footprint_clearance, run_mission

START = Pose(0.0, 0.0, 0.0)


def states(log):
    seq = []
    for _, _, out in log:
        if not seq or seq[-1] != out.state:
            seq.append(out.state)
    return seq


def min_clearance(world, log):
    return min(footprint_clearance(world, pose) for _, pose, _ in log)


def max_progress(log, start=START):
    course = CourseFrame(start)
    return max(course.to_course(p.x, p.y)[0] for _, p, _ in log)


def assert_success(world, log, start=START, obstacle_far_s=None):
    mission_out = log[-1][2]
    assert mission_out.state == ARRIVED, (states(log), mission_out.info)
    end = log[-1][1]
    assert math.hypot(end.x - start.x, end.y - start.y) < 0.3
    assert min_clearance(world, log) > 0.05
    if obstacle_far_s is not None:
        assert max_progress(log, start) > obstacle_far_s + 0.35


def test_idle_until_started():
    mission = Mission()
    out = mission.step(0.0, START, World().raycast(START))
    assert out.state == IDLE and out.vx == 0.0 and out.wz == 0.0


@pytest.mark.parametrize("world, far_s", [
    (World(circles=[(3.0, 0.0, 0.3)]), 3.3),
    (World(circles=[(3.5, 0.2, 0.4)]), 3.9),
    (World(boxes=[(2.5, -0.3, 3.3, 0.3)]), 3.3),
    (World(boxes=[(3.0, -0.6, 3.4, 0.1)]), 3.4),
])
def test_pass_obstacle_and_return(world, far_s):
    _, log = run_mission(world, Mission(), START)
    seq = states(log)
    assert seq[:4] == [INIT, OUTBOUND, AVOID, TURNAROUND]
    assert RETURN in seq and seq[-1] == ARRIVED
    assert_success(world, log, obstacle_far_s=far_s)


def test_rotated_start_frame():
    start = Pose(2.0, -1.0, 1.0)
    course = CourseFrame(start)
    cx, cy = course.to_world(3.0, 0.0)
    world = World(circles=[(cx, cy, 0.35)])
    _, log = run_mission(world, Mission(), start)
    assert_success(world, log, start, obstacle_far_s=3.35)


def test_course_with_side_walls():
    world = World(circles=[(3.0, 0.0, 0.3)],
                  boxes=[(-1.0, 1.2, 8.0, 1.4), (-1.0, -1.4, 8.0, -1.2)])
    _, log = run_mission(world, Mission(), START)
    assert_success(world, log, obstacle_far_s=3.3)


def test_no_obstacle_turns_at_max_distance():
    params = MissionParams(outbound_max_dist=4.0)
    _, log = run_mission(World(), Mission(params), START)
    assert states(log) == [INIT, OUTBOUND, TURNAROUND, RETURN, ARRIVED]
    assert max_progress(log) == pytest.approx(3.7, abs=0.2)
    assert log[-1][2].info["reason"] == "finish reached"


def test_obstacle_beyond_detect_band_is_ignored():
    world = World(circles=[(3.0, 1.5, 0.3)])
    params = MissionParams(outbound_max_dist=5.0)
    _, log = run_mission(world, Mission(params), START)
    assert AVOID not in states(log)
    assert log[-1][2].state == ARRIVED


def test_wall_across_course_aborts_safely():
    world = World(boxes=[(2.0, -6.0, 2.2, 6.0)])
    mission = Mission(MissionParams(mission_timeout_s=150.0))
    _, log = run_mission(world, mission, START)
    seq = states(log)
    assert RECOVERY in seq
    assert log[-1][2].state == ABORT
    assert min_clearance(world, log) > 0.05


def test_timeout_aborts():
    params = MissionParams(mission_timeout_s=5.0)
    _, log = run_mission(World(), Mission(params), START)
    assert log[-1][2].state == ABORT
    assert log[-1][2].info["reason"] == "mission timeout"
    assert log[-1][0] == pytest.approx(5.05, abs=0.06)


def test_stop_request_aborts_and_zeroes():
    mission = Mission()
    world = World()
    mission.start()
    for k in range(40):
        mission.step(k * 0.05, START, world.raycast(START))
    mission.stop("operator")
    out = mission.step(2.05, START, world.raycast(START))
    assert out.state == ABORT and out.vx == 0.0 and out.wz == 0.0
    assert out.info["reason"] == "operator"


def test_restart_after_finish():
    mission = Mission(MissionParams(outbound_max_dist=2.0))
    world = World()
    _, log = run_mission(world, mission, START)
    assert mission.state == ARRIVED
    mission.start()
    out = mission.step(100.0, log[-1][1], world.raycast(log[-1][1]))
    assert out.state == INIT
