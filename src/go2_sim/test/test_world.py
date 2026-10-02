import math

import numpy as np
import pytest

from go2_sim.world import DriftingOdometry, KinematicRobot, SimPose, World, voxel_cloud

RES = 0.05
ORIGIN = SimPose(0.0, 0.0, 0.0)


def ray(scan, bearing):
    idx = int(round((bearing - scan.angle_min) / scan.angle_increment)) % scan.ranges.size
    return scan.ranges[idx]


def test_raycast_circle_and_box():
    world = World(circles=[(3.0, 0.0, 0.5)], boxes=[(-1.0, 1.0, 1.0, 1.5)])
    scan = world.raycast(ORIGIN)
    assert ray(scan, 0.0) == pytest.approx(2.5, abs=1e-9)
    assert ray(scan, math.pi / 2) == pytest.approx(1.0, abs=1e-9)
    assert math.isinf(ray(scan, -math.pi / 2))


def test_raycast_is_in_robot_frame():
    world = World(circles=[(0.0, 3.0, 0.5)])
    assert ray(world.raycast(SimPose(0.0, 0.0, math.pi / 2)), 0.0) == pytest.approx(2.5, abs=1e-9)


def test_raycast_range_max():
    world = World(circles=[(20.0, 0.0, 0.5)])
    assert np.isinf(world.raycast(ORIGIN, range_max=10.0).ranges).all()


def test_min_clearance():
    world = World(circles=[(3.0, 0.0, 0.5)], boxes=[(0.0, 2.0, 1.0, 3.0)])
    assert world.min_clearance(0.0, 0.0) == pytest.approx(2.0)
    assert world.min_clearance(2.0, 0.0) == pytest.approx(0.5)


def test_kinematic_robot_straight_and_turn():
    robot = KinematicRobot(ORIGIN)
    for _ in range(200):
        robot.step(0.5, 0.0, 0.05)
    assert robot.pose.x == pytest.approx(0.5 * 10.0 - 0.5 * 0.25, abs=0.05)
    assert robot.pose.y == pytest.approx(0.0, abs=1e-9)
    robot = KinematicRobot(ORIGIN)
    for _ in range(100):
        robot.step(0.0, 5.0, 0.05)
    assert robot.wz == pytest.approx(1.0, abs=1e-3)


def test_heights_parsed_with_default():
    world = World.from_dict({"circles": [[1, 0, 0.3], [2, 0, 0.2, 0.1]],
                             "boxes": [[0, 0, 1, 1], [3, 3, 4, 4, 1.5]]})
    assert world.circles == [(1.0, 0.0, 0.3), (2.0, 0.0, 0.2)]
    assert world.circle_heights == [0.8, 0.1]
    assert world.box_heights == [0.8, 1.5]


def test_inside_obstacle():
    world = World(circles=[(1.0, 0.0, 0.3)], boxes=[(3.0, -1.0, 4.0, 1.0)])
    assert world.inside_obstacle([1.0, 1.5, 3.5, 2.9], [0.0, 0.0, 0.5, 0.0]).tolist() == \
        [True, False, True, False]


def test_cloud_empty_world_is_floor_only():
    pts = voxel_cloud(World(), 0.0, 0.0, half_size=1.0)
    assert pts.dtype == np.float32 and pts.shape[1] == 3
    assert np.all(pts[:, 2] == 0.0)
    assert pts.shape[0] == 40 * 40
    assert np.all(np.abs(pts[:, :2]) <= 1.0 + RES)


def test_cloud_without_floor():
    assert voxel_cloud(World(), 0.0, 0.0, floor=False).shape == (0, 3)


def test_cloud_is_on_voxel_grid():
    world = World(circles=[(1.0, 0.3, 0.25)], boxes=[(-2.0, -1.0, -1.5, 1.0)])
    pts = voxel_cloud(world, 0.1, -0.2).astype(float)
    frac = pts / RES - 0.5
    assert np.allclose(frac[:, :2], np.round(frac[:, :2]), atol=1e-4)
    assert np.unique(pts, axis=0).shape[0] == pts.shape[0]


def test_circle_surface_and_height():
    world = World(circles=[(1.5, 0.0, 0.3, 0.6)])
    pts = voxel_cloud(world, 0.0, 0.0).astype(float)
    wall = pts[pts[:, 2] > 0.0]
    r = np.hypot(wall[:, 0] - 1.5, wall[:, 1])
    assert wall.shape[0] > 0
    assert np.all(np.abs(r - 0.3) < RES)
    assert wall[:, 2].max() == pytest.approx(0.575, abs=1e-4)
    floor = pts[pts[:, 2] == 0.0]
    assert not world.inside_obstacle(floor[:, 0], floor[:, 1]).any()


def test_cloud_window_clips_far_obstacles():
    world = World(circles=[(10.0, 0.0, 0.3)])
    pts = voxel_cloud(world, 0.0, 0.0, half_size=3.2)
    assert np.all(pts[:, 2] == 0.0)


def test_drifting_odometry_scale_and_bias():
    exact = DriftingOdometry((0.0, 0.0, 0.0))
    drift = DriftingOdometry((0.0, 0.0, 0.0), scale_error=0.03, yaw_bias=0.0)
    for _ in range(200):
        exact.update(0.5, 0.0, 0.05)
        drift.update(0.5, 0.0, 0.05)
    assert exact.pose.x == pytest.approx(5.0)
    assert drift.pose.x == pytest.approx(5.15)

    yawing = DriftingOdometry((1.0, 2.0, 0.5), yaw_bias=0.01)
    for _ in range(100):
        yawing.update(0.0, 0.0, 0.1)
    assert yawing.pose.yaw == pytest.approx(0.6)
    assert (yawing.pose.x, yawing.pose.y) == (1.0, 2.0)


def test_drifting_odometry_matches_truth_without_errors():
    odom = DriftingOdometry((0.0, 0.0, 0.0))
    for _ in range(100):
        odom.update(0.4, 0.3, 0.05)
    r = 0.4 / 0.3
    yaw = 0.3 * 5.0
    assert odom.pose.yaw == pytest.approx(yaw)
    assert odom.pose.x == pytest.approx(r * math.sin(yaw), abs=1e-3)
    assert odom.pose.y == pytest.approx(r * (1 - math.cos(yaw)), abs=1e-3)
