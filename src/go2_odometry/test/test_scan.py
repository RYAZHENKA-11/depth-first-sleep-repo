import math

import numpy as np
import pytest

from go2_odometry.scan import cloud_to_scan, quaternion_to_yaw, transform_points_to_base


def test_quaternion_to_yaw_normalizes_and_rejects_invalid_values():
    yaw = 0.7
    assert quaternion_to_yaw(0.0, 0.0, 2.0 * math.sin(yaw / 2),
                             2.0 * math.cos(yaw / 2)) == pytest.approx(yaw)
    with pytest.raises(ValueError):
        quaternion_to_yaw(0.0, 0.0, 0.0, 0.0)


def test_transform_points_to_base_inverts_robot_pose():
    points = np.array([[2.0, 3.0, 0.4]])
    transformed = transform_points_to_base(points, (2.0, 2.0, 0.1), math.pi / 2)
    assert transformed[0] == pytest.approx((1.0, 0.0, 0.3))


def test_transform_points_to_base_uses_full_quaternion():
    half_angle = math.pi / 4
    orientation = (math.sin(half_angle), 0.0, 0.0, math.cos(half_angle))
    transformed = transform_points_to_base(
        [[0.0, 0.0, 1.0]], (0, 0, 0), orientation)
    assert transformed[0] == pytest.approx((0.0, 1.0, 0.0))


def test_cloud_to_scan_uses_robot_center_and_keeps_nearest_return():
    points = np.array([
        [1.0, 0.0, 0.3],
        [2.0, 0.0, 0.3],
        [0.0, 1.0, 0.3],
    ])
    ranges = cloud_to_scan(points, (0.0, 0.0, 0.0), 0.0)
    assert ranges[180] == pytest.approx(1.0)
    assert ranges[270] == pytest.approx(1.0)
    assert np.isinf(ranges[0])


def test_cloud_to_scan_filters_floor_body_invalid_and_out_of_range_points():
    points = np.array([
        [1.0, 0.0, 0.0],       # floor
        [0.2, 0.0, 0.3],       # robot body
        [1.0, 0.0, 0.3],       # valid
        [1.0, 0.0, np.nan],    # invalid
        [11.0, 0.0, 0.3],      # beyond range_max
    ])
    ranges = cloud_to_scan(points, (0.0, 0.0, 0.0), 0.0)
    assert ranges[180] == pytest.approx(1.0)


def test_cloud_to_scan_handles_empty_and_invalid_input():
    assert np.all(np.isinf(cloud_to_scan(np.empty((0, 3)), (0, 0, 0), 0)))
    with pytest.raises(ValueError):
        cloud_to_scan(np.zeros((2, 2)), (0, 0, 0), 0)