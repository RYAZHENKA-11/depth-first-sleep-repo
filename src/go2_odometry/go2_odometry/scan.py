import math

import numpy as np


def quaternion_to_yaw(x, y, z, w):
    quaternion = np.asarray((x, y, z, w), dtype=float)
    if not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion must contain finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("quaternion must have non-zero length")
    x, y, z, w = quaternion / norm
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def transform_points_to_base(points, position, orientation):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError("points must have shape (N, 3) or wider")
    x, y, z = (float(value) for value in position)
    if isinstance(orientation, (int, float)):
        cosine = math.cos(orientation)
        sine = math.sin(orientation)
        rotation = np.array(((cosine, -sine, 0.0),
                             (sine, cosine, 0.0),
                             (0.0, 0.0, 1.0)))
    else:
        quaternion = np.asarray(orientation, dtype=float)
        if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
            raise ValueError("orientation must be a yaw or finite (x, y, z, w) quaternion")
        norm = float(np.linalg.norm(quaternion))
        if norm < 1e-12:
            raise ValueError("quaternion must have non-zero length")
        qx, qy, qz, qw = quaternion / norm
        rotation = np.array((
            (1.0 - 2.0 * (qy * qy + qz * qz),
             2.0 * (qx * qy - qz * qw),
             2.0 * (qx * qz + qy * qw)),
            (2.0 * (qx * qy + qz * qw),
             1.0 - 2.0 * (qx * qx + qz * qz),
             2.0 * (qy * qz - qx * qw)),
            (2.0 * (qx * qz - qy * qw),
             2.0 * (qy * qz + qx * qw),
             1.0 - 2.0 * (qx * qx + qy * qy)),
        ))
    return (points[:, :3] - np.array((x, y, z))) @ rotation


def cloud_to_scan(points, position, orientation, *, beam_count=360, range_min=0.05,
                  range_max=10.0, min_obstacle_height=0.08,
                  max_obstacle_height=1.5, body_half_length=0.40,
                  body_half_width=0.205, body_height=0.65):
    """Project odom-frame XYZ points into a robot-centered planar scan."""
    if beam_count < 1:
        raise ValueError("beam_count must be positive")
    if not (0.0 <= range_min < range_max):
        raise ValueError("scan ranges must satisfy 0 <= range_min < range_max")
    if not (0.0 <= min_obstacle_height < max_obstacle_height):
        raise ValueError("obstacle heights must satisfy 0 <= min < max")

    base_points = transform_points_to_base(points, position, orientation)
    x = base_points[:, 0]
    y = base_points[:, 1]
    z = base_points[:, 2]
    finite = np.all(np.isfinite(base_points), axis=1)
    above_ground = (z >= min_obstacle_height) & (z <= max_obstacle_height)
    inside_body = ((np.abs(x) <= body_half_length)
                   & (np.abs(y) <= body_half_width)
                   & (z <= body_height))

    ranges = np.full(beam_count, np.inf, dtype=np.float32)
    valid = finite & above_ground & ~inside_body
    distances = np.hypot(x, y)
    valid &= (distances >= range_min) & (distances < range_max)
    if np.any(valid):
        angle_increment = 2.0 * math.pi / beam_count
        angles = np.arctan2(y[valid], x[valid])
        indices = np.floor((angles + math.pi) / angle_increment).astype(int)
        indices %= beam_count
        np.minimum.at(ranges, indices, distances[valid].astype(np.float32))
    return ranges