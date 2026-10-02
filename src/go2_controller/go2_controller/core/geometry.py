import math
from typing import NamedTuple, Tuple


class Pose(NamedTuple):
    x: float
    y: float
    yaw: float


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def distance(pose: Pose, gx: float, gy: float) -> float:
    return math.hypot(gx - pose.x, gy - pose.y)


def bearing_to(pose: Pose, gx: float, gy: float) -> float:
    return wrap(math.atan2(gy - pose.y, gx - pose.x) - pose.yaw)


def to_local(pose: Pose, wx: float, wy: float) -> Tuple[float, float]:
    dx, dy = wx - pose.x, wy - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return c * dx + s * dy, -s * dx + c * dy


def to_world(pose: Pose, lx: float, ly: float) -> Tuple[float, float]:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return pose.x + c * lx - s * ly, pose.y + s * lx + c * ly


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class CourseFrame:

    def __init__(self, origin: Pose):
        self.origin = origin

    def to_course(self, wx: float, wy: float) -> Tuple[float, float]:
        return to_local(self.origin, wx, wy)

    def to_world(self, s: float, lat: float) -> Tuple[float, float]:
        return to_world(self.origin, s, lat)
