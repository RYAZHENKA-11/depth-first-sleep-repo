from collections import deque
from typing import NamedTuple, Sequence, Tuple

import numpy as np


class Scan(NamedTuple):
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


def make_scan(ranges: Sequence[float], angle_min: float, angle_increment: float,
              range_min: float, range_max: float) -> Scan:
    return Scan(np.asarray(ranges, dtype=float), float(angle_min), float(angle_increment),
                float(range_min), float(range_max))


def scan_angles(scan: Scan) -> np.ndarray:
    return scan.angle_min + scan.angle_increment * np.arange(scan.ranges.size)


def scan_points(scan: Scan) -> Tuple[np.ndarray, np.ndarray]:
    r = scan.ranges
    valid = np.isfinite(r) & (r >= scan.range_min) & (r < scan.range_max)
    a = scan_angles(scan)[valid]
    r = r[valid]
    return r * np.cos(a), r * np.sin(a)


def corridor_free_distance(xs: np.ndarray, ys: np.ndarray, bearings, half_width: float,
                           max_dist: float) -> np.ndarray:
    b = np.atleast_1d(np.asarray(bearings, dtype=float))
    if xs.size == 0:
        return np.full(b.shape, float(max_dist))
    c = np.cos(b)[:, None]
    s = np.sin(b)[:, None]
    u = c * xs[None, :] + s * ys[None, :]
    v = -s * xs[None, :] + c * ys[None, :]
    blocking = (u > 0.0) & (np.abs(v) < half_width)
    u = np.where(blocking, u, np.inf)
    return np.minimum(u.min(axis=1), max_dist)


class TimeWindowMean:

    def __init__(self, window_s: float):
        self.window_s = window_s
        self._buf = deque()

    def reset(self):
        self._buf.clear()

    def update(self, t: float, value: float) -> float:
        self._buf.append((t, value))
        while len(self._buf) > 1 and t - self._buf[0][0] > self.window_s:
            self._buf.popleft()
        return sum(v for _, v in self._buf) / len(self._buf)
