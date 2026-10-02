import math
from typing import NamedTuple

import numpy as np

GO2_HALF_LENGTH = 0.35
GO2_HALF_WIDTH = 0.155


class SimPose(NamedTuple):
    x: float
    y: float
    yaw: float


class RayScan(NamedTuple):
    ranges: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float


DEFAULT_OBSTACLE_HEIGHT = 0.8


class World:

    def __init__(self, circles=(), boxes=(), start=(0.0, 0.0, 0.0),
                 default_height: float = DEFAULT_OBSTACLE_HEIGHT):
        self.circles = [tuple(map(float, c[:3])) for c in circles]
        self.boxes = [tuple(map(float, b[:4])) for b in boxes]
        self.circle_heights = [float(c[3]) if len(c) > 3 else default_height for c in circles]
        self.box_heights = [float(b[4]) if len(b) > 4 else default_height for b in boxes]
        self.start = SimPose(*map(float, start))

    @classmethod
    def from_dict(cls, data):
        return cls(circles=data.get("circles") or (), boxes=data.get("boxes") or (),
                   start=data.get("start") or (0.0, 0.0, 0.0))

    def inside_obstacle(self, x, y):
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        inside = np.zeros(np.broadcast(x, y).shape, dtype=bool)
        for cx, cy, r in self.circles:
            inside |= np.hypot(x - cx, y - cy) <= r
        for x0, y0, x1, y1 in self.boxes:
            inside |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
        return inside

    def raycast(self, pose, n: int = 360, range_max: float = 10.0,
                range_min: float = 0.05) -> RayScan:
        inc = 2.0 * math.pi / n
        angles = -math.pi + inc * np.arange(n)
        world_angles = pose.yaw + angles
        dx, dy = np.cos(world_angles), np.sin(world_angles)
        hit = np.full(n, np.inf)
        for cx, cy, r in self.circles:
            hit = np.minimum(hit, _ray_circle(pose.x, pose.y, dx, dy, cx, cy, r))
        for box in self.boxes:
            hit = np.minimum(hit, _ray_box(pose.x, pose.y, dx, dy, *box))
        hit = np.where(hit < range_max, hit, np.inf)
        return RayScan(hit, -math.pi, inc, range_min, range_max)

    def min_clearance(self, x: float, y: float) -> float:
        best = math.inf
        for cx, cy, r in self.circles:
            best = min(best, math.hypot(x - cx, y - cy) - r)
        for x0, y0, x1, y1 in self.boxes:
            ddx = max(x0 - x, 0.0, x - x1)
            ddy = max(y0 - y, 0.0, y - y1)
            best = min(best, math.hypot(ddx, ddy))
        return best


def voxel_cloud(world: World, center_x: float, center_y: float, half_size: float = 3.2,
                height: float = 1.9, res: float = 0.05, floor: bool = True) -> np.ndarray:
    def grid(lo, hi):
        return (np.arange(math.floor(lo / res), math.ceil(hi / res)) + 0.5) * res

    xs = grid(center_x - half_size, center_x + half_size)
    ys = grid(center_y - half_size, center_y + half_size)
    zs = grid(0.0, height)
    chunks = []

    if floor:
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        gx, gy = gx.ravel(), gy.ravel()
        keep = ~world.inside_obstacle(gx, gy)
        chunks.append(np.column_stack([gx[keep], gy[keep], np.zeros(int(keep.sum()))]))

    walls = []
    for (cx, cy, r), h in zip(world.circles, world.circle_heights):
        n = max(8, int(math.ceil(2.0 * math.pi * r / (0.5 * res))))
        a = 2.0 * math.pi * np.arange(n) / n
        walls.append((cx + r * np.cos(a), cy + r * np.sin(a), h))
    for (x0, y0, x1, y1), h in zip(world.boxes, world.box_heights):
        step = 0.5 * res
        tx = np.arange(x0, x1 + step, step)
        ty = np.arange(y0, y1 + step, step)
        px = np.concatenate([tx, tx, np.full(ty.size, x0), np.full(ty.size, x1)])
        py = np.concatenate([np.full(tx.size, y0), np.full(tx.size, y1), ty, ty])
        walls.append((px, py, h))

    for px, py, h in walls:
        cols = np.unique(np.column_stack([np.floor(px / res), np.floor(py / res)]), axis=0)
        wx = (cols[:, 0] + 0.5) * res
        wy = (cols[:, 1] + 0.5) * res
        inside = (np.abs(wx - center_x) <= half_size) & (np.abs(wy - center_y) <= half_size)
        wx, wy = wx[inside], wy[inside]
        wz = zs[zs <= h]
        if wx.size and wz.size:
            chunks.append(np.column_stack([np.repeat(wx, wz.size), np.repeat(wy, wz.size),
                                           np.tile(wz, wx.size)]))

    if not chunks:
        return np.zeros((0, 3), dtype=np.float32)
    return np.ascontiguousarray(np.vstack(chunks), dtype=np.float32)


class DriftingOdometry:

    def __init__(self, start, scale_error: float = 0.0, yaw_bias: float = 0.0):
        self.pose = SimPose(*start)
        self.scale_error = scale_error
        self.yaw_bias = yaw_bias

    def update(self, vx: float, wz: float, dt: float) -> SimPose:
        v = vx * (1.0 + self.scale_error)
        w = wz + self.yaw_bias
        p = self.pose
        mid = p.yaw + 0.5 * w * dt
        yaw = p.yaw + w * dt
        self.pose = SimPose(p.x + v * dt * math.cos(mid), p.y + v * dt * math.sin(mid),
                            math.atan2(math.sin(yaw), math.cos(yaw)))
        return self.pose


def footprint_clearance(world: World, pose, half_length: float = GO2_HALF_LENGTH,
                        half_width: float = GO2_HALF_WIDTH, samples: int = 8) -> float:
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    best = math.inf
    for k in range(samples + 1):
        f = -1.0 + 2.0 * k / samples
        for lx, ly in ((f * half_length, half_width), (f * half_length, -half_width),
                       (half_length, f * half_width), (-half_length, f * half_width)):
            wx = pose.x + c * lx - s * ly
            wy = pose.y + s * lx + c * ly
            best = min(best, world.min_clearance(wx, wy))
    return best


def _ray_circle(ox, oy, dx, dy, cx, cy, r):
    fx, fy = ox - cx, oy - cy
    b = fx * dx + fy * dy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    ok = disc >= 0.0
    t = -b - np.sqrt(np.where(ok, disc, 0.0))
    return np.where(ok & (t > 0.0), t, np.inf)


def _slab(o, d, lo, hi):
    inside = lo <= o <= hi
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (lo - o) / d
        t2 = (hi - o) / d
    parallel = d == 0.0
    t_min = np.where(parallel, -np.inf if inside else np.inf, np.minimum(t1, t2))
    t_max = np.where(parallel, np.inf if inside else -np.inf, np.maximum(t1, t2))
    return t_min, t_max


def _ray_box(ox, oy, dx, dy, x0, y0, x1, y1):
    tx_min, tx_max = _slab(ox, dx, x0, x1)
    ty_min, ty_max = _slab(oy, dy, y0, y1)
    t_near = np.maximum(tx_min, ty_min)
    t_far = np.minimum(tx_max, ty_max)
    return np.where((t_far >= t_near) & (t_near > 0.0), t_near, np.inf)


class KinematicRobot:

    def __init__(self, pose, tau: float = 0.25, max_vx: float = 0.6, max_wz: float = 1.0):
        self.pose = SimPose(*pose)
        self.tau = tau
        self.max_vx = max_vx
        self.max_wz = max_wz
        self.vx = 0.0
        self.wz = 0.0

    def step(self, cmd_vx: float, cmd_wz: float, dt: float) -> SimPose:
        cmd_vx = max(-self.max_vx, min(self.max_vx, cmd_vx))
        cmd_wz = max(-self.max_wz, min(self.max_wz, cmd_wz))
        k = dt / (self.tau + dt)
        self.vx += k * (cmd_vx - self.vx)
        self.wz += k * (cmd_wz - self.wz)
        p = self.pose
        mid_yaw = p.yaw + 0.5 * self.wz * dt
        yaw = p.yaw + self.wz * dt
        self.pose = SimPose(p.x + self.vx * dt * math.cos(mid_yaw),
                            p.y + self.vx * dt * math.sin(mid_yaw),
                            math.atan2(math.sin(yaw), math.cos(yaw)))
        return self.pose
