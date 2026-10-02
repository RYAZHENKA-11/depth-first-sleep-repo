"""Реактивный контроллер без ROS2: страховка при отказе Nav2 и рулевой на время
bringup. Здесь же obstacle_scan() — общий для решения разбор облака лидара.
"""
import math
import os
from collections import deque

import numpy as np

from nav.perception import detect

_DEBUG = os.environ.get("GO2_DEBUG") == "1"
_debug_tick = 0
_last_tick_t = None
_dead_end_score = 0.0
_backup_until = None
_backup_streak = 0
_hard_stopped = False
_dead_end_dir = 0.0
_blind_score = 0.0
_front_min_hist = deque()

BACKUP_AFTER_S = 3.0
BACKUP_DECAY_RATE = 0.4
BACKUP_DURATION_S = 1.2
BACKUP_VX = -0.20
MAX_BACKUP_STREAK = 3
DEAD_END_VYAW = 0.5

BLIND_LIMIT_S = 4.0
BLIND_SEARCH_VYAW = 0.35

CRUISE_VX = 0.40
SLOW_VX = 0.20
NARROW_DIST = 1.0
STOP_DIST = 0.45
MAX_VYAW = 1.0

# лидар вынесен вперёд от центра; без этого корма попадает в скан как препятствие
LIDAR_X = 0.2
LIDAR_Z = 0.16
ROBOT_HALF_L = 0.5
ROBOT_HALF_W = 0.3
LIDAR_MIN_H = 0.25

FRONT_HALF_ANGLE = math.radians(75)
GAP_WINDOW = 5
FRONT_MIN_WINDOW_S = 0.3
FRONT_MIN_CLEAR = 1.5



_LIDAR_GEOM_CACHE = {}


def _unit_vectors(h_fov, h_res, layers, v_fov):
    """Единичные направления всех лучей купола в системе ЛИДАРА. Зависят только от
    геометрии сенсора, поэтому считаются один раз."""
    key = (h_fov, h_res, layers, v_fov)
    cached = _LIDAR_GEOM_CACHE.get(key)
    if cached is not None:
        return cached
    azimuth, elevation = np.meshgrid(
        np.linspace(h_fov * -0.5, h_fov * 0.5, h_res),
        np.linspace(v_fov * 0.5, v_fov * -0.5, layers),
    )
    cos_e = np.cos(elevation)
    _LIDAR_GEOM_CACHE[key] = (cos_e * np.cos(azimuth), -cos_e * np.sin(azimuth),
                              np.sin(elevation))
    return _LIDAR_GEOM_CACHE[key]


def obstacle_scan(robot):
    """Дальности до ближайшего препятствия по каждому азимуту; max_range = чисто."""
    h_fov, h_res, max_range, layers, v_fov = robot.lidar_info()
    cloud = robot.lidar()
    if not cloud or h_res <= 0 or layers <= 0:
        return []
    ranges = np.asarray(cloud, dtype=float).reshape(layers, h_res)
    dx, dy, dz = _unit_vectors(h_fov, h_res, layers, v_fov)

    roll, pitch, _yaw = robot.imu()
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    t = sr * dy + cr * dz
    ux = cp * dx + sp * t
    uy = cr * dy - sr * dz
    uz = -sp * dx + cp * t

    valid = np.isfinite(ranges)
    ranges = np.where(valid, ranges, 0.0)

    lidar_h = robot.position()[2] + (-sp * LIDAR_X + cp * cr * LIDAR_Z)
    valid &= lidar_h + ranges * uz >= LIDAR_MIN_H
    ranges, ux, uy = ranges[valid], ux[valid], uy[valid]

    x = LIDAR_X + ranges * ux
    y = ranges * uy
    outside = (np.abs(x) >= ROBOT_HALF_L) | (np.abs(y) >= ROBOT_HALF_W)
    x = x[outside] - LIDAR_X
    y = y[outside]

    bearing = (np.arctan2(y, x) + math.pi) % (2.0 * math.pi) - math.pi
    angle_inc = h_fov / max(h_res - 1, 1)
    idx = np.clip(np.rint((bearing + h_fov * 0.5) / angle_inc).astype(np.int64),
                  0, h_res - 1)

    out = np.full(h_res, float(max_range))
    np.minimum.at(out, idx, np.hypot(x, y))
    return out.tolist()


def _front_indices(h_fov, n):
    if n <= 1:
        return list(range(n))
    idxs = []
    for i in range(n):
        bearing = -h_fov / 2 + i * h_fov / (n - 1)
        if abs(bearing) <= FRONT_HALF_ANGLE:
            idxs.append(i)
    if not idxs:
        idxs = list(range(n))
    idxs.sort(key=lambda i: abs(-h_fov / 2 + i * h_fov / (n - 1)))
    return idxs


def _widest_gap_bearing(scan, h_fov, indices, window=GAP_WINDOW):
    """Азимут самого открытого сектора. Дальность берётся как минимум по окну лучей:
    иначе далёкая точка в щели у стены выглядит проходом.
    """
    n = len(scan)
    if n == 0 or not indices:
        return 0.0, 0.0

    def val(i):
        return scan[i]

    best_i, best_r = indices[0], -1.0
    for i in indices:
        lo, hi = max(0, i - window), min(n - 1, i + window)
        w = min(val(k) for k in range(lo, hi + 1))
        if w > best_r:
            best_r, best_i = w, i
    bearing = -h_fov / 2 + best_i * h_fov / (n - 1)
    return bearing, scan[best_i]


def _dbg(robot, **kv):
    global _debug_tick
    if not _DEBUG:
        return
    _debug_tick += 1
    if _debug_tick % 60 != 0:
        return
    parts = " ".join(f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}" for k, v in kv.items())
    print(f"[fallback] t={robot.time():.1f} {parts}", flush=True)


def tick(robot, stage_color):
    """Один шаг управления. stage_color — цвет маркера, который ищем сейчас."""
    global _last_tick_t, _dead_end_score, _backup_until, _backup_streak, _blind_score, \
        _hard_stopped, _dead_end_dir
    scan = obstacle_scan(robot)
    h_fov, h_res, max_range, _layers, _v_fov = robot.lidar_info()
    if not scan or h_res <= 1:
        robot.drive(vx=0.0, vyaw=0.0)
        return

    now = robot.time()
    dt = 0.0 if _last_tick_t is None else max(0.0, min(1.0, now - _last_tick_t))
    _last_tick_t = now

    if _hard_stopped:
        _dbg(robot, mode="HARD_STOP")
        robot.stand()  # не drive(0,0): тот гоняет походку на месте
        return

    if _backup_until is not None:
        if now < _backup_until:
            _dbg(robot, mode="BACKUP", vx=BACKUP_VX)
            robot.drive(vx=BACKUP_VX, vyaw=0.0)
            return
        _backup_until = None

    front_idx = _front_indices(h_fov, h_res)
    front_min_raw = min((scan[i] for i in front_idx), default=float(max_range))

    raw_for_hist = min(front_min_raw, FRONT_MIN_CLEAR)
    _front_min_hist.append((now, raw_for_hist))
    while len(_front_min_hist) > 1 and now - _front_min_hist[0][0] > FRONT_MIN_WINDOW_S:
        _front_min_hist.popleft()
    front_min = sum(v for _, v in _front_min_hist) / len(_front_min_hist)

    if front_min < STOP_DIST:
        if _dead_end_score == 0.0:
            gap_bearing, _ = _widest_gap_bearing(scan, h_fov, list(range(h_res)))
            _dead_end_dir = DEAD_END_VYAW if gap_bearing >= 0 else -DEAD_END_VYAW
        _dead_end_score = min(BACKUP_AFTER_S, _dead_end_score + dt)
        if _dead_end_score >= BACKUP_AFTER_S:
            _dead_end_score = 0.0
            _backup_streak += 1
            if _backup_streak >= MAX_BACKUP_STREAK:
                _hard_stopped = True
                _dbg(robot, mode="HARD_STOP_START", front_min=front_min, streak=_backup_streak)
                robot.stand()
                return
            _backup_until = now + BACKUP_DURATION_S
            _dbg(robot, mode="BACKUP_START", front_min=front_min, streak=_backup_streak)
            robot.drive(vx=BACKUP_VX, vyaw=0.0)
            return

        _dbg(robot, mode="DEAD_END", front_min=front_min, vyaw=_dead_end_dir, score=_dead_end_score)
        robot.drive(vx=0.0, vyaw=_dead_end_dir)
        return

    _dead_end_score = max(0.0, _dead_end_score - dt * BACKUP_DECAY_RATE)
    _backup_streak = 0

    gap_bearing, _gap_dist = _widest_gap_bearing(scan, h_fov, front_idx)

    found, marker_bearing, _area = detect(robot, stage_color)

    if not found and front_min_raw >= max_range - 1e-6:
        _blind_score = min(BLIND_LIMIT_S, _blind_score + dt)
    else:
        _blind_score = max(0.0, _blind_score - dt * BACKUP_DECAY_RATE)

    if _blind_score >= BLIND_LIMIT_S:
        _dbg(robot, mode="BLIND_SEARCH", blind_score=_blind_score)
        robot.drive(vx=0.0, vyaw=BLIND_SEARCH_VYAW)
        return

    target_bearing = marker_bearing if found else gap_bearing

    idx = int(round((target_bearing + h_fov / 2) / h_fov * (h_res - 1)))
    idx = max(0, min(h_res - 1, idx))
    dist_ahead = scan[idx]

    if dist_ahead < NARROW_DIST:
        target_bearing = gap_bearing

    vx = CRUISE_VX
    if front_min < NARROW_DIST:
        vx = SLOW_VX

    if abs(target_bearing) > FRONT_HALF_ANGLE:
        vx = 0.0

    vyaw = max(-MAX_VYAW, min(MAX_VYAW, 1.4 * target_bearing))
    _dbg(robot, mode=("MARKER" if found else "GAP"), front_min=front_min,
         target_bearing=target_bearing, vx=vx, vyaw=vyaw)
    robot.drive(vx=vx, vyaw=vyaw)
