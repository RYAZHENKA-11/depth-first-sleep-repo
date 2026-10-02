import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from go2_controller.core.geometry import Pose, bearing_to, distance
from go2_controller.core.scan_utils import Scan, TimeWindowMean, corridor_free_distance, scan_points

DRIVE = "DRIVE"
TURN = "TURN"
BLOCKED = "BLOCKED"


@dataclass
class PlannerParams:
    robot_half_length: float = 0.35
    robot_half_width: float = 0.16
    side_clearance: float = 0.15
    stop_clearance: float = 0.25
    slow_dist: float = 1.2
    cruise_vx: float = 0.35
    slow_vx: float = 0.15
    max_wz: float = 0.8
    k_yaw: float = 1.4
    turn_in_place_angle: float = math.radians(60.0)
    lookahead: float = 2.0
    detour_min_free: float = 1.0
    max_detour_angle: float = math.radians(110.0)
    bearing_step: float = math.radians(3.0)
    side_switch_penalty: float = math.radians(30.0)
    front_window_s: float = 0.3
    max_accel: float = 0.5
    max_decel: float = 1.5
    max_alpha: float = 2.0
    max_alpha_decel: float = 4.0
    turn_clearance: float = 0.08
    goal_slow_radius: float = 0.6
    min_goal_vx: float = 0.08

    @property
    def corridor_half_width(self) -> float:
        return self.robot_half_width + self.side_clearance

    @property
    def sweep_radius(self) -> float:
        return math.hypot(self.robot_half_length, self.robot_half_width)

    @property
    def stop_dist(self) -> float:
        return self.robot_half_length + self.stop_clearance


@dataclass
class PlannerOutput:
    vx: float
    wz: float
    status: str
    target_bearing: float
    front_free: float
    goal_dist: float
    nearest: float


class LocalPlanner:

    def __init__(self, params: Optional[PlannerParams] = None):
        self.p = params or PlannerParams()
        self._front = TimeWindowMean(self.p.front_window_s)
        self._side = 0
        self._last_t: Optional[float] = None
        self._vx = 0.0
        self._wz = 0.0

    def reset(self):
        self._front.reset()
        self._side = 0
        self._last_t = None
        self._vx = 0.0
        self._wz = 0.0

    @property
    def committed_side(self) -> int:
        return self._side

    def compute(self, t: float, pose: Pose, scan: Scan, goal: Tuple[float, float],
                side_hint: int = 0) -> PlannerOutput:
        p = self.p
        xs, ys = scan_points(scan)
        goal_dist = distance(pose, goal[0], goal[1])
        goal_bearing = bearing_to(pose, goal[0], goal[1])

        target, target_free = self._choose_direction(xs, ys, goal_bearing, goal_dist, side_hint)

        front_raw = float(
            corridor_free_distance(xs, ys, 0.0, p.corridor_half_width, p.slow_dist)[0])
        front = self._front.update(t, front_raw)
        nearest = float(np.hypot(xs, ys).min()) if xs.size else math.inf
        can_spin = nearest >= p.sweep_radius + p.turn_clearance

        wz = float(np.clip(p.k_yaw * target, -p.max_wz, p.max_wz))
        blocked = front_raw < p.stop_dist and target_free < p.stop_dist + 0.1
        if blocked:
            vx = 0.0
            status = BLOCKED
            if not can_spin:
                wz = 0.0
        elif abs(target) > p.turn_in_place_angle:
            vx = 0.0
            if can_spin:
                status = TURN
            else:
                status = BLOCKED
                wz = 0.0
        else:
            vx = self._speed_limit(front_raw, front, goal_dist)
            vx *= max(0.0, 1.0 - abs(target) / p.turn_in_place_angle)
            status = DRIVE

        vx, wz = self._rate_limit(t, vx, wz)
        return PlannerOutput(vx, wz, status, target, front_raw, goal_dist, nearest)

    def _choose_direction(self, xs, ys, goal_bearing, goal_dist, side_hint):
        p = self.p
        need = min(p.lookahead, goal_dist)
        w = p.corridor_half_width
        goal_free = float(corridor_free_distance(xs, ys, goal_bearing, w, p.lookahead)[0])
        if goal_free >= need - 1e-6:
            self._side = 0
            return goal_bearing, goal_free

        n = int(round(p.max_detour_angle / p.bearing_step))
        offsets = p.bearing_step * np.arange(-n, n + 1)
        candidates = np.arctan2(np.sin(goal_bearing + offsets), np.cos(goal_bearing + offsets))
        free = corridor_free_distance(xs, ys, candidates, w, p.lookahead)

        feasible = free >= min(p.detour_min_free, need) - 1e-6
        preferred = self._side or side_hint
        if feasible.any():
            cost = np.abs(offsets)
            if preferred:
                cost = cost + p.side_switch_penalty * (np.sign(offsets) == -preferred)
            cost = np.where(feasible, cost, np.inf)
            i = int(np.argmin(cost))
        else:
            best = free.max()
            ties = np.flatnonzero(free >= best - 1e-6)
            i = int(ties[np.argmin(np.abs(offsets[ties]))])

        if offsets[i] != 0.0:
            self._side = int(np.sign(offsets[i]))
        return float(candidates[i]), float(free[i])

    def _speed_limit(self, front_raw, front_smooth, goal_dist):
        p = self.p
        if front_raw < p.stop_dist:
            return 0.0
        free = min(front_raw, front_smooth)
        if free >= p.slow_dist:
            v = p.cruise_vx
        else:
            k = (free - p.stop_dist) / max(p.slow_dist - p.stop_dist, 1e-6)
            v = p.slow_vx + max(0.0, min(1.0, k)) * (p.cruise_vx - p.slow_vx)
        if goal_dist < p.goal_slow_radius:
            v = min(v, max(p.min_goal_vx, p.cruise_vx * goal_dist / p.goal_slow_radius))
        return v

    def _rate_limit(self, t, vx, wz):
        p = self.p
        dt = 0.0 if self._last_t is None else max(0.0, min(0.5, t - self._last_t))
        self._last_t = t
        lim = (p.max_accel if abs(vx) > abs(self._vx) else p.max_decel) * dt
        vx = self._vx + float(np.clip(vx - self._vx, -lim, lim))
        wl = (p.max_alpha if abs(wz) > abs(self._wz) else p.max_alpha_decel) * dt
        wz = self._wz + float(np.clip(wz - self._wz, -wl, wl))
        self._vx, self._wz = vx, wz
        return vx, wz
