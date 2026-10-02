import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from go2_controller.core.geometry import CourseFrame, Pose, bearing_to, distance
from go2_controller.core.local_planner import BLOCKED, LocalPlanner, PlannerParams
from go2_controller.core.scan_utils import Scan, corridor_free_distance, scan_points

IDLE = "IDLE"
INIT = "INIT"
OUTBOUND = "OUTBOUND"
AVOID = "AVOID"
TURNAROUND = "TURNAROUND"
RETURN = "RETURN"
RECOVERY = "RECOVERY"
ARRIVED = "ARRIVED"
ABORT = "ABORT"

PROGRESS_STATES = (OUTBOUND, AVOID, RETURN)


@dataclass
class MissionParams:
    start_delay_s: float = 1.0
    detect_dist: float = 1.5
    detect_half_width: float = 0.5
    detect_min_points: int = 3
    outbound_max_dist: float = 8.0
    obstacle_min_depth: float = 0.3
    obstacle_max_depth: float = 1.5
    pass_margin: float = 0.8
    goal_tol: float = 0.3
    finish_tol: float = 0.3
    turn_tol: float = math.radians(20.0)
    blocked_timeout_s: float = 3.0
    progress_window_s: float = 8.0
    progress_min: float = 0.1
    backup_duration_s: float = 1.2
    backup_vx: float = -0.2
    rear_clearance: float = 0.3
    max_recoveries: int = 3
    recovery_reset_s: float = 10.0
    mission_timeout_s: float = 180.0
    side_hint: int = 0


@dataclass
class MissionOutput:
    vx: float
    wz: float
    state: str
    info: dict = field(default_factory=dict)


class Mission:

    def __init__(self, params: Optional[MissionParams] = None,
                 planner_params: Optional[PlannerParams] = None):
        self.p = params or MissionParams()
        self.planner = LocalPlanner(planner_params)
        self.state = IDLE
        self.reason = ""
        self._armed = False
        self._course: Optional[CourseFrame] = None
        self._t0 = 0.0
        self._state_t = 0.0
        self._s_obs: Optional[float] = None
        self._s_far: Optional[float] = None
        self._goal_s: Optional[float] = None
        self._side_hint = self.p.side_hint
        self._blocked_since: Optional[float] = None
        self._progress_ref: Optional[tuple] = None
        self._recoveries = 0
        self._last_recovery_t = -math.inf
        self._resume_state = OUTBOUND
        self._backup_until = 0.0
        self._last_planner = None
        self._last_goal = None

    @property
    def done(self) -> bool:
        return self.state in (ARRIVED, ABORT)

    def start(self):
        if self.state in (IDLE, ARRIVED, ABORT):
            self._armed = True
            self.state = IDLE
            self.reason = ""

    def stop(self, reason: str = "stop requested"):
        self._enter(ABORT, self._state_t)
        self.reason = reason
        self._armed = False

    def step(self, t: float, pose: Pose, scan: Scan) -> MissionOutput:
        if self.state == IDLE and self._armed:
            self._course = CourseFrame(pose)
            self._t0 = t
            self._recoveries = 0
            self._s_obs = self._s_far = self._goal_s = None
            self._last_planner = None
            self._last_goal = None
            self._side_hint = self.p.side_hint
            self.planner.reset()
            self._enter(INIT, t)

        if self.state in (IDLE, ARRIVED, ABORT):
            return self._out(0.0, 0.0)

        if t - self._t0 > self.p.mission_timeout_s:
            self._abort(t, "mission timeout")
            return self._out(0.0, 0.0)

        if t - self._last_recovery_t > self.p.recovery_reset_s and self.state != RECOVERY:
            self._recoveries = 0

        handler = {
            INIT: self._step_init,
            OUTBOUND: self._step_outbound,
            AVOID: self._step_avoid,
            TURNAROUND: self._step_turnaround,
            RETURN: self._step_return,
            RECOVERY: self._step_recovery,
        }[self.state]
        return handler(t, pose, scan)

    def _step_init(self, t, pose, scan):
        if t - self._state_t >= self.p.start_delay_s:
            self._enter(OUTBOUND, t)
        return self._out(0.0, 0.0)

    def _step_outbound(self, t, pose, scan):
        s, _ = self._course.to_course(pose.x, pose.y)
        if s >= self.p.outbound_max_dist - self.p.goal_tol:
            self.reason = "no obstacle within outbound_max_dist"
            self._enter(TURNAROUND, t)
            return self._step_turnaround(t, pose, scan)

        s_obs, s_far = self._detect_obstacle(pose, scan, s)
        if s_obs is not None:
            self._s_obs = s_obs
            self._s_far = max(s_far, s_obs + self.p.obstacle_min_depth)
            self._enter(AVOID, t)
            return self._step_avoid(t, pose, scan)

        goal = self._course.to_world(self.p.outbound_max_dist, 0.0)
        return self._drive(t, pose, scan, goal)

    def _step_avoid(self, t, pose, scan):
        self._refine_far_edge(pose, scan)
        goal_s = self._avoid_goal_s()
        if self._goal_s is None or abs(goal_s - self._goal_s) > 0.05:
            self._goal_s = goal_s
            self._progress_ref = None
        goal = self._course.to_world(goal_s, 0.0)
        if distance(pose, *goal) < self.p.goal_tol:
            self._enter(TURNAROUND, t)
            return self._step_turnaround(t, pose, scan)
        return self._drive(t, pose, scan, goal)

    def _step_turnaround(self, t, pose, scan):
        home = (self._course.origin.x, self._course.origin.y)
        if abs(bearing_to(pose, *home)) < self.p.turn_tol:
            self._enter(RETURN, t)
            return self._step_return(t, pose, scan)
        return self._drive(t, pose, scan, home, check_progress=False)

    def _step_return(self, t, pose, scan):
        home = (self._course.origin.x, self._course.origin.y)
        if distance(pose, *home) < self.p.finish_tol:
            self._enter(ARRIVED, t)
            self.reason = "finish reached"
            return self._out(0.0, 0.0)
        return self._drive(t, pose, scan, home)

    def _step_recovery(self, t, pose, scan):
        if t < self._backup_until:
            xs, ys = scan_points(scan)
            pp = self.planner.p
            rear = float(corridor_free_distance(xs, ys, math.pi, pp.corridor_half_width,
                                                pp.robot_half_length + 1.0)[0])
            if rear > pp.robot_half_length + self.p.rear_clearance:
                return self._out(self.p.backup_vx, 0.0)
            return self._out(0.0, 0.0)
        self.planner.reset()
        self._enter(self._resume_state, t)
        return self._out(0.0, 0.0)

    def _drive(self, t, pose, scan, goal, check_progress=True):
        out = self.planner.compute(t, pose, scan, goal, self._side_hint)
        self._last_planner = out
        self._last_goal = goal

        if out.status == BLOCKED:
            if self._blocked_since is None:
                self._blocked_since = t
            elif t - self._blocked_since >= self.p.blocked_timeout_s:
                return self._start_recovery(t, "blocked")
        else:
            self._blocked_since = None

        if check_progress and self.state in PROGRESS_STATES:
            ref = self._progress_ref
            if ref is None or out.goal_dist < ref[1] - self.p.progress_min:
                self._progress_ref = (t, out.goal_dist)
            elif t - ref[0] >= self.p.progress_window_s:
                return self._start_recovery(t, "no progress")

        return self._out(out.vx, out.wz)

    def _start_recovery(self, t, why):
        self._recoveries += 1
        self._last_recovery_t = t
        if self._recoveries > self.p.max_recoveries:
            self._abort(t, f"{why}: recovery limit reached")
            return self._out(0.0, 0.0)
        side = self.planner.committed_side
        self._side_hint = -side if side else -self._side_hint if self._side_hint else 1
        self._resume_state = self.state
        self._backup_until = t + self.p.backup_duration_s
        self.reason = why
        self._enter(RECOVERY, t)
        return self._out(0.0, 0.0)

    def _detect_obstacle(self, pose, scan, s_robot):
        s, on_track = self._track_points(pose, scan)
        ahead = s - s_robot
        mask = on_track & (ahead > 0.0) & (ahead < self.p.detect_dist)
        if int(mask.sum()) < self.p.detect_min_points:
            return None, None
        s_obs = float(s[mask].min())
        band = on_track & (s >= s_obs) & (s <= s_obs + self.p.obstacle_max_depth)
        return s_obs, float(s[band].max())

    def _refine_far_edge(self, pose, scan):
        s, on_track = self._track_points(pose, scan)
        band = on_track & (s >= self._s_obs - 0.2) & (s <= self._s_obs + self.p.obstacle_max_depth)
        if band.any():
            self._s_far = max(self._s_far, float(s[band].max()))

    def _track_points(self, pose, scan):
        s, lat = self._points_in_course(pose, scan)
        return s, np.abs(lat) < self.p.detect_half_width

    def _avoid_goal_s(self):
        pp = self.planner.p
        cap = self._s_obs + self.p.obstacle_max_depth
        return min(self._s_far, cap) + pp.robot_half_length + self.p.pass_margin

    def _points_in_course(self, pose, scan):
        xs, ys = scan_points(scan)
        if xs.size == 0:
            return xs, ys
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        px = pose.x + c * xs - s * ys
        py = pose.y + s * xs + c * ys
        o = self._course.origin
        co, so = math.cos(o.yaw), math.sin(o.yaw)
        dx, dy = px - o.x, py - o.y
        return co * dx + so * dy, -so * dx + co * dy

    def _enter(self, state, t):
        self.state = state
        self._state_t = t
        self._blocked_since = None
        self._progress_ref = None

    def _abort(self, t, reason):
        self._enter(ABORT, t)
        self.reason = reason
        self._armed = False

    def _out(self, vx, wz):
        info = {"state": self.state, "reason": self.reason, "recoveries": self._recoveries,
                "side_hint": self._side_hint}
        if self._s_obs is not None:
            info["obstacle_s"] = round(self._s_obs, 3)
            info["obstacle_far_s"] = round(self._s_far, 3)
        if self._course is not None:
            o = self._course.origin
            info["start"] = [round(o.x, 3), round(o.y, 3), round(o.yaw, 3)]
        if self._last_goal is not None and self.state not in (ARRIVED, ABORT):
            info["goal"] = [round(self._last_goal[0], 3), round(self._last_goal[1], 3)]
        if self._last_planner is not None:
            lp = self._last_planner
            info.update(planner=lp.status, goal_dist=round(lp.goal_dist, 3),
                        front_free=round(lp.front_free, 3), target=round(lp.target_bearing, 3))
        return MissionOutput(float(vx), float(wz), self.state, info)
