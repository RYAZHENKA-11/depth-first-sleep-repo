"""Выбор цели для Nav2: на цветной маркер, если он в кадре, иначе далёкая точка по
курсу трассы — маршрут в обход препятствий строит планировщик.

Направления хранятся в мировой системе: пеленг в системе робота устаревает, пока
робот доворачивает. Модуль не вызывает drive() — иначе спорил бы с Nav2.
"""
import math
import os

from nav.fallback import obstacle_scan
from nav.perception import PURPLE_RGB, YELLOW_RGB, detect

_DEBUG = os.environ.get("GO2_DEBUG") == "1"
_dbg_tick = 0

LOOKAHEAD_MAX = 2.0
GOAL_RESEND_S = 2.0
# минимум между целями, даже после провала: иначе на серии отказов цель переставляется
# каждый такт и Nav2 не успевает ни спланировать, ни проехать
GOAL_MIN_INTERVAL_S = 0.6
BEARING_RESEND_RAD = math.radians(15)
LOST_MARKER_S = 3.0
ARRIVE_DIST = 0.55
GOAL_MARGIN = 0.3
GOAL_MIN_DIST = 0.3

EXPLORE_FALLBACK_DISTS = (8.0, 4.0, 2.0)
EXPLORE_YAW_NUDGE = math.radians(25)

# курс правим по нетто-смещению: виляние походки иначе работает как храповик
BIAS_NET_STEP = 1.0
BIAS_ALPHA = 0.25
BIAS_MAX_DEV = math.radians(80)
# Трасса по TASK.md: сначала прямо, потом поворот НАЛЕВО на 90°. Направо и назад она не
# уходит никогда, поэтому курс держим в этом секторе от стартового. Без ограничения
# курс сползал на юг (в логах цели уходили в y=-8) и робот покидал трассу.
BIAS_MIN_REL = math.radians(-25)
BIAS_MAX_REL = math.radians(115)

EVAL_PERIOD_S = 0.15

STAGE_COLORS = [YELLOW_RGB, YELLOW_RGB, YELLOW_RGB, PURPLE_RGB]


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class GoalManager:
    def __init__(self, bridge):
        self.bridge = bridge
        self.stage = 0
        self._last_world_bearing = None
        self._last_seen_t = None
        self._last_goal_world_bearing = None
        self._last_goal_t = -1e9
        self._last_goal_xy = None
        self._last_goal_from_marker = False
        self._last_eval_t = -1e9
        self._bias = None
        self._nudge = 0.0
        self._explore_try = 0
        self._pending_source = "explore"
        self._prev_xy = None

    def target_color(self):
        return STAGE_COLORS[min(self.stage, len(STAGE_COLORS) - 1)]

    def observe(self, robot):
        """Курс трассы копится и пока рулит fallback."""
        x, y, yaw = robot.pose()
        self._update_bias(x, y, yaw)

    def tick(self, robot, now):
        found, bearing, _area = detect(robot, self.target_color())
        x, y, yaw = robot.pose()
        self.bridge.publish_marker_bearing(found, bearing, self.stage)

        if found:
            self._last_world_bearing = _wrap(yaw + bearing)
            self._last_seen_t = now

        self._update_bias(x, y, yaw)
        self._check_arrival(x, y, now)

        if now - self._last_eval_t < EVAL_PERIOD_S:
            return
        self._last_eval_t = now
        scan = obstacle_scan(robot)

        if _DEBUG:
            global _dbg_tick
            _dbg_tick += 1
            if _dbg_tick % 7 == 0:
                print(f"[goal_manager] t={now:.1f} pose=({x:.2f},{y:.2f},{yaw:.2f}) "
                      f"stage={self.stage} found={found} nav={self.bridge.nav_status()}",
                      flush=True)

        world_bearing, source = self._pick_direction(robot, now, yaw, scan)
        if world_bearing is None:
            return
        if not self._should_send(world_bearing, now):
            return
        self._send_goal(robot, x, y, yaw, world_bearing, source, now, scan)

    def _pick_direction(self, robot, now, yaw, scan):
        """Маркер, пока он свежий; иначе — далёкая цель по курсу трассы."""
        if self._last_seen_t is not None and now - self._last_seen_t <= LOST_MARKER_S:
            return self._last_world_bearing, "marker"
        return _wrap(self._bias + self._nudge), "explore"

    def _update_bias(self, x, y, yaw):
        if self._bias is None:
            self._bias = yaw
            self._start_yaw = yaw
        if self._prev_xy is None:
            self._prev_xy = (x, y)
            return
        dx, dy = x - self._prev_xy[0], y - self._prev_xy[1]
        if math.hypot(dx, dy) < BIAS_NET_STEP:
            return
        self._prev_xy = (x, y)
        travel = math.atan2(dy, dx)
        if abs(_wrap(travel - self._bias)) > BIAS_MAX_DEV:
            return
        cand = _wrap(self._bias + BIAS_ALPHA * _wrap(travel - self._bias))
        rel = _wrap(cand - self._start_yaw)
        self._bias = _wrap(self._start_yaw + min(BIAS_MAX_REL, max(BIAS_MIN_REL, rel)))

    def _should_send(self, world_bearing, now):
        if now - self._last_goal_t < GOAL_MIN_INTERVAL_S:
            return False
        if self.bridge.nav_status() == "failed":
            if self._pending_source == "explore":
                self._explore_try += 1
                if self._explore_try % len(EXPLORE_FALLBACK_DISTS) == 0:
                    self._nudge = _wrap(self._nudge + EXPLORE_YAW_NUDGE)
            return True
        if self._last_goal_world_bearing is None:
            return True
        if abs(_wrap(world_bearing - self._last_goal_world_bearing)) > BEARING_RESEND_RAD:
            return True
        return now - self._last_goal_t > GOAL_RESEND_S

    def _send_goal(self, robot, x, y, yaw, world_bearing, source, now, scan):
        rel = _wrap(world_bearing - yaw)
        if source == "marker":
            free = self._free_along(robot, rel, scan)
            dist = max(GOAL_MIN_DIST, min(free - GOAL_MARGIN, LOOKAHEAD_MAX))
        else:
            dist = EXPLORE_FALLBACK_DISTS[self._explore_try % len(EXPLORE_FALLBACK_DISTS)]
        gx = x + dist * math.cos(world_bearing)
        gy = y + dist * math.sin(world_bearing)

        if not self.bridge.send_goal(gx, gy):
            return
        self._pending_source = source
        self._last_goal_world_bearing = world_bearing
        self._last_goal_t = now
        self._last_goal_xy = (gx, gy)
        self._last_goal_from_marker = source == "marker"
        if _DEBUG:
            print(f"[goal_manager] t={now:.1f} цель[{source}] stage={self.stage} "
                  f"rel={rel:+.2f} dist={dist:.2f} -> ({gx:.2f},{gy:.2f})", flush=True)

    def _free_along(self, robot, rel_bearing, scan):
        h_fov, h_res, _max_range, _layers, _v_fov = robot.lidar_info()
        if not scan or h_res <= 1:
            return LOOKAHEAD_MAX
        idx = int(round((rel_bearing + h_fov / 2) / h_fov * (h_res - 1)))
        idx = max(0, min(h_res - 1, idx))
        return scan[idx]

    def _check_arrival(self, x, y, now):
        """Доехали до цели по маркеру — берём следующий цвет. Цели разведки не считаются."""
        if self._last_goal_xy is None or not self._last_goal_from_marker:
            return
        if math.hypot(x - self._last_goal_xy[0], y - self._last_goal_xy[1]) >= ARRIVE_DIST:
            return
        self.stage = min(self.stage + 1, len(STAGE_COLORS) - 1)
        self._last_goal_xy = None
        self._last_goal_from_marker = False
        self._last_seen_t = None
        self._last_world_bearing = None
        if _DEBUG:
            print(f"[goal_manager] t={now:.1f} маркер достигнут -> stage={self.stage}",
                  flush=True)
