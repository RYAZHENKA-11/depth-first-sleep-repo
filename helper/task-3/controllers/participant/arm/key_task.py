"""Автомат руки: перенести ключ-куб в приёмник. Один шаг за тик, собака едет параллельно.

Ключ и приёмник закреплены на самой станции и в ЕЁ системе координат стоят всегда
одинаково (шаблон генератора: ключ на тумбе в -0.40 по y, приёмник в +0.40). Это
геометрия стенда, а не координаты трассы, поэтому её можно брать напрямую. Если захват
не удался, пробуем соседние точки — на случай, если стенд соберут чуть иначе.
"""
import os

KEY_XY = (0.0, -0.40)
SOCKET_XY = (0.0, 0.40)

Z_HOVER = 0.45
Z_GRASP = 0.36          # кончик в 7 см от центра куба — внутри радиуса магнита (9 см)
Z_PLACE = 0.40          # объект висит на 7.5 см ниже кончика; плита приёмника на 0.28
Z_LIFT = 0.50

SETTLE_TIMEOUT_S = 8.0  # ход от парковки до рабочей точки идёт несколько секунд
GRAB_TIMEOUT_S = 2.0
TASK_TIMEOUT_S = 200.0
RETRY_OFFSETS = ((0.0, 0.0), (0.06, 0.0), (-0.06, 0.0), (0.0, 0.06), (0.0, -0.06))

_DEBUG = os.environ.get("GO2_DEBUG") == "1"


class KeyTask:
    def __init__(self):
        self.state = "HOVER_KEY"
        self.done = False
        self.ok = False
        self._t0 = None
        self._deadline = 0.0
        self._try = 0

    def _dbg(self, msg):
        if _DEBUG:
            print(f"[key_task] {msg}", flush=True)

    def _key_xy(self):
        dx, dy = RETRY_OFFSETS[self._try % len(RETRY_OFFSETS)]
        return KEY_XY[0] + dx, KEY_XY[1] + dy

    def _goto(self, arm, now, xy, z, next_state):
        arm.move(xy[0], xy[1], z)
        self._deadline = now + SETTLE_TIMEOUT_S
        self.state = next_state

    def _arrived(self, arm, now):
        return arm.at_target() or now > self._deadline

    def tick(self, robot):
        if self.done:
            return
        arm = robot.arm
        if not getattr(arm, "ok", False):
            self.done = True
            return

        now = robot.time()
        if self._t0 is None:
            self._t0 = now
            self._goto(arm, now, self._key_xy(), Z_HOVER, "HOVER_KEY_WAIT")
            return
        if now - self._t0 > TASK_TIMEOUT_S:
            self._dbg(f"t={now:.1f} общий таймаут — сворачиваюсь")
            self._finish(arm, False)
            return

        getattr(self, f"_st_{self.state.lower()}")(arm, now)

    def _st_hover_key(self, arm, now):
        self._goto(arm, now, self._key_xy(), Z_HOVER, "HOVER_KEY_WAIT")

    def _st_hover_key_wait(self, arm, now):
        if self._arrived(arm, now):
            self._goto(arm, now, self._key_xy(), Z_GRASP, "DOWN_WAIT")

    def _st_down_wait(self, arm, now):
        if self._arrived(arm, now):
            arm.grab()
            self._deadline = now + GRAB_TIMEOUT_S
            self.state = "GRAB_WAIT"

    def _st_grab_wait(self, arm, now):
        if arm.holding():
            self._dbg(f"t={now:.1f} ключ взят с попытки {self._try + 1}")
            self._goto(arm, now, self._key_xy(), Z_LIFT, "LIFT_WAIT")
            return
        if now > self._deadline:
            self._try += 1
            if self._try >= len(RETRY_OFFSETS):
                self._dbg(f"t={now:.1f} захват не удался, попытки кончились")
                self._finish(arm, False)
                return
            self._dbg(f"t={now:.1f} захват не удался, пробую смещение {self._try}")
            self.state = "HOVER_KEY"

    def _st_lift_wait(self, arm, now):
        if self._arrived(arm, now):
            self._goto(arm, now, SOCKET_XY, Z_LIFT, "OVER_SOCKET_WAIT")

    def _st_over_socket_wait(self, arm, now):
        if self._arrived(arm, now):
            self._goto(arm, now, SOCKET_XY, Z_PLACE, "PLACE_WAIT")

    def _st_place_wait(self, arm, now):
        if self._arrived(arm, now):
            arm.release()
            self._dbg(f"t={now:.1f} ключ отпущен в приёмнике")
            self._goto(arm, now, SOCKET_XY, Z_LIFT, "RETREAT_WAIT")

    def _st_retreat_wait(self, arm, now):
        if self._arrived(arm, now):
            self._finish(arm, True)

    def _finish(self, arm, ok):
        if not ok and arm.holding():
            arm.release()
        arm.stow()
        self.state = "DONE"
        self.done, self.ok = True, ok

    def _st_done(self, arm, now):
        pass
