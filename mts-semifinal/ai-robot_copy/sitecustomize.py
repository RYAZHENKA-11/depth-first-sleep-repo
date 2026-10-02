"""Venv-wide блок опасных sport-команд робота.

Кладётся в site-packages venv (~/ai-robot/venv) и авто-грузится Python'ом на КАЖДОМ старте.
Ставит фильтр на единственную точку отправки команд роботу
(unitree_webrtc_connect ... pub_sub.publish_request_new на топик rt/api/sport/request).

Покрывает ОБА пути и любой способ вызова:
  • WebRTC — наши 01/02/03/go2 и любой участник, кто шлёт по имени ИЛИ «сырым» api_id;
  • ROS 2 — ros_bridge и любая нода участника, даже если она импортирует библиотеку напрямую,
    минуя наш go2.
Список DENY — акробатика/прыжки/трюки (высокий риск травмы/поломки/падения).
"""
DENY_SPORT_IDS = {
    1030, 1044, 1042, 1043,  # FrontFlip / BackFlip / LeftFlip / RightFlip — сальто
    1031, 1032,              # FrontJump / FrontPounce — прыжки
    1301,                    # Handstand — стойка на лапах
    1304, 1305, 1302, 1303,  # Bound / MoonWalk / CrossStep / OnesidedStep
    1022, 1023,              # Dance1 / Dance2
    1029, 1039, 1021,        # Scrape / StandOut / Wallow
}

try:
    from unitree_webrtc_connect.msgs.pub_sub import WebRTCDataChannelPubSub as _P
    from unitree_webrtc_connect.constants import RTC_TOPIC as _T

    if not getattr(_P, "_fleet_guarded", False):
        _sport = _T["SPORT_MOD"]
        _orig = _P.publish_request_new

        async def _guarded(self, topic, options=None):
            if topic == _sport and options and options.get("api_id") in DENY_SPORT_IDS:
                raise PermissionError(
                    f"опасная команда заблокирована (api_id={options.get('api_id')})")
            return await _orig(self, topic, options)

        _P.publish_request_new = _guarded
        _P._fleet_guarded = True
except Exception:
    # библиотеки ещё нет (первый pip) или другой формат — молча пропускаем, управление не ломаем
    pass


# ── Лог кода, прогоняемого через stdin (`python -`) ─────────────────────────────────────────────
# Участники запускают `venv/bin/python -` и скармливают код по конвейеру — на диск он не ложится,
# снапшот его не видит (ровно так уходил от аудита pi-0008). Ловим: сами читаем программу из stdin,
# пишем её в сессию (pyexec-stdin.log — уедет в дашборд) и исполняем как обычный __main__.
# DENY-фильтр опасных команд уже стоит выше, так что он действует и на этот код.
import sys as _sys
try:
    _a0 = _sys.argv[0] if _sys.argv else ""
    # ловим только реальный stdin-код: `python -` или `... | python`, но НЕ интерактив, НЕ файл, НЕ -c
    if _a0 in ("-", "") and not _sys.stdin.isatty():
        import os as _os
        import time as _t
        _code = _sys.stdin.read()
        try:
            _d = _os.path.expanduser("~/.fleet/sessions")
            _os.makedirs(_d, exist_ok=True)
            with open(_os.path.join(_d, "pyexec-stdin.log"), "a") as _f:
                _f.write("\n===== %s  argv=%r  cwd=%s =====\n"
                         % (_t.strftime("%Y-%m-%dT%H:%M:%S"), _sys.argv, _os.getcwd()))
                _f.write(_code)
                if not _code.endswith("\n"):
                    _f.write("\n")
        except Exception:
            pass
        _rc = 0
        _g = {"__name__": "__main__", "__file__": "<stdin>"}
        try:
            exec(compile(_code, "<stdin>", "exec"), _g)
        except SystemExit as _e:
            _rc = _e.code if isinstance(_e.code, int) else (0 if _e.code is None else 1)
        except BaseException:
            import traceback as _tb
            _tb.print_exc()
            _rc = 1
        try:
            _sys.stdout.flush()
            _sys.stderr.flush()
        except Exception:
            pass
        _os._exit(_rc if isinstance(_rc, int) else 0)
except SystemExit:
    raise
except Exception:
    pass
