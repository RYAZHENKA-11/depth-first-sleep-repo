"""
Shared helpers for talking to a Unitree Go2 Pro over the LOCAL WebRTC API.
No ROS, no root — pure Python. Connection method: LocalSTA (by IP).

Robot signaling endpoint: http://<ip>:9991  (con_notify / con_ing).
Our robot returned data2=2 → static AES-GCM key, handled automatically,
so NO per-device AES key is needed.

Env:
  UNITREE_ROBOT_IP   robot IP            (default 192.168.123.161)
  GO2_LOGLEVEL       lib log level       (default FATAL; try INFO to debug)
"""
import asyncio
import json
import logging
import os

logging.basicConfig(level=os.environ.get("GO2_LOGLEVEL", "FATAL"))

# Headless node has no audio server → stub sounddevice (data-channel control needs no audio).
# Заглушка ПОЛНАЯ: webrtc_audio.py трогает sd.PortAudioError и sd.default при загрузке — без них
# заглушка «наполовину» роняла импорт либы на картах без аудио-сервера (PulseAudio лежит).
try:
    import sounddevice  # noqa: F401
except Exception:
    import sys as _sys
    import types as _types
    _sd = _types.ModuleType("sounddevice")
    _sd.query_devices = lambda *a, **k: []
    _sd.OutputStream = _sd.InputStream = _sd.RawOutputStream = object
    _sd.PortAudioError = Exception
    _sd.default = type("_SDDefault", (), {})()
    _sys.modules["sounddevice"] = _sd

from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection,
    WebRTCConnectionMethod,
)
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD  # noqa: F401

# ── Блок опасных команд ─────────────────────────────────────────────────────────
# Все sport-команды (по имени И «сырым» api_id) уходят к роботу через ОДНУ точку —
# pub_sub.publish_request_new(SPORT_MOD, {api_id}). Вешаем фильтр там: покрывает go2.sport,
# ROS-мост и любой код участника, который импортирует go2. Список — акробатика/трюки.
DENY_SPORT_IDS = {
    1030, 1044, 1042, 1043,  # FrontFlip / BackFlip / LeftFlip / RightFlip — сальто
    1031, 1032,              # FrontJump / FrontPounce — прыжки
    1301,                    # Handstand — стойка на лапах
    1304, 1305, 1302, 1303,  # Bound / MoonWalk / CrossStep / OnesidedStep
    1022, 1023,              # Dance1 / Dance2
    1029, 1039, 1021,        # Scrape / StandOut / Wallow
}
try:
    from unitree_webrtc_connect.msgs.pub_sub import WebRTCDataChannelPubSub as _PubSub
    if not getattr(_PubSub, "_fleet_guarded", False):
        _SPORT_TOPIC = RTC_TOPIC["SPORT_MOD"]
        _orig_pub = _PubSub.publish_request_new

        async def _guarded_pub(self, topic, options=None):
            if topic == _SPORT_TOPIC and options and options.get("api_id") in DENY_SPORT_IDS:
                raise PermissionError(f"опасная команда заблокирована (api_id={options.get('api_id')})")
            return await _orig_pub(self, topic, options)

        _PubSub.publish_request_new = _guarded_pub
        _PubSub._fleet_guarded = True
except Exception as _e:  # noqa: BLE001
    print(f"[go2] не удалось поставить фильтр опасных команд: {_e}")

ROBOT_IP = os.environ.get("UNITREE_ROBOT_IP", "192.168.123.161")
def _load_aes_key():
    v = (os.environ.get("UNITREE_AES_KEY") or "").strip()
    if v:
        return v
    try:
        with open(os.path.expanduser("~/.fleet/aes_key")) as f:
            return f.read().strip() or None
    except OSError:
        return None


AES_KEY = _load_aes_key()  # env UNITREE_AES_KEY, иначе ~/.fleet/aes_key (ставит дашборд/агент)


async def connect(ip: str | None = None, retries: int = 5, backoff: float = 4.0,
                  aes_128_key: str | None = None):
    """Connect over LocalSTA with retry.

    The robot throttles/black-holes rapid TCP probes, so the library's port
    check (socket.create_connection on :9991) can transiently fail with
    LocalSignalingPortError even though the endpoint is alive. We retry with a
    backoff instead of giving up. Don't run nmap / port scanners against the
    robot in parallel — that is what trips the throttle.
    """
    ip = ip or ROBOT_IP
    key = aes_128_key or AES_KEY
    last = None
    for attempt in range(1, retries + 1):
        if key:
            conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ip, aes_128_key=key)
        else:
            conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ip)
        try:
            print(f"[*] Connecting to {ip} (attempt {attempt}/{retries})...")
            await conn.connect()
            print("[+] Connected — data channel open.")
            return conn
        except Exception as e:  # noqa: BLE001
            last, name = e, type(e).__name__
            print(f"[!] {name}: {e}")
            if name == "AesKeyRequiredError":
                print("    Firmware wants a per-device AES key (data2=3). "
                      "Fetch with: ./venv/bin/unitree-fetch-aes-key  (not our case: data2=2).")
                raise
            if name == "RobotBusyError":
                print("    Close the Unitree mobile app — it holds the single WebRTC slot.")
            if name == "LocalSignalingPortError":
                print("    :9991 probe failed — robot may be booting/throttling or powered off.")
            if attempt < retries:
                print(f"    Retrying in {backoff:.0f}s...")
                await asyncio.sleep(backoff)
    raise last


async def disconnect(conn) -> None:
    """Best-effort graceful close (method name varies across lib versions)."""
    for closer in ("disconnect", "close", "stop"):
        fn = getattr(conn, closer, None)
        if fn:
            try:
                res = fn()
                if asyncio.iscoroutine(res):
                    await res
            except Exception:  # noqa: BLE001
                pass
            return


async def ensure_normal_mode(conn) -> str | None:
    """Make sure the robot is in 'normal' sport mode (not 'ai'/'mcf').
    Needed before high-level posture/locomotion commands behave predictably."""
    resp = await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1001}
    )
    mode = None
    try:
        if resp["data"]["header"]["status"]["code"] == 0:
            mode = json.loads(resp["data"]["data"])["name"]
    except Exception:  # noqa: BLE001
        pass
    print(f"[*] Current motion mode: {mode}")
    if mode and mode != "normal":
        print("[*] Switching motion mode -> 'normal' ...")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1002, "parameter": {"name": "normal"}}
        )
        await asyncio.sleep(5)  # give it time to stand into normal mode
    return mode


async def sport(conn, cmd_name: str, parameter: dict | None = None):
    """Send one high-level sport command by friendly SPORT_CMD key."""
    payload = {"api_id": SPORT_CMD[cmd_name]}
    if parameter is not None:
        payload["parameter"] = parameter
    return await conn.datachannel.pub_sub.publish_request_new(RTC_TOPIC["SPORT_MOD"], payload)
