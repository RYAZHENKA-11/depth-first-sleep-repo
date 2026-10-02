#!/usr/bin/env python3
"""Резидентный агент управления собакой Go2.

Держит ОДИН постоянный WebRTC-конект и мгновенно исполняет команды с сервера через
long-poll. Раньше был oneshot по таймеру раз в 5с с реконнектом на каждую команду
(лаг 5–10с); теперь отклик <1с. Запускается venv-питоном под ubuntu (нужен import go2)."""
import asyncio
import json
import os
import ssl
import sys
import time
import urllib.request

HOME = os.path.expanduser("~")
AI_DIR = os.path.join(HOME, "ai-robot")
KEY_FILE = os.path.join(HOME, ".fleet", "aes_key")
POLL_WAIT = 25  # сек: сервер держит long-poll до команды или таймаута

sys.path.insert(0, AI_DIR)  # go2.py лежит рядом со скриптами, не в site-packages
import go2  # noqa: E402
from unitree_webrtc_connect.constants import RTC_TOPIC  # noqa: E402

# CLI-имя -> ключ SPORT_CMD (сверено с 03_command.py / API.md). Без акробатики.
SPORT = {
    "stand_up": "StandUp", "stand_down": "StandDown", "sit": "Sit", "rise_sit": "RiseSit",
    "balance": "BalanceStand", "recovery": "RecoveryStand", "hello": "Hello",
    "stretch": "Stretch", "wiggle": "WiggleHips", "heart": "FingerHeart",
    "damp": "Damp", "stop": "StopMove",
}
ALLOWED = set(SPORT) | {"move"}
LIM = {"x": 0.6, "y": 0.5, "z": 1.0}
MOVE_MAX = 3.0  # сек: потолок одного move (сервер зажимает так же)
EXIT_CANCELLED = 130  # рапорт «прервано СТОПом» — сервер ставит status=cancelled


def load_env(path="/etc/pi-fleet/fleet.env"):
    env = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    env[k.strip()] = v.strip()
    except OSError:
        pass
    return env


def read_key():
    try:
        with open(KEY_FILE) as f:
            return f.read().strip()
    except OSError:
        return ""


def write_key(key):
    os.makedirs(os.path.dirname(KEY_FILE), exist_ok=True)
    tmp = KEY_FILE + ".tmp"
    with open(tmp, "w") as f:
        f.write(key)
    os.chmod(tmp, 0o600)
    os.replace(tmp, KEY_FILE)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _http(url, token, data, timeout):
    headers = {"X-Fleet-Token": token}
    body = None
    if data is not None:
        body = json.dumps(data).encode()
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=body, headers=headers)
    with urllib.request.urlopen(req, timeout=timeout, context=ssl.create_default_context()) as r:
        return json.loads(r.read().decode())


async def http(url, token, data=None, timeout=45):
    return await asyncio.to_thread(_http, url, token, data, timeout)


async def pull(server, token, card, wait, keyonly=False):
    q = f"{server}/api/agent/pull?card={card}&wait={wait}"
    if keyonly:  # лёгкая сверка ключа ДО connect — сервер не трогает/не диспатчит команды
        q += "&keyonly=1"
    return await http(q, token, timeout=wait + 20)


async def report(server, token, cid, code, output):
    try:
        await http(f"{server}/api/agent/command-result", token,
                   {"id": cid, "exit_code": code, "output": output[-2000:]}, timeout=20)
    except Exception:
        pass


_BG = set()


def spawn(coro):
    """Фоновая задача без ожидания (ссылку держим, чтобы GC не прибил её на полпути)."""
    t = asyncio.ensure_future(coro)
    _BG.add(t)
    t.add_done_callback(_BG.discard)
    return t


async def _quiet(fn, *a):
    try:
        await asyncio.to_thread(fn, *a)
    except Exception:
        pass


# ── Камера собаки (аддитивно; любой сбой видео не влияет на управление) ──────────
def _post_frame(server, token, card, jpeg):
    req = urllib.request.Request(
        f"{server}/api/machines/{card}/frame", data=jpeg,
        headers={"X-Fleet-Token": token, "Content-Type": "image/jpeg"})
    with urllib.request.urlopen(req, timeout=10, context=ssl.create_default_context()) as r:
        r.read()


def _post_battery(server, token, card, soc):
    req = urllib.request.Request(
        f"{server}/api/machines/{card}/battery", data=json.dumps({"soc": soc}).encode(),
        headers={"X-Fleet-Token": token, "Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10, context=ssl.create_default_context()) as r:
        r.read()


def _jpeg_encoder():
    """JPEG-энкодер кадра: cv2 (всегда в стеке go2_webrtc), иначе Pillow, иначе None."""
    try:
        import cv2  # noqa: F401

        def enc(frame):
            arr = frame.to_ndarray(format="bgr24")  # av.VideoFrame -> numpy BGR
            h, w = arr.shape[:2]
            if w > 640:
                arr = cv2.resize(arr, (640, max(1, h * 640 // w)))
            ok, buf = cv2.imencode(".jpg", arr, [cv2.IMWRITE_JPEG_QUALITY, 55])
            return buf.tobytes() if ok else None

        return enc
    except Exception:
        pass
    try:
        from io import BytesIO
        from PIL import Image  # noqa: F401

        def enc(frame):
            img = frame.to_image()  # av.VideoFrame -> PIL.Image
            if img.width > 640:
                img = img.resize((640, max(1, img.height * 640 // img.width)))
            buf = BytesIO()
            img.save(buf, format="JPEG", quality=55)
            return buf.getvalue()

        return enc
    except Exception:
        return None


async def _camera_recv(track, cam):
    """Колбэк видео-трека: держим ПОСЛЕДНИЙ кадр как JPEG (кодируем только когда смотрят)."""
    encode = _jpeg_encoder()
    if encode is None:
        return  # ни cv2, ни Pillow — видео недоступно (управление/лидар не трогаем)
    while True:
        try:
            frame = await track.recv()
        except Exception:
            return  # трек закрылся (видео выключили/обрыв) — переустановится на реконнекте
        if not cam["on"]:
            continue
        try:
            j = encode(frame)
            if j:
                cam["jpeg"] = j
        except Exception:
            pass


async def _camera_pusher(cam, server, token, card):
    """Шлём последний кадр на сервер ~24 к/с, пока кто-то смотрит."""
    while True:
        await asyncio.sleep(1 / 24)
        j = cam["jpeg"]
        if cam["on"] and j:
            try:
                await asyncio.to_thread(_post_frame, server, token, card, j)
            except Exception:
                pass


def _camera_set(conn, cam, on):
    """Вкл/выкл видео-канал по флагу «смотрю» из pull."""
    if cam.get("bad") or on == cam["on"]:
        return
    try:
        conn.video.switchVideoChannel(on)
        cam["on"] = on
        print(f"[fleet-dog] камера {'включена' if on else 'выключена'}", flush=True)
    except Exception as ex:
        cam["bad"] = True
        print(f"[fleet-dog] видео недоступно: {ex}", flush=True)


# ---------- лидар: облако точек -> бинарь, пуш на сервер (как камера, по требованию) ----------
# Рендер 3D делает БРАУЗЕР (Three.js). Малинка лишь готовит компактное облако:
# роботоцентрично (сдвиг на позу собаки + поворот на yaw), радиус-клип, воксель-дедуп,
# int16-сетка, zlib. Браузер разжимает (DecompressionStream('deflate')) и рисует воксели.
def _post_lidar_cloud(server, token, card, body):
    req = urllib.request.Request(
        f"{server}/api/machines/{card}/lidar-frame", data=body,
        headers={"X-Fleet-Token": token, "Content-Type": "application/octet-stream"})
    with urllib.request.urlopen(req, timeout=10, context=ssl.create_default_context()) as r:
        r.read()


def _encode_cloud(points, pose, res, rng=6.5, zlo=-1.2, zhi=2.8, maxv=20000, floor=0.06):
    """Облако Nx3 (метры, СК одометрии) + поза {x,y,yaw} + res (родное разрешение воксель-карты
    Go2, обычно 0.05 м) -> zlib(int16-воксели, СК робота). Сетка = max(родное, floor) — держим
    точность у источника, но не мельче floor (иначе трафик/CPU растут). Плюс лёгкая чистка шума.
    Формат тела до сжатия: '<If' (count, grid_m) + int16[count*3] (gi,gj,gk = вперёд,влево,верх)."""
    import numpy as np
    import struct
    import zlib
    p = np.asarray(points, dtype=np.float32)
    if p.ndim != 2 or p.shape[1] < 3 or len(p) == 0:
        return None
    grid = float(res) if (res and float(res) > 0.01) else 0.05
    if grid < floor:
        grid = floor
    x = p[:, 0] - float(pose.get("x", 0.0))
    y = p[:, 1] - float(pose.get("y", 0.0))
    yaw = float(pose.get("yaw", 0.0))
    cs, sn = np.cos(yaw), np.sin(yaw)
    fx = x * cs + y * sn      # вперёд (нос собаки)
    ly = -x * sn + y * cs     # влево
    z = p[:, 2]
    m = (fx * fx + ly * ly <= rng * rng) & (z >= zlo) & (z <= zhi)
    fx, ly, z = fx[m], ly[m], z[m]
    if len(fx) == 0:
        return None
    gi = np.round(fx / grid).astype(np.int32)
    gj = np.round(ly / grid).astype(np.int32)
    gk = np.round(z / grid).astype(np.int32)
    v = np.unique(np.stack([gi, gj, gk], axis=1), axis=0)  # воксели родной сетки: дедуп + сортировка
    # чистка шума: выкинуть одиночные воксели (нет ни одного из 26 соседей) — артефакты лидара
    if len(v) > 60:
        off = np.int64(1 << 15)
        vi = v.astype(np.int64) + off
        base = np.sort((vi[:, 0] << 34) | (vi[:, 1] << 17) | vi[:, 2])
        neigh = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    neigh.append(((vi[:, 0] + dx) << 34) | ((vi[:, 1] + dy) << 17) | (vi[:, 2] + dz))
        allk = np.concatenate(neigh)
        idx = np.clip(np.searchsorted(base, allk), 0, len(base) - 1)
        has = (base[idx] == allk).reshape(26, len(v)).any(axis=0)
        v = v[has]
    if len(v) == 0:
        return None
    if len(v) > maxv:
        v = v[np.linspace(0, len(v) - 1, maxv).astype(np.int64)]
    head = struct.pack("<If", len(v), float(grid))
    return zlib.compress(head + v.astype("<i2").tobytes(), 6)


def _lidar_recv(msg, lid):
    """Колбэк облака: кодируем не чаще ~4 к/с и только когда смотрят."""
    if not lid.get("on"):
        return
    import time
    now = time.monotonic()
    if now - lid.get("t", 0.0) < 0.25:
        return
    lid["t"] = now
    try:
        d = msg["data"]
        lid["cloud"] = _encode_cloud(d["data"]["points"], lid.get("pose") or {}, d.get("resolution"))
    except Exception:
        pass


async def _lidar_pusher(lid, server, token, card):
    while True:
        await asyncio.sleep(0.22)
        b = lid.get("cloud")
        if lid.get("on") and b:
            try:
                await asyncio.to_thread(_post_lidar_cloud, server, token, card, b)
            except Exception:
                pass


async def _lidar_set(conn, lid, on):
    """Вкл/выкл лидар по флагу «смотрю». native-декодер + подписка — ЛЕНИВО при первом
    включении. Всё в try/except: если лидар отвалится — камера и управление не страдают."""
    if lid.get("bad") or on == lid.get("on"):
        return
    try:
        if on and not lid.get("subscribed"):
            conn.datachannel.set_decoder("native")
            conn.datachannel.pub_sub.subscribe(RTC_TOPIC["ULIDAR_ARRAY"], lambda m: _lidar_recv(m, lid))
            lid["subscribed"] = True
        await conn.datachannel.disableTrafficSaving(bool(on))  # True шлёт лидар, False — молчит
        lid["on"] = on
        if not on:
            lid["cloud"] = None
        print(f"[fleet-dog] лидар {'вкл' if on else 'выкл'}", flush=True)
    except Exception as ex:
        lid["bad"] = True
        print(f"[fleet-dog] лидар недоступен: {ex}", flush=True)


def _ok(resp):
    try:
        return resp["data"]["header"]["status"]["code"] == 0
    except Exception:
        return True  # неожиданный формат ответа не считаем ошибкой


def _alive(conn):
    """Дёшево, без публикаций: жив ли WebRTC-канал (собаку мог забрать другой клиент/обрыв)."""
    try:
        if not conn.datachannel.data_channel_opened:
            return False
        cs = getattr(conn.pc, "connectionState", None)
        return cs in (None, "new", "connecting", "connected")
    except Exception:
        return True  # не смогли проверить — не паникуем


async def do_command(conn, cmd, args, drive):
    """Выполнить одну команду на уже открытом конекте. -> (exit_code, output).
    move не блокирует: задаёт уставку скорости до дедлайна, её крутит driver()."""
    if cmd == "move":
        a = json.loads(args or "{}")
        x = clamp(float(a.get("x", 0)), -LIM["x"], LIM["x"])
        y = clamp(float(a.get("y", 0)), -LIM["y"], LIM["y"])
        z = clamp(float(a.get("z", 0)), -LIM["z"], LIM["z"])
        dur = clamp(float(a.get("duration", 1) or 1), 0.1, MOVE_MAX)
        drive["v"] = {"x": x, "y": y, "z": z}
        drive["until"] = time.monotonic() + dur
        drive["evt"].set()
        return 0, f"move x={x} y={y} z={z} {dur}s"

    if drive["v"]:  # поза на ходу — сначала останавливаемся
        drive["v"] = None
        await asyncio.wait_for(go2.sport(conn, "StopMove"), timeout=8)
    resp = await asyncio.wait_for(go2.sport(conn, SPORT[cmd]), timeout=8)
    if not _ok(resp):  # вдруг вышли из normal — вернём режим и повторим один раз
        await asyncio.wait_for(go2.ensure_normal_mode(conn), timeout=12)
        resp = await asyncio.wait_for(go2.sport(conn, SPORT[cmd]), timeout=8)
    return (0 if _ok(resp) else 1), SPORT[cmd] + (" ok" if _ok(resp) else " не выполнено")


async def driver(conn, drive):
    """Езда по уставке: Move ~10 Гц до дедлайна, истёк сам — один StopMove. Новый move лишь
    меняет/продлевает уставку — джойстик едет плавно, и в очереди ничего не копится."""
    while True:
        await drive["evt"].wait()
        drive["evt"].clear()
        while drive["v"] and time.monotonic() < drive["until"]:
            try:
                await asyncio.wait_for(go2.sport(conn, "Move", drive["v"]), timeout=8)
            except Exception as ex:
                print(f"[fleet-dog] Move не ушёл: {ex}", flush=True)
            await asyncio.sleep(0.1)
        if drive["v"]:  # дедлайн истёк сам (не СТОП) — останавливаемся
            drive["v"] = None
            try:
                await asyncio.wait_for(go2.sport(conn, "StopMove"), timeout=8)
            except Exception:
                pass


async def session(server, token, card):
    """Один жизненный цикл: connect -> normal -> long-poll команд, пока конект жив."""
    # Сервер — источник правды по ключу (селектор собаки в дашборде). Сверяемся ДО connect:
    # иначе застрявший неверный ключ роняет connect() раньше, чем сработает смена ключа
    # в цикле ниже — и малинка навсегда залипает в цикле «AES key rejected».
    key = read_key()
    try:
        p = await pull(server, token, card, 0, keyonly=True)
        sk = (p.get("aes_key") or "").strip()
        if sk and sk != key:
            write_key(sk)
            key = sk
        elif not sk:
            key = ""  # собаку сняли/не назначили в дашборде — не лезем со старым ключом
    except Exception:
        pass  # сервер недоступен — пробуем с локальным ключом
    if not key:  # ключа нет — ждущие команды отобьём понятной ошибкой и подождём
        try:
            p = await pull(server, token, card, 0)
            for c in p.get("commands") or []:
                await report(server, token, c.get("id"), 3,
                             "нет AES-ключа на малинке — задай собаку во вкладке «Собака»")
        except Exception:
            pass
        await asyncio.sleep(3)
        return

    conn = await go2.connect(aes_128_key=key)
    dead = asyncio.Event()
    cam = {"on": False, "jpeg": None}  # камера: по требованию, кадры шлём только когда смотрят
    try:
        conn.video.add_track_callback(lambda t: _camera_recv(t, cam))
    except Exception as ex:  # старая либа без видео — не мешаем управлению
        cam["bad"] = True
        print(f"[fleet-dog] камера недоступна: {ex}", flush=True)
    cam_task = asyncio.ensure_future(_camera_pusher(cam, server, token, card))
    pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}  # одометрия собаки (для роботоцентричного лидара)
    lid = {"on": False, "cloud": None, "t": 0.0, "pose": pose}  # лидар: по требованию, как камера
    lid_task = asyncio.ensure_future(_lidar_pusher(lid, server, token, card))

    bat = {"soc": None, "t": 0.0}  # заряд собаки (soc из BMS) — шлём на сервер в цикле опроса (для страницы камер)

    def _bat_cb(msg):
        try:
            d = msg.get("data", msg) if isinstance(msg, dict) else msg
            bms = (d.get("bms_state") or d.get("bms")) if isinstance(d, dict) else None
            soc = bms.get("soc") if isinstance(bms, dict) else None
            if isinstance(soc, (int, float)) and 0 <= soc <= 100:
                bat["soc"] = int(soc)
        except Exception:
            pass

    try:
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LOW_STATE"], _bat_cb)
    except Exception:
        pass

    def _pose_cb(msg):  # поза собаки из sportmodestate -> для центрирования облака на собаке
        try:
            d = msg.get("data", msg) if isinstance(msg, dict) else msg
            pos = d.get("position") if isinstance(d, dict) else None
            imu = (d.get("imu_state") or {}) if isinstance(d, dict) else {}
            rpy = imu.get("rpy") or []
            if isinstance(pos, list) and len(pos) >= 2:
                pose["x"], pose["y"] = float(pos[0]), float(pos[1])
                if isinstance(rpy, (list, tuple)) and len(rpy) >= 3:
                    pose["yaw"] = float(rpy[2])
        except Exception:
            pass

    try:
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], _pose_cb)
    except Exception:
        pass

    async def watchdog():  # ловим мёртвый канал за ~2с даже без команд -> реконнект
        while True:
            await asyncio.sleep(2)
            if not _alive(conn):
                dead.set()
                return

    # опрос сервера висит ВСЕГДА (и пока собака идёт) — СТОП долетает сразу, а не после очереди
    drive = {"v": None, "until": 0.0, "evt": asyncio.Event()}  # уставка движения, её крутит driver()
    jobs = asyncio.Queue()  # позы и прочее — строго по порядку; move здесь лишь меняет уставку
    reports = asyncio.Queue()  # рапорты шлём в фоне по одному — исполнитель не ждёт HTTP

    async def reporter():
        while True:
            await report(server, token, *(await reports.get()))

    async def executor():
        while True:
            c = await jobs.get()
            cid, cmd = c.get("id"), c.get("cmd", "")
            if cmd not in ALLOWED:
                reports.put_nowait((cid, 2, f"команда не разрешена: {cmd}"))
                continue
            try:
                code, out = await do_command(conn, cmd, c.get("args"), drive)
            except Exception as ex:
                code, out = 1, f"ошибка выполнения: {ex}"
            reports.put_nowait((cid, code, out))
            if not _alive(conn):  # команда уронила/обнаружила мёртвый канал -> реконнект
                print("[fleet-dog] канал закрыт после команды — переподключаюсь", flush=True)
                dead.set()
                return

    run = {"exe": asyncio.ensure_future(executor())}

    async def super_stop():
        """СТОП мимо очереди: рвём текущую команду, выкидываем ждущие, сразу StopMove."""
        run["exe"].cancel()
        while not jobs.empty():
            jobs.get_nowait()
        drive["v"] = None
        run["exe"] = asyncio.ensure_future(executor())
        await asyncio.wait_for(go2.sport(conn, "StopMove"), timeout=3)

    wd = asyncio.ensure_future(watchdog())
    drv = asyncio.ensure_future(driver(conn, drive))
    rep = asyncio.ensure_future(reporter())
    try:
        await asyncio.wait_for(go2.ensure_normal_mode(conn), timeout=12)
        print("[fleet-dog] конект открыт, режим normal — готов", flush=True)
        while not dead.is_set():
            # ждём команду (long-poll) ИЛИ смерть канала — что раньше
            pull_task = asyncio.ensure_future(pull(server, token, card, POLL_WAIT))
            dead_task = asyncio.ensure_future(dead.wait())
            await asyncio.wait({pull_task, dead_task}, return_when=asyncio.FIRST_COMPLETED)
            if dead.is_set():
                pull_task.cancel()
                print("[fleet-dog] канал закрыт (обрыв/забрал другой клиент) — переподключаюсь", flush=True)
                return
            dead_task.cancel()
            p = pull_task.result()  # HTTP-ошибка пробросится -> внешний цикл переподключит
            cmds = p.get("commands") or []
            last_stop = max((i for i, c in enumerate(cmds) if c.get("cmd") == "stop"), default=-1)
            if last_stop >= 0:  # СТОП — первым делом, до ключа/камеры; всё до него отменено
                try:
                    await super_stop()
                    code, out = 0, "StopMove ok, очередь сброшена"
                except Exception as ex:
                    code, out = 1, f"ошибка СТОПа: {ex}"
                for c in cmds[:last_stop + 1]:
                    res = (code, out) if c.get("cmd") == "stop" else (EXIT_CANCELLED, "отменено СТОПом")
                    reports.put_nowait((c.get("id"), *res))
                cmds = cmds[last_stop + 1:]
            nk = (p.get("aes_key") or "").strip()
            if nk and nk != key:  # ключ сменили в дашборде — переоткрыть конект с новым
                write_key(nk)
                print("[fleet-dog] ключ сменился — переподключаюсь", flush=True)
                return
            for c in cmds:
                jobs.put_nowait(c)
            _camera_set(conn, cam, bool(p.get("stream")))  # вкл/выкл камеру по «смотрю» из дашборда
            await _lidar_set(conn, lid, bool(p.get("lidar")))  # вкл/выкл лидар по «смотрю»
            if bat["soc"] is not None and time.monotonic() - bat["t"] > 10:  # заряд → сервер, опрос не ждёт
                bat["t"] = time.monotonic()
                spawn(_quiet(_post_battery, server, token, card, bat["soc"]))
    finally:
        wd.cancel()
        drv.cancel()
        rep.cancel()
        run["exe"].cancel()
        cam_task.cancel()
        lid_task.cancel()
        if drive["v"]:  # уходим посреди езды (смена ключа/обрыв) — пробуем остановить
            try:
                await asyncio.wait_for(go2.sport(conn, "StopMove"), timeout=1)
            except Exception:
                pass
        try:
            conn.video.switchVideoChannel(False)
        except Exception:
            pass
        await go2.disconnect(conn)


async def main_async():
    env = load_env()
    server = env.get("SERVER_URL", "").rstrip("/")
    token = env.get("FLEET_TOKEN", "")
    card = os.uname().nodename
    if not server or not token:
        print("[fleet-dog] нет SERVER_URL/FLEET_TOKEN в /etc/pi-fleet/fleet.env — стоп", flush=True)
        return
    while True:  # держим агента живым: разрыв конекта/сети -> переподключение
        try:
            await session(server, token, card)
        except Exception as ex:
            print(f"[fleet-dog] сессия оборвалась: {ex} — переподключаюсь", file=sys.stderr, flush=True)
            await asyncio.sleep(3)


if __name__ == "__main__":
    asyncio.run(main_async())
