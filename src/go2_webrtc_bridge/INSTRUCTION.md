# Управление робо-собакой (Unitree Go2)

Всё уже настроено — адреса и ключ собаки прописаны, вводить ничего не нужно. Порядок:
подключиться (Шаг 1) → найти файлы (Шаг 2) → управлять (Шаг 3: Python/WebRTC или ROS 2).
Сенсоры (лидар, IMU, поза) читаются по WebRTC — в конце «Способа A».

---

## Шаг 1 — Подключение

### OpenVPN
Малинки в закрытой сети, доступ только через VPN. Организаторы выдают конфиг `.ovpn` —
импортируй его в клиент OpenVPN и подключись:

- macOS: Tunnelblick или OpenVPN Connect → перетащи `.ovpn` → Connect.
- Windows: OpenVPN Connect / GUI → импорт `.ovpn` → Connect.
- Linux: `sudo openvpn --config твой-конфиг.ovpn`

Сначала отключи все другие VPN — два одновременно конфликтуют, до малинки не достучаться.

### SSH
Ключ и IP выдают организаторы (IP — в сообщении):
```bash
ssh -i путь/к/ключу ubuntu@<IP>
```
Пользователь `ubuntu`, вход по ключу. Если `Permission denied` — `chmod 600 ключ` и добавь `-o IdentitiesOnly=yes`.

---

## Шаг 2 — Файлы

Всё в `~/ai-robot/`:

| Файл | Для чего |
|---|---|
| `01_read_state.py` | состояние собаки (ничего не двигает) |
| `02_battery.py` | заряд и температуры |
| `03_command.py` | отправка команд (позы, движение) — WebRTC |
| `go2.py` | хелпер для Python-кода |
| `ros_bridge.py`, `ros2-setup.sh` | мост и установка для ROS 2 |
| `API.md`, `ROS2.md` | справочник команд, гайд по ROS 2 |
| `venv/` | Python-окружение, запуск: `./venv/bin/python <файл>` |

---

## Шаг 3 — Управление

Два интерфейса к одной собаке, пределы безопасности одинаковые:

| | Python (WebRTC) | ROS 2 |
|---|---|---|
| Установка | готово сразу | ~10 мин (`ros2-setup.sh`) |
| Команды | `03_command.py`, хелпер `go2` | топики `/go2/sport_cmd`, `/cmd_vel` |
| Состояние | `01_read_state.py` | топик `/joint_states` |

Собака принимает одного клиента за раз — работай одним способом.

---

### Способ A — Python (WebRTC)

Запуск скриптов: `./venv/bin/python <скрипт>` из `~/ai-robot`. Команды — в [API.md](API.md).

Проверка связи:
```bash
cd ~/ai-robot
./venv/bin/python 01_read_state.py
./venv/bin/python 02_battery.py
```

Команды через `03_command.py` — по умолчанию только показывает, что сделает; `--yes` выполняет:
```bash
./venv/bin/python 03_command.py list
./venv/bin/python 03_command.py sit --yes
./venv/bin/python 03_command.py stand_up --yes
./venv/bin/python 03_command.py move 0.3 0 0 --duration 2 --yes
./venv/bin/python 03_command.py stop --yes
```
`move x y z`: `x` вперёд, `y` вбок (м/с), `z` поворот (рад/с). После `--duration` останавливается сама.

Свой код — через хелпер `go2`:
```python
import asyncio
import go2

async def main():
    conn = await go2.connect()
    try:
        await go2.ensure_normal_mode(conn)
        await go2.sport(conn, "Sit")
        await asyncio.sleep(2)
        await go2.sport(conn, "Move", {"x": 0.3, "y": 0, "z": 0})
        await asyncio.sleep(2)
        await go2.sport(conn, "StopMove")
    finally:
        await go2.disconnect(conn)

asyncio.run(main())
```
`ensure_normal_mode` вызывай перед позами и движением. Имена команд — в [API.md](API.md).

**Сенсоры и телеметрия** — тем же соединением, подпиской на топик:
```python
import asyncio
import go2
from unitree_webrtc_connect.constants import RTC_TOPIC

async def main():
    conn = await go2.connect()
    conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LOW_STATE"], lambda m: print(m.get("data", m)))
    await asyncio.sleep(10)
    await go2.disconnect(conn)

asyncio.run(main())
```

| Топик | Что внутри |
|---|---|
| `rt/lf/lowstate` | IMU, углы/скорости моторов, силы стоп |
| `rt/sportmodestate` | режим/поза, скорость и высота корпуса, положение стоп |
| `rt/utlidar/robot_pose` | поза (одометрия) |
| `rt/utlidar/voxel_map_compressed` | лидар — облако точек |
| `rt/utlidar/lidar_state` | состояние лидара |
| `rt/wirelesscontroller` | пульт |

Полный список имён — `RTC_TOPIC` из `unitree_webrtc_connect.constants`.

Лидар — включи поток и поставь декодер точек:
```python
conn.datachannel.set_decoder("native")
conn.datachannel.pub_sub.subscribe(RTC_TOPIC["ULIDAR_ARRAY"],
    lambda m: print(m["data"]["data"]["points"].shape))
await conn.datachannel.disableTrafficSaving(True)
```
`native` даёт numpy Nx3 в метрах. Без `disableTrafficSaving(True)` лидар не шлётся.

Камера — это видео-канал, не топик: `conn.video.switchVideoChannel(True)` и `conn.video.add_track_callback(cb)`.

---

### Способ B — ROS 2

Топики `/go2/sport_cmd` (позы), `/cmd_vel` (движение), `/joint_states` (состояние). Установка (~10 мин):
```bash
bash ~/ai-robot/ros2-setup.sh
```
Запуск моста, примеры `ros2 topic pub` и работа из кода — в [ROS2.md](ROS2.md).

Сенсоры этот мост не отдаёт. Читай их по WebRTC (выше) или подними
[go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) с `CONN_TYPE=webrtc` (для ROS-топиков `/point_cloud2` и т.п.).

---

## Безопасность

- Собака на полу, вокруг ≥ 2 м свободно, рядом никого, кнопка стоп под рукой.
- `move` едет сразу — всегда останавливай (`stop` или после `--duration`).
- Первые запуски — малая скорость и короткий `--duration`.

## 🚫 Запрещённые команды

**Сальто, прыжки, стойка на лапах, «бег» (Bound), танцы и трюки, прямое управление моторами —
заблокированы на роботе и записываются. Робот их не выполнит, а попытка видна организаторам.**

Полный список — в [API.md](API.md). Не отправляй их никаким способом — ни готовой командой, ни своим кодом.

---

## Если не работает

| Симптом | Что делать |
|---|---|
| Не заходит по SSH / не пингуется | Отключи все другие VPN и перезапусти наш OpenVPN. Проверь IP, `chmod 600 ключ`, `-o IdentitiesOnly=yes`. |
| Собака занята (busy) | Подключён другой клиент (скрипт, приложение, другой способ). Подожди или спроси организаторов. |
| Ошибка про связь или ключ собаки | Сообщи организаторам. |
| Поза не слушается | Вызови `go2.ensure_normal_mode(conn)` перед командой. |
| Ошибка сразу после прошлого запуска | Подожди ~15 секунд между подключениями. |
