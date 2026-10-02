# Способ B — управление собакой через ROS 2 (для участников)

Это **второй способ** управления (первый, Python/WebRTC, — в [INSTRUCTION.md](INSTRUCTION.md)).
Собака и команды те же, отличается только интерфейс: здесь — стандартные **ROS 2** топики.

> Нужны **сенсоры** (лидар, камера, IMU)? Этот мост их не публикует. Быстрее — читать их по WebRTC (см. [INSTRUCTION.md](INSTRUCTION.md)). Нужны именно ROS-топики (`/point_cloud2` и т.п.) — [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) с `CONN_TYPE=webrtc`.

Как зайти на малинку по SSH — в [INSTRUCTION.md](INSTRUCTION.md). Дальше всё делается на малинке.

Есть готовый мост: команды из ROS-топиков идут на робота, а состояние робота приходит обратно
в ROS-топик. Ставится на малинке за ~10 минут (один раз).

Что получишь:

| Топик | Тип | Направление |
|---|---|---|
| `/go2/sport_cmd` | `std_msgs/String` | ты → робот: `sit`, `stand_up`, `hello`, … |
| `/cmd_vel` | `geometry_msgs/Twist` | ты → робот: движение (`linear.x/y`, `angular.z`) |
| `/joint_states` | `sensor_msgs/JointState` | робот → ты: 12 углов суставов |

---

## 1. Установка (один раз)
На малинке (там уже есть стек управления в `~/ai-robot/`):
```bash
bash ~/ai-robot/ros2-setup.sh
```
Поставит ROS 2 Lyrical и зависимости. В конце напишет `ГОТОВО ✅`.

## 2. Освободи управление
Робот принимает **одного клиента** за раз. Останови штатный агент управления, чтобы мост занял связь:
```bash
sudo systemctl stop fleet-dog
```
(Вернуть потом: `sudo systemctl start fleet-dog`.)

## 3. Запусти мост
```bash
source /opt/ros/lyrical/setup.bash
~/ai-robot/venv/bin/python ~/ai-robot/ros_bridge.py
```
Увидишь `go2_bridge готов: …` — мост подключился к собаке и слушает ROS-топики. Оставь его работать.

## 4. Управляй из ROS 2 (второй терминал)
В новом терминале **сначала** подгрузи ROS, потом командуй:
```bash
source /opt/ros/lyrical/setup.bash

# поза/жест:
ros2 topic pub --once /go2/sport_cmd std_msgs/String "{data: sit}"
ros2 topic pub --once /go2/sport_cmd std_msgs/String "{data: stand_up}"
ros2 topic pub --once /go2/sport_cmd std_msgs/String "{data: hello}"

# движение (едет, пока шлёшь; останови Ctrl+C и командой stop):
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"        # вперёд
ros2 topic pub /cmd_vel geometry_msgs/Twist "{angular: {z: 0.5}}"       # поворот
ros2 topic pub --once /go2/sport_cmd std_msgs/String "{data: stop}"     # стоп

# состояние робота:
ros2 topic echo /joint_states
```

Из своего кода — обычный `rclpy`/`rclcpp`: публикуй в `/go2/sport_cmd` и `/cmd_vel`, подписывайся на `/joint_states`.

**Команды `/go2/sport_cmd`:** `sit`, `rise_sit`, `stand_up`, `stand_down`, `hello`, `stretch`, `wiggle`, `balance`, `recovery`, `damp`, `stop`.

---

## ⚠️ Безопасность
- Робот **на полу**, вокруг **≥ 2 м** свободно, рядом никого. Кнопка стоп под рукой.
- `/cmd_vel` едет сразу — скорости зажаты в безопасные пределы, но всегда останавливай (`stop`).
- Опасных трюков (сальто/прыжки/суставные моменты) в мосте нет — только высокоуровневые команды.

# 🚫🔴 ЗАПРЕЩЁННЫЕ КОМАНДЫ 🔴🚫

> **Сальто, прыжки, стойка на лапах, «бег» (Bound), танцы/трюки, прямое управление моторами — ЗАБЛОКИРОВАНЫ на роботе и ЗАПИСЫВАЮТСЯ. Не пытайся их слать — ни через топики, ни своим кодом.**

⛔ Полный список — в [API.md](API.md).

## Если не работает
| Симптом | Что делать |
|---|---|
| Мост пишет «WebRTC не подключился» / «занят» | Робота держит другой клиент. Сделал `sudo systemctl stop fleet-dog`? Приложение Unitree закрыто? |
| `ros2: command not found` | Не подгрузил ROS: `source /opt/ros/lyrical/setup.bash`. |
| `ModuleNotFoundError` при запуске моста | Перезапусти `bash ~/ai-robot/ros2-setup.sh` (доставит зависимости). |
| Пусто в `/joint_states` | Робот включён и на связи? Мост подключился (`go2_bridge готов`)? |

## Чего в мосте нет
- **Сенсоры** (лидар, камера, IMU, одометрия) — этот мост их не публикует. Быстрее всего читать по
  WebRTC (см. [INSTRUCTION.md](INSTRUCTION.md)); нужны ROS-топики (`/point_cloud2`, `/scan`, `/imu` …)
  — [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) с `CONN_TYPE=webrtc`.
- Прямого управления **моментами отдельных суставов** (`lowcmd`) — недоступно через этот канал.

Всё высокоуровневое (позы, ходьба по `/cmd_vel`) и чтение состояния — работает.
