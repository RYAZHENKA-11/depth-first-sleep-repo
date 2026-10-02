# Go2 Semifinal — команда depth-first-sleep

Цель плана-минимум: реальный Unitree Go2 стартует, обнаруживает первое препятствие,
обходит его и возвращается на финиш. **Финиш = точка старта.**

## Кто где пишет

| Папка | Владелец | Что внутри |
|---|---|---|
| `src/go2_controller/` | **контроллер** | автомат миссии, обход препятствия, возврат на старт → `/cmd_vel` |
| `src/go2_odometry/` | **одометрия** | TF `odom→base_link`, `/odom`, облако лидара → `/scan` |
| `analysis/` | **аналитика бэгов** | скрипты и выводы по записанным бэгам |
| `src/go2_bringup/` | общее | launch всего стека и общие параметры |
| `src/go2_webrtc_bridge/` | общее | ROS2-мост WebRTC ↔ Go2 (уже работает) |
| `helper/task-2`, `helper/task-3` | только чтение | решения 1-го этапа (Webots), источник кода для переноса |
| `mts-semifinal/ai-robot_copy/` | только чтение | комплект организаторов (`~/ai-robot` на малинке) |

Правило: **в чужую папку не коммитим.** Нужна правка у соседа — пишем ему или открываем MR,
который он ревьюит. Общие папки меняем только через MR с ревью второго участника.

## Контракт между пакетами

Пакеты общаются только через эти топики. Пока контракт соблюдён, каждый может переписывать
своё внутри как угодно.

| Топик / TF | Тип | Публикует | Читает |
|---|---|---|---|
| `/go2/odom/sport_lf`, `/go2/odom/robot_pose` | `nav_msgs/Odometry` | bridge | odometry |
| `/go2/imu/data` | `sensor_msgs/Imu` | bridge | odometry |
| `/go2/lidar/points` | `sensor_msgs/PointCloud2` (frame `odom`) | bridge | odometry |
| TF `odom → base_link` | — | odometry | controller, rviz |
| `/odom` | `nav_msgs/Odometry` (`odom` / `base_link`) | odometry | controller |
| `/scan` | `sensor_msgs/LaserScan` (frame `base_link`, 360°, `inf` = чисто) | odometry | controller |
| `/cmd_vel` | `geometry_msgs/Twist` | controller | bridge (`enable_cmd_vel:=true`) |

Лимиты на первых прогонах: `|vx| ≤ 0.4 м/с`, `|wz| ≤ 0.8 рад/с`. Аппаратные пределы робота:
x ±0.6, y ±0.5, z ±1.0.

Изменение контракта (новый топик, другой frame, другая семантика) — только по договорённости
всех троих и с правкой этой таблицы в том же MR.

## Ветки

- `main` — всегда собирается; прямые пуши запрещены, только через MR.
- Свои ветки с префиксом зоны: `controller/<задача>`, `odometry/<задача>`, `analysis/<задача>`.
- Общие правки: `shared/<задача>`.

## Сборка

Корень репозитория — colcon workspace:

```bash
source /opt/ros/<дистрибутив>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Только свой пакет: `colcon build --packages-select go2_controller` (или `go2_odometry`).

## Что не попадает в git

Бэги и архивы (`*.7z`, `*.db3`, `*.mcap`, `bags/`), логи, `build/ install/ log/`, venv,
настройки IDE, ключи и `.ovpn`. Бэги передаём отдельно, не через репозиторий.
Подключение к роботу: `src/go2_webrtc_bridge/ROBOT_CONNECT_PLAN.md`.
