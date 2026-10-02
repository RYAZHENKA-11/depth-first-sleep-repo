# Go2 Semifinal — команда depth-first-sleep

Цель плана-минимум: реальный Unitree Go2 стартует, обнаруживает первое препятствие,
обходит его и возвращается на финиш. **Финиш = точка старта.**

## Кто где пишет

| Папка | Владелец | Что внутри |
|---|---|---|
| `src/go2_controller/` | **контроллер** | автомат миссии, обход препятствия, возврат на старт → `/cmd_vel` |
| `src/go2_odometry/` | **одометрия** | TF `odom→base_link`, `/odom`, облако лидара → `/scan` |
| `src/go2_sim/` | **контроллер** | виртуальный Go2 для RViz, принимает `/cmd_vel`. Режим `contract`: сам публикует `/odom`, `/scan`, TF. Режим `bridge`: публикует `/go2/*` как настоящий мост |
| `analysis/` | **аналитика бэгов** | скрипты и выводы по записанным бэгам |
| `src/go2_bringup/` | общее | launch всего стека и общие параметры |
| `src/go2_webrtc_bridge/` | общее | ROS2-мост WebRTC ↔ Go2 (уже работает) |
| `helper/task-2`, `helper/task-3` | только чтение | решения 1-го этапа (Webots), источник кода для переноса |
| `mts-semifinal/ai-robot_copy/` | только чтение | комплект организаторов (`~/ai-robot` на малинке) |

Правило: **в чужую папку не коммитим.** Нужна правка у соседа — пишем ему или открываем PR,
который он ревьюит. Общие папки меняем только через PR с ревью второго участника.

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

### Требования к `/scan` и `/odom`

Контроллер не проверяет эти условия сам: нарушение не роняет узел, а молча портит поведение.
Узел пишет предупреждение в лог только про `frame_id` скана и скачки `/odom`.

| Требование | Если нарушить |
|---|---|
| `/scan`: `header.frame_id = base_link`, дальности **от центра робота** (TF не используется) | все препятствия сдвинуты на смещение лидара (~0.2 м вперёд) |
| `/scan`: убраны точки **корпуса и пола** | точка ближе 0.46 м запрещает разворот, ближе 0.6 м впереди — стоп; робот встаёт навсегда |
| `/scan`: «свободно» = `inf` или значение ≥ `range_max`; `NaN` и `< range_min` отбрасываются | — |
| `/scan`: любые `angle_min` / `angle_increment`, лучше полный круг 360° | без задних лучей отъезд назад в recovery идёт вслепую |
| `/scan` ≥ 5 Гц, `/odom` ≥ 10 Гц, задержка < 0.5 с | данные старше 0.5 с — контроллер выдаёт ноль и ждёт |
| `/odom`: поза `odom → base_link` с полным кватернионом, **без скачков** (никакой релокализации во время миссии) | поза старта и цели фиксируются в `odom`, скачок сдвигает финиш |
| QoS: любой (контроллер подписан как best effort) | — |

Изменение контракта (новый топик, другой frame, другая семантика) — только по договорённости
всех троих и с правкой этой таблицы в том же PR.

## Ветки

- `main` — всегда собирается; прямые пуши запрещены, только через PR.
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

## Виртуальный Go2 в RViz

Нужен только Docker. Собирает пакеты и поднимает виртуального Go2, контроллер и RViz;
RViz открывается в браузере.

```bash
src/go2_sim/docker/run_sim.sh                      # мир pole, миссия стартует сама
src/go2_sim/docker/run_sim.sh world:=corridor      # другие миры: box, corridor, wall_across
src/go2_sim/docker/run_sim.sh controller:=false    # только робот: можно гнать свой /cmd_vel
```

Открыть: http://localhost:6080/vnc.html?autoconnect=true&resize=scale. Миры лежат в
`src/go2_sim/worlds/*.yaml`: круги `[x, y, r]` или `[x, y, r, высота]`, боксы `[x0, y0, x1, y1]`
или `[x0, y0, x1, y1, высота]` (по умолчанию 0.8 м), старт `[x, y, yaw]`.

### Режим эмуляции моста (`mode:=bridge`) — для отладки `go2_odometry`

```bash
src/go2_sim/docker/run_sim.sh mode:=bridge
```

Симулятор публикует то же, что `go2_webrtc_bridge` на реальном роботе, и **не** публикует
`/odom`, `/scan`, TF `odom → base_link` — их должны дать узлы `go2_odometry`:

| Топик | Частота | Как в симуляторе |
|---|---|---|
| `/go2/odom/sport_lf` | 20 Гц | одометрия по ногам **с дрейфом**: +3 % пути, курс уходит на 0.003 рад/с |
| `/go2/odom/robot_pose` | 20 Гц | лидарная поза: шум 1 см, без twist |
| `/go2/imu/data` | 20 Гц | `imu_link`, курс из одометрии по ногам |
| `/go2/lidar/points` | 4 Гц | воксельная карта 0.05 м, окно 6.4×6.4 м вокруг робота, frame `odom`, **с полом** (z = 0) |

Пока узлов одометрии нет, контроллер ждёт данных, модель собаки в RViz не рисуется (нет TF);
истинное положение робота видно голубым прямоугольником (`/fake_go2/truth`). Свои узлы
запускаются внутри того же контейнера:

```bash
docker exec -it go2_sim bash
source /opt/ros/lyrical/setup.bash && source /tmp/install/setup.bash
ros2 run go2_odometry <твой_узел>      # пакет пересобирается при каждом run_sim.sh
```

Параметры эмуляции (`odom_scale_error`, `odom_yaw_bias`, `pose_noise`, `cloud_rate`,
`cloud_floor`) — у узла `fake_go2`.

## Что не попадает в git

Бэги и архивы (`*.7z`, `*.db3`, `*.mcap`, `bags/`), логи, `build/ install/ log/`, venv,
настройки IDE, ключи и `.ovpn`. Бэги передаём отдельно, не через репозиторий.
Подключение к роботу: `src/go2_webrtc_bridge/ROBOT_CONNECT_PLAN.md`.
