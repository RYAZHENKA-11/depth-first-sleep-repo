# go2_controller

Контроллер миссии полуфинала: робот стартует, проходит первое препятствие на трассе и
возвращается на старт (он же финиш). На вход — `/odom` и `/scan` от `go2_odometry`,
на выход — `/cmd_vel` для `go2_webrtc_bridge`.

Требования к входным данным — в корневом [README](../../README.md#требования-к-scan-и-odom).

## Статус

| | |
|---|---|
| Готово | автомат миссии, локальный планировщик с объездом, ROS 2 узел, launch, параметры |
| Проверено | 65 тестов (`colcon test`, ROS 2 Lyrical); виртуальный Go2 в RViz: столб, бокс, коридор — проходит без столкновений; глухая стена — 3 попытки, затем `ABORT` без столкновений; цепочка «эмулятор моста → одометрия → контроллер» |
| Не проверено | малинка и реальный робот; скорости и дистанции на железе |
| Ждёт | `/odom` и `/scan` от `go2_odometry` — без них контроллер стоит и ждёт данных |

## Как устроено

Логика отделена от ROS: ядро — чистый Python + numpy, его можно гонять в тестах и
симуляторе без ROS.

| Файл | Что делает |
|---|---|
| `go2_controller/core/geometry.py` | поза, пеленг, перевод координат, система координат трассы (s — вдоль, lat — поперёк, начало в позе старта) |
| `go2_controller/core/scan_utils.py` | скан → точки в `base_link`; свободная дистанция «коридора» шириной с робота в любом направлении |
| `go2_controller/core/local_planner.py` | езда к точке с объездом: выбор направления, профиль скорости, ограничение ускорений, проверка места для разворота |
| `go2_controller/core/mission.py` | автомат миссии (ниже) |
| `go2_controller/controller_node.py` | ROS 2 узел: подписки, таймер 20 Гц, сервисы, отладочные топики, проверки входных данных |
| `config/controller.yaml` | основные параметры |
| `launch/controller.launch.py` | запуск узла |
| `test/` | тесты ядра; `synthetic.py` прогоняет планировщик и миссию на модели робота из `go2_sim` |

Решения взяты из решения 1-го этапа (`helper/task-3`, `nav/fallback.py`): поиск свободного
направления, сглаживание дистанции впереди, скорости «крейсер / медленно / стоп»,
выход из тупика задним ходом, детект застревания.

## Автомат миссии

```
IDLE → INIT → OUTBOUND → AVOID → TURNAROUND → RETURN → ARRIVED
                  └────── RECOVERY ──────┘           ABORT
```

| Состояние | Что происходит |
|---|---|
| `IDLE` | ждёт команды старта |
| `INIT` | запоминает позу старта: ось трассы = направление носа; пауза 1 с |
| `OUTBOUND` | едет по оси трассы. Препятствие = ≥3 точки скана в полосе ±0.5 м от оси не дальше 1.5 м впереди |
| `AVOID` | цель — точка на оси за дальним краем препятствия + 0.8 м; край уточняется по скану во время объезда; сторону объезда выбирает сам |
| `TURNAROUND` | разворот к старту |
| `RETURN` | едет к старту, препятствие объезжает так же |
| `ARRIVED` | ближе 0.3 м к старту — стоп |
| `RECOVERY` | 3 с подряд нет пути или 8 с нет продвижения → назад 1.2 с (если сзади свободно) → попытка с другой стороны |
| `ABORT` | больше 3 восстановлений подряд, таймаут 180 с или команда `stop` |

Если на первых 8 м препятствия нет — разворачивается и возвращается.

## Запуск на роботе

Собирать на малинке без симулятора (он тянет RViz):

```bash
colcon build --packages-skip go2_sim
source install/setup.bash
```

```bash
ros2 launch go2_controller controller.launch.py                  # ждёт команды старта
ros2 launch go2_controller controller.launch.py dry_run:=true    # команды уходят в /go2_controller/cmd_vel_dry, робот не едет
ros2 launch go2_controller controller.launch.py params_file:=/путь/к/своему.yaml
```

Миссия **не стартует сама**. Робот ставится на старт носом вдоль трассы, затем:

```bash
ros2 service call /go2_controller/start std_srvs/srv/Trigger    # старт
ros2 service call /go2_controller/stop  std_srvs/srv/Trigger    # аварийная остановка
ros2 topic echo /go2_controller/state --field data              # что происходит
```

Пока миссия не активна, `/cmd_vel` не публикуется — watchdog моста через 0.5 с отправляет
роботу `StopMove`. Если `/odom` или `/scan` старше 0.5 с, контроллер выдаёт ноль и ждёт.

## Интерфейс

| Имя | Тип | |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | вход |
| `/scan` | `sensor_msgs/LaserScan` | вход |
| `/cmd_vel` | `geometry_msgs/Twist` | выход, 20 Гц, только пока миссия активна |
| `/go2_controller/state` | `std_msgs/String` (JSON) | состояние, причина, цель, края препятствия, статус планировщика |
| `/go2_controller/markers` | `visualization_msgs/MarkerArray` | для RViz: старт, ось, финиш, края препятствия, цель, текст состояния |
| `/go2_controller/start`, `/go2_controller/stop` | `std_srvs/Trigger` | управление миссией |

В лог пишутся предупреждения, если `frame_id` скана не `base_link` или `/odom` скачет
больше чем на 0.3 м между сообщениями.

## Параметры

Основные — в [config/controller.yaml](config/controller.yaml); полный список —
`ros2 param list /go2_controller` (группы `mission.*` и `planner.*`).

| Параметр | По умолчанию | Смысл |
|---|---|---|
| `planner.cruise_vx` / `slow_vx` | 0.35 / 0.15 м/с | скорость на свободном участке / у препятствия |
| `planner.max_wz` | 0.8 рад/с | предел поворота |
| `planner.stop_clearance` | 0.25 м | стоп, когда от носа до препятствия меньше |
| `planner.side_clearance` | 0.15 м | запас сбоку при объезде |
| `planner.turn_clearance` | 0.08 м | запас при развороте на месте; на роботе поднять до ~0.15 |
| `mission.detect_dist` | 1.5 м | дальность обнаружения препятствия |
| `mission.pass_margin` | 0.8 м | насколько уйти за препятствие перед разворотом |
| `mission.outbound_max_dist` | 8.0 м | дальше без препятствия не едем |
| `mission.side_hint` | 0 | сторона объезда: 0 авто, 1 слева, −1 справа |
| `mission.mission_timeout_s` | 180 с | общий лимит времени |

Габариты Go2: `planner.robot_half_length` 0.35 м, `planner.robot_half_width` 0.16 м.

## Проверка в RViz (виртуальный Go2)

Нужен только Docker. Из корня репозитория:

```bash
src/go2_sim/docker/run_sim.sh                     # столб на трассе, миссия стартует сама
src/go2_sim/docker/run_sim.sh world:=corridor     # ещё: box, wall_across
src/go2_sim/docker/run_sim.sh mode:=bridge        # симулятор как настоящий мост — для go2_odometry
```

Открыть http://localhost:6080/vnc.html?autoconnect=true&resize=scale. Подробнее —
в корневом README, раздел «Виртуальный Go2 в RViz».

## Тесты

```bash
cd src/go2_controller && python3 -m pytest -q test       # 47 тестов, без ROS
```

Полный прогон как на малинке (ROS 2 Lyrical в Docker, оба пакета, 65 тестов):

```bash
docker run --rm -e HEADLESS=1 -v "$PWD":/ws:ro go2-sim-lyrical bash -c '
  cd /tmp && colcon --log-base /tmp/log test --base-paths /ws/src/go2_controller /ws/src/go2_sim \
    --build-base /tmp/build --install-base /tmp/install --pytest-args -p no:cacheprovider > /tmp/t.log 2>&1
  colcon test-result --test-result-base /tmp/build --all | tail -1'
```

## Ограничения

- На малинке и реальном роботе ещё не запускался.
- Рельеф не обрабатывается: возвышенность выше порога фильтра в `go2_odometry` выглядит
  как стена, обрыв — как свободный путь. Наклон корпуса (IMU) не контролируется.
- `RECOVERY` включает задний ход скачком, без плавного разгона.

## Дальше

1. Связка с `go2_odometry` в симуляторе (`mode:=bridge`), затем на бэгах в `dry_run`.
2. Сторож наклона по IMU.
3. Первые прогоны на роботе на малой скорости, подбор параметров.
