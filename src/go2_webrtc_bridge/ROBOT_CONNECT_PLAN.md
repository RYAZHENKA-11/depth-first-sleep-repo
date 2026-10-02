# План подключения к роботу Go2 и работы с go2_webrtc_bridge

Этот документ описывает **полный порядок действий** с момента подключения к роботу по SSH/SFTP, включая работу с текущим ROS2-мостом (`go2_webrtc_bridge`). Он дополнает инструкции от организаторов (`INSTRUCTION.md`, `ROS2.md`, `API.md`).

> Важное правило: **один клиент к роботу за раз**. Если запущен штатный агент `fleet-dog`, нужно его остановить перед запуском моста. После работы — при необходимости вернуть обратно.

## Содержание

1. [0. Предварительные требования](#0-предварительные-требования)
2. [1. Подключение к VPN (OpenVPN)](#1-подключение-к-vpn-openvpn)
3. [2. Подключение по SSH](#2-подключение-по-ssh)
4. [3. Структура файлов на роботе/малинке](#3-структура-файлов-на-роботемалинке)
5. [4. Освобождение управления (обязательно перед мостом)](#4-освобождение-управления-обязательно-перед-мостом)
6. [5. Клонирование/обновление репозитория go2_webrtc_bridge](#5-клонированиеобновление-репозитория-go2_webrtc_bridge)
7. [6. Установка зависимостей](#6-установка-зависимостей)
8. [7. Конфигурация моста](#7-конфигурация-моста)
9. [8. Запуск go2_webrtc_bridge (ROS 2)](#8-запуск-go2_webrtc_bridge-ros-2)
10. [9. Проверка работы (тестовые команды)](#9-проверка-работы-тестовые-команды)
11. [10. Запись данных в rosbag (рекомендуется для съёмки всего)](#10-запись-данных-в-rosbag-рекомендуется-для-съёмки-всего)
12. [11. Работа по SFTP (копирование файлов)](#11-работа-по-sftp-копирование-файлов)
13. [12. Возврат штатного управления](#12-возврат-штатного-управления)
14. [13. Чек-лист перед/после работы](#13-чек-лист-передпосле-работы)
15. [14. Диагностика и типичные проблемы](#14-диагностика-и-типичные-проблемы)
16. [15. Команды «одной строкой» (шпаргалка)](#15-команды-одной-строкой-шпаргалка)

---

## 0. Предварительные требования

- Конфиг OpenVPN `.ovpn` от организаторов
- SSH-ключ для подключения к малинке (ubuntu)
- IP-адрес малинки `<IP>` из сообщения организаторов
- Рабочая станция с ROS 2 Humble/Iron/Lyrical (или запуск моста **на самой малинке** — там установлен Lyrical). Мост можно собирать/запускать как на малинке, так и на отдельной машине в той же VPN-сети (адрес робота тот же).

> **Важно:** никогда не запускайте два VPN одновременно — будет конфликт.

---

## 1. Подключение к VPN (OpenVPN)

Организаторы выдают конфиг `.ovpn`. Подключитесь к VPN **перед** SSH.

| ОС | Клиент | Как подключить |
|---|---|---|
| macOS | Tunnelblick / OpenVPN Connect | Перетащите `.ovpn` → Connect |
| Windows | OpenVPN Connect / OpenVPN GUI | Импорт `.ovpn` → Connect |
| Linux (CLI) | `openvpn` | `sudo openvpn --config /path/to/your-config.ovpn` |
| Linux (GUI) | OpenVPN Connect/Gnome Network | Импортировать `.ovpn` |

**Рекомендации:**
- Отключите все другие VPN перед подключением.
- Дождитесь, пока OpenVPN установит маршрут (нет красных ошибок в логе).
- Проверьте доступность малинки (пинг можно выполнить после SSH, либо сразу по IP).

---

## 2. Подключение по SSH

IP и путь к ключу выдаются организаторами. Пользователь `ubuntu`.

```bash
chmod 600 /path/to/private_key
ssh -i /path/to/private_key -o IdentitiesOnly=yes ubuntu@<IP>
```

> Если сразу не подключается — переподключите VPN, проверьте правильность IP.

---

## 3. Структура файлов на роботе/малинке

Организаторы предоставляют `~/ai-robot/` (штатный комплект). Этот репозиторий (`go2_webrtc_bridge`) — отдельный ROS 2 мост, который можно разместить как на малинке, так и на рабочей машине в той же VPN-сети.

**Штатные файлы (`~/ai-robot/`, справочно):**
| Файл | Назначение |
|---|---|
| `01_read_state.py`, `02_battery.py`, `03_command.py` | Быстрые проверки/управление по WebRTC (Python) |
| `go2.py` | Хелпер подключения |
| `ros_bridge.py`, `ros2-setup.sh` | Упрощённый ROS-мост организаторов |
| `INSTRUCTION.md`, `ROS2.md`, `API.md` | Инструкции организаторов |

**Наш мост (`go2_webrtc_bridge/`):** полнокровный ROS 2 мост с типизированными топиками, `/go2/raw/...`, параметрами записи и жёсткой безопасностью по `cmd_vel`.

---

## 4. Освобождение управления (обязательно перед мостом)

Робот принимает **одного клиента** за раз. Перед запуском `go2_webrtc_bridge` остановите штатный агент:

```bash
sudo systemctl stop fleet-dog
```

Вернуть обратно после работы: `sudo systemctl start fleet-dog`.

> Если `go2_webrtc_bridge` не подключается, а агент запущен — Вы получите `RobotBusyError`.

---

## 5. Клонирование/обновление репозитория go2_webrtc_bridge

### Вариант A: на малинке

```bash
cd ~
git clone <URL-репозитория> go2_webrtc_bridge
cd go2_webrtc_bridge
```

### Вариант B: локально, затем SFTP (rsync)

```bash
# на рабочей машине, в локальной копии
rsync -avz --exclude 'backup_code/*.pkl' --exclude '__pycache__' \
  ./go2_webrtc_bridge ubuntu@<IP>:~/go2_webrtc_bridge/

# или scp -r для разовой копии
scp -r go2_webrtc_bridge ubuntu@<IP>:~/
```

> `backup_code/*.pkl` — это ~1 ГБ локальных дампов, их пересылать не нужно.

---

## 6. Установка зависимостей

**На малинке (Lyrical):** выполните один раз `bash ~/ai-robot/ros2-setup.sh` (если ещё не выполняли), затем:

```bash
source /opt/ros/lyrical/setup.bash

# зависимости Python
pip3 install -r requirements.txt
# или с sudo: sudo pip3 install -r requirements.txt

# сборка и установка ROS 2 пакета
cd ~/go2_webrtc_bridge
colcon build --packages-select go2_webrtc_bridge
source install/setup.bash
```

**На рабочей машине с ROS 2:**

```bash
source /opt/ros/<дистрибутив>/setup.bash   # humble / iron / jazzy
pip3 install -r requirements.txt
colcon build --packages-select go2_webrtc_bridge
source install/setup.bash
```

> `unitree_webrtc_connect==2.1.2` — обязательная версия библиотеки. `sensor_msgs_py` больше не нужен (numpy-путь).

---

## 7. Конфигурация моста

Все параметры — в `config/go2_webrtc_bridge.yaml`. Дублировать их вручную не нужно: launch подхватывает из конфига автоматически, если запускать без аргументов.

**Ключевые параметры (значения по умолчанию):**
| Параметр | По умолчанию | Назначение |
|---|---|---|
| `robot_ip` | `192.168.8.181` | IP робота (задайте ваш) |
| `lidar_auto_enable` | `false` | Включать лидар автоматически |
| `imu_source` | `sport` | источник IMU: `sport`, `lowstate` или `both` |
| `publish_full_arrays` | `true` | Полная запись массивов (без обрезки) |
| `exclude_raw_fields` | `["motor_state"]` | Исключить из raw-канала (суставы) |
| `enable_cmd_vel` | `false` | **Оставьте `false` на первых порах** |

> Рекомендую начать с `enable_cmd_vel:=false` и `lidar_auto_enable:=false` — только чтение данных, без движения и без управления лидаром.

---


## 8. Запуск go2_webrtc_bridge (ROS 2)

### Вариант A: из launch-файла (рекомендуется)

```bash
source /opt/ros/lyrical/setup.bash
source ~/go2_webrtc_bridge/install/setup.bash

ros2 launch go2_webrtc_bridge go2_webrtc_bridge.launch.py \
  robot_ip:=<IP-РОБОТА> \
  enable_cmd_vel:=false \
  lidar_auto_enable:=false
```

### Вариант B: напрямую через ros2 run

```bash
source /opt/ros/lyrical/setup.bash
source ~/go2_webrtc_bridge/install/setup.bash

ros2 run go2_webrtc_bridge bridge_node --ros-args \
  -p robot_ip:=<IP-РОБОТА> \
  -p enable_cmd_vel:=false \
  -p lidar_auto_enable:=false
```

> `bridge_node` подключается к роботу автоматически при старте. Проверьте, что в выводе появилось `connected`.

---

## 9. Проверка работы (тестовые команды)

Откройте **второй терминал** с тем же `source`:

```bash
# список топиков
ros2 topic list | grep go2

# IMU
ros2 topic echo /go2/imu/data --once

# Одометрия
ros2 topic echo /go2/odom/sport_lf --once

# Суставы (12 суставов лап)
ros2 topic echo /go2/joint_states --once

# Лидар (PointCloud2)
ros2 topic echo /go2/lidar/points --once | head -20

# Сырые данные (все поля из payload)
ros2 topic echo /go2/raw/rt_lf_lowstate --once
ros2 topic echo /go2/raw/rt_utlidar_voxel_map --once | head -5

# Статус активности топиков
ros2 topic echo /go2/bridge/topic_status --once

# Статус подключения
ros2 topic echo /go2/bridge/connection --once
```

**Управление (позы, без движения):**
```bash
# посадить
ros2 topic pub --once /go2/sport_cmd std_msgs/String "{data: sit}"

# встать
ros2 topic pub --once /go2/sport_cmd std_msgs/String "{data: stand_up}"
```

> **Осторожно:** сначала безопасно вызовите `sit` и `stand_up` (позы без движения). Проверьте `enable_cmd_vel:=false` перед любыми тестами `cmd_vel`.

---

## 10. Запись данных в rosbag (рекомендуется для съёмки всего)

Откройте третий терминал:

```bash
# полная запись всех топиков
ros2 bag record -a -o ~/bags/go2_full_$(date +%Y%m%d_%H%M%S)
```

Или выборочно (исключая потенциально «шумные»):
```bash
ros2 bag record \
  /go2/imu/data \
  /go2/joint_states \
  /go2/odom/sport_lf \
  /go2/odom/robot_pose \
  /go2/lidar/points \
  /go2/lidar/state \
  /go2/raw/rt_lf_lowstate \
  /go2/raw/rt_lf_sportmodestate \
  /go2/raw/rt_utlidar_voxel_map \
  /go2/raw/rt_utlidar_lidar_state \
  /go2/raw/rt_utlidar_robot_pose \
  -o ~/bags/go2_sel_$(date +%Y%m%d_%H%M%S)
```

**Рекомендации по записи:**
- `publish_full_arrays:=true` (по умолчанию) → полные облака лидара в `/go2/raw/rt_utlidar_voxel_map`.
- Объём: **15–25 ГБ/час** (в основном лидар). Убедитесь, что на диске есть место.
- Если места мало: `publish_full_arrays:=false` → **~0.09 ГБ/час** (точки лидара остаются на `/go2/lidar/points` в виде PointCloud2).

**Остановить запись:** `Ctrl+C` в терминале с `ros2 bag record`.

**Скопировать на локальную машину:**
```bash
scp -r ubuntu@<IP>:~/bags/go2_full_20250101_120000 ~/bags/
```

---

## 11. Работа по SFTP (копирование файлов)

### Настройка SFTP-клиента

| ОС | Инструмент | Примечание |
|---|---|---|
| Windows | WinSCP / FileZilla | Укажите SSH-ключ вместо пароля |
| macOS | Cyberduck / ForkLift / `sftp` в терминале | Используйте тот же ключ |
| Linux | `sftp` в терминале / FileZilla | Стандартно |

**Пример (интерактивный `sftp`):**
```bash
sftp -i /path/to/private_key ubuntu@<IP>
# в сессии sftp:
sftp> ls
sftp> cd ~/go2_webrtc_bridge
sftp> ls
sftp> get bridge_node.py          # скачать файл
sftp> put local_file.txt          # загрузить файл
sftp> bye
```

**Синхронизация каталогов через `rsync` по SSH:**
```bash
# на рабочей машине
rsync -avz -e "ssh -i /path/to/key -o IdentitiesOnly=yes" \
  ./config/ ubuntu@<IP>:~/go2_webrtc_bridge/config/

# обратно (с малинки на локальную)
rsync -avz -e "ssh -i /path/to/key -o IdentitiesOnly=yes" \
  ubuntu@<IP>:~/bags/go2_full_20250101/ ~/bags/
```

---

## 12. Возврат штатного управления

После завершения работы с мостом (если нужно вернуть агента):
```bash
sudo systemctl start fleet-dog
```

Остановить запись rosbag: `Ctrl+C` в терминале с `ros2 bag record`.

---


## 13. Чек-лист перед/после работы

**Перед началом:**
- [ ] Отключены другие VPN
- [ ] Наш OpenVPN подключён
- [ ] `sudo systemctl stop fleet-dog` выполнена
- [ ] `enable_cmd_vel:=false` (для первого раза)
- [ ] Робот на полу, ≥2 м свободно, кнопка стоп рядом
- [ ] Место на диске (15–25 ГБ/час при полной записи, 0.09 ГБ/час при `publish_full_arrays:=false`)

**После работы:**
- [ ] `Ctrl+C` в терминале с `ros2 bag record`
- [ ] `ros2 bag info ~/bags/go2_full_...` — проверить, что запись непустая
- [ ] Мост остановлен (`Ctrl+C`)
- [ ] `sudo systemctl start fleet-dog` (если нужен штатный агент)
- [ ] Скопировать rosbag на локальную машину через SFTP/rsync
- [ ] Отключить VPN (по желанию)

---

## 14. Диагностика и типичные проблемы

| Симптом | Причина | Что делать |
|---|---|---|
| `Permission denied (publickey)` | Права на ключ или не тот ключ | `chmod 600 ключ`, `-o IdentitiesOnly=yes` |
| Не пингуется `<IP>` | VPN не подключён или конфликт двух VPN | Отключите другие VPN, переподключите наш |
| `RobotBusyError` при старте | Другой клиент держит соединение | `sudo systemctl stop fleet-dog`, закройте Unitree-приложение, подождите 15 сек |
| `ros2: command not found` | Не подгружен ROS | `source /opt/ros/lyrical/setup.bash` |
| `ModuleNotFoundError: unitree_webrtc_connect` | Не установлена библиотека | `pip3 install -r requirements.txt` (нужна версия 2.1.2) |
| `ModuleNotFoundError: rclpy` | Не подгружен ROS или не тот Python | `source /opt/ros/...`, используйте `python3` из ROS-окружения |
| `PackageNotFoundError: go2_webrtc_bridge` | Не собран или не подгружен `install/` | `colcon build` в каталоге пакета, затем `source install/setup.bash` |
| `InvalidParameterTypeException` | Строка вместо bool в override | Не передавайте `:=false` без кавычек через `ros2 run`; используйте launch-файл |
| Пусто на `/go2/imu/data` | Мост не подключён или `imu_source` не тот | `ros2 topic echo /go2/bridge/connection`; проверьте `imu_source` |
| `/go2/lidar/points` пуст | `lidar_auto_enable:=false` и лидар выключен | Включите лидар на роботе или `lidar_auto_enable:=true` |
| `ros2 bag record` тормозит / падает | Облака лидара очень большие (1.46 МБ JSON) | `publish_full_arrays:=false` — точки останутся на `/go2/lidar/points` |
| Диск заполняется быстро | 15–25 ГБ/час при полной записи | `publish_full_arrays:=false` → ~0.09 ГБ/час |
| Данные не появляются | Робот не включён / нет питания | Проверьте питание и `ros2 topic echo /go2/bridge/connection` |

---

## 15. Команды «одной строкой» (шпаргалка)

```bash
# Подключение
sudo openvpn --config ~/Downloads/go2.ovpn &
chmod 600 ~/Downloads/robot_key
ssh -i ~/Downloads/robot_key -o IdentitiesOnly=yes ubuntu@<IP>

# Освободить управление
sudo systemctl stop fleet-dog

# Установка зависимостей (один раз)
source /opt/ros/lyrical/setup.bash
pip3 install -r ~/go2_webrtc_bridge/requirements.txt
cd ~/go2_webrtc_bridge && colcon build --packages-select go2_webrtc_bridge
source install/setup.bash

# Запуск моста
ros2 launch go2_webrtc_bridge go2_webrtc_bridge.launch.py \
  robot_ip:=<IP> enable_cmd_vel:=false lidar_auto_enable:=false

# Запись данных
ros2 bag record -a -o ~/bags/run_$(date +%Y%m%d_%H%M%S)

# Вернуть управление
sudo systemctl start fleet-dog
```

---

## Приложение: полный список топиков моста

Сгенерировано из `bridge_node.py`, не вручную — чтобы имена не расходились с кодом.

### Типизированные и сервисные топики

| топик | тип | направление | примечание |
| --- | --- | --- | --- |
| `/go2/imu/data` | `sensor_msgs/Imu` | робот → ROS | imu_source=sport (по умолчанию) |
| `/go2/joint_states` | `sensor_msgs/JointState` | робот → ROS | 12 суставов лап, без скоростей |
| `/go2/odom/sport_lf` | `nav_msgs/Odometry` | робот → ROS | twist в базовом фрейме |
| `/go2/odom/sport` | `nav_msgs/Odometry` | робот → ROS | из rt/sportmodestate |
| `/go2/odom/robot_pose` | `nav_msgs/Odometry` | робот → ROS | из rt/utlidar/robot_pose |
| `/go2/odom/lio_sam` | `nav_msgs/Odometry` | робот → ROS | молчит без включённого LIO-SAM |
| `/go2/odom/uslam_mapping` | `nav_msgs/Odometry` | робот → ROS | молчит без включённого uSLAM |
| `/go2/odom/uslam_localization` | `nav_msgs/Odometry` | робот → ROS | молчит без локализации |
| `/go2/lidar/points` | `sensor_msgs/PointCloud2` | робот → ROS | float32 xyz, фрейм odom |
| `/go2/lidar/state` | `std_msgs/String` | робот → ROS | 17 полей, imu_rpy в градусах |
| `/go2/bridge/topic_status` | `std_msgs/String` | мост → ROS | счётчики по каждому топику |
| `/go2/bridge/connection` | `std_msgs/String` | мост → ROS | connected / disconnected |
| `/go2/sport_cmd` | `std_msgs/String` | ROS → робот | sit, stand_up, hello, stop, … |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → робот | ТОЛЬКО при enable_cmd_vel:=true |
| `/cmd_vel_stamped` | `geometry_msgs/TwistStamped` | ROS → робот | ТОЛЬКО при enable_cmd_vel:=true |
| `/go2/lidar/set_enabled` | `std_srvs/SetBool` | ROS → робот | вкл/выкл лидар |
| `/go2/uslam/command` | `std_msgs/String` | ROS → робот | карта / локализация / стоп |
| `/go2/legacy_slam/command` | `std_msgs/String` | ROS → робот | старый SLAM-интерфейс |

| топик | WebRTC-топик | формат |
| --- | --- | --- |
| `/go2/raw/rt_lf_lowstate` | `rt/lf/lowstate` | `String` (JSON всего payload) |
| `/go2/raw/rt_lf_sportmodestate` | `rt/lf/sportmodestate` | `String` (JSON всего payload) |
| `/go2/raw/rt_sportmodestate` | `rt/sportmodestate` | `String` (JSON всего payload) |
| `/go2/raw/rt_utlidar_voxel_map` | `rt/utlidar/voxel_map` | `String` (JSON всего payload) |
| `/go2/raw/rt_utlidar_lidar_state` | `rt/utlidar/lidar_state` | `String` (JSON всего payload) |
| `/go2/raw/rt_utlidar_robot_pose` | `rt/utlidar/robot_pose` | `String` (JSON всего payload) |
| `/go2/raw/rt_lio_sam_ros2_mapping_odometry` | `rt/lio_sam_ros2/mapping/odometry` | `String` (JSON всего payload) |
| `/go2/raw/rt_pctoimage_local_bytes` | `rt/pctoimage_local` | `UInt8MultiArray` (сырой CDR) |
| `/go2/raw/rt_pctoimage_local_meta` | `rt/pctoimage_local` | `String` (метаданные) |
| `/go2/raw/rt_qt_notice` | `rt/qt_notice` | `String` (JSON всего payload) |
| `/go2/raw/rt_uslam_frontend_cloud_world_ds_bytes` | `rt/uslam/frontend/cloud_world_ds` | `UInt8MultiArray` (сырой CDR) |
| `/go2/raw/rt_uslam_frontend_cloud_world_ds_meta` | `rt/uslam/frontend/cloud_world_ds` | `String` (метаданные) |
| `/go2/raw/rt_uslam_frontend_odom` | `rt/uslam/frontend/odom` | `String` (JSON всего payload) |
| `/go2/raw/rt_uslam_server_log` | `rt/uslam/server_log` | `String` (JSON всего payload) |
| `/go2/raw/rt_uslam_localization_cloud_world_bytes` | `rt/uslam/localization/cloud_world` | `UInt8MultiArray` (сырой CDR) |
| `/go2/raw/rt_uslam_localization_cloud_world_meta` | `rt/uslam/localization/cloud_world` | `String` (метаданные) |
| `/go2/raw/rt_uslam_localization_odom` | `rt/uslam/localization/odom` | `String` (JSON всего payload) |
| `/go2/raw/rt_mapping_grid_map_bytes` | `rt/mapping/grid_map` | `UInt8MultiArray` (сырой CDR) |
| `/go2/raw/rt_mapping_grid_map_meta` | `rt/mapping/grid_map` | `String` (метаданные) |

> Часть топиков молчит, пока соответствующий режим не включён на роботе
> (`/go2/odom/lio_sam`, `/go2/odom/uslam_*`). Это намеренно: мост не публикует
> одометрию из нераспознанного payload, потому что «всё нули» неотличимо от
> «робот в начале координат».

### Сырые каналы `/go2/raw/...` (весь payload как есть)

Имена сгенерированы из строки WebRTC-топика: `/` → `_`. Проверить соответствие
можно так:

```bash
ros2 topic list | grep /go2/raw/
```

> У `*_bytes` / `*_meta` пар binary-топиков `_bytes` — это `UInt8MultiArray` с
> неразобранным CDR, `_meta` — метаданные (включая параметры сжатия), по которым
> его можно распаковать офлайн. Такие топики — сырой CDR, а не JSON.

## Ссылки

- [INSTRUCTION.md](INSTRUCTION.md) — инструкции организаторов (подключение, управление, безопасность)
- [ROS2.md](ROS2.md) — управление через ROS 2 (для организаторов)
- [API.md](API.md) — справочник разрешённых/запрещённых команд
- [README.md](README.md) — документация моста `go2_webrtc_bridge` (параметры, топики, надёжность)
