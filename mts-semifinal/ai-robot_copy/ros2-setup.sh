#!/bin/bash
# Ставит ROS 2 Lyrical + зависимости для моста ros_bridge.py. Запускать ОДИН раз на малинке:
#   bash ~/ai-robot/ros2-setup.sh
# Идемпотентно. ~10 минут (в основном скачивание ROS).
set -e
. /etc/os-release

if [ ! -f /opt/ros/lyrical/setup.bash ]; then
  echo "[ros2-setup] подключаю репозиторий ROS 2 ..."
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y curl gnupg
  RAS=$(curl -sSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest 2>/dev/null | grep -oE "[0-9]+\.[0-9]+\.[0-9]+" | head -1)
  [ -n "$RAS" ] || RAS=1.3.0
  curl -fL -o /tmp/ros2-apt.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${RAS}/ros2-apt-source_${RAS}.${VERSION_CODENAME}_all.deb"
  sudo apt-get install -y /tmp/ros2-apt.deb
  echo "[ros2-setup] ставлю ros-lyrical-ros-base (долго) ..."
  sudo apt-get update
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y ros-lyrical-ros-base
else
  echo "[ros2-setup] ROS 2 уже стоит"
fi

# python-зависимости ROS в наш venv (venv изолирован → доставляем то, что нужно rclpy)
echo "[ros2-setup] доставляю python-зависимости в venv ..."
~/ai-robot/venv/bin/pip install -q empy lark catkin_pkg pyyaml typeguard packaging

# проверка: rclpy + go2 в одном питоне (из папки ai-robot — там лежит go2.py)
source /opt/ros/lyrical/setup.bash
cd ~/ai-robot
if ./venv/bin/python -c "import rclpy, go2; from sensor_msgs.msg import JointState" 2>/dev/null; then
  echo "[ros2-setup] ГОТОВО ✅  запускай мост:"
  echo "  source /opt/ros/lyrical/setup.bash && ~/ai-robot/venv/bin/python ~/ai-robot/ros_bridge.py"
else
  echo "[ros2-setup] что-то не импортится — проверь вывод:"
  ./venv/bin/python -c "import rclpy, go2; from sensor_msgs.msg import JointState" || true
fi
