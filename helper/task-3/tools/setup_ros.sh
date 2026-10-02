#!/usr/bin/env bash
# Разово: локальный ROS 2 Humble под macOS (Apple Silicon) через RoboStack + micromamba.
# Ставит ровно тот дистрибутив, что на судейском стенде (Humble), включая Nav2,
# slam_toolbox и RViz2, плюс numpy/MNN — чтобы тем же питоном запускалась и походка.
#   tools/setup_ros.sh        (~2 ГБ, несколько минут)
set -euo pipefail

ROS_ENV_NAME="${ROS_ENV_NAME:-ros_humble}"
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"

command -v micromamba >/dev/null 2>&1 || {
  echo "ставлю micromamba через brew…"
  brew install micromamba
}

eval "$(micromamba shell hook -s bash)"

if micromamba env list | awk '{print $1}' | grep -qx "$ROS_ENV_NAME"; then
  echo "окружение '$ROS_ENV_NAME' уже есть — пропускаю создание"
else
  micromamba create -y -n "$ROS_ENV_NAME" -c conda-forge -c robostack-staging \
    python=3.11 ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox
fi

micromamba activate "$ROS_ENV_NAME"
python3 -c "import MNN" 2>/dev/null || python3 -m pip install MNN

echo
echo "готово. проверка:"
for m in rclpy numpy MNN; do python3 -c "import $m" && echo "  $m OK"; done
command -v ros2 >/dev/null && echo "  ros2 OK: $(command -v ros2)"
command -v rviz2 >/dev/null && echo "  rviz2 OK: $(command -v rviz2)"
echo
echo "дальше:  tools/run_webots.sh   и в другом терминале  tools/rviz.sh"
