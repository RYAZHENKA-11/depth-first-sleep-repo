#!/usr/bin/env bash
# Активация локального ROS 2 Humble (RoboStack/micromamba, нативно под osx-arm64).
# Использовать через source:  source tools/ros_env.sh
#
# Судейский образ — Ubuntu 22.04 + ROS 2 Humble; здесь тот же дистрибутив, собранный
# conda-forge/robostack, чтобы локально гонять ровно тот путь кода (Nav2 + slam_toolbox),
# который поедет на зачёт, а не только fallback-ветку.
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$HOME/micromamba}"
ROS_ENV_NAME="${ROS_ENV_NAME:-ros_humble}"

if ! command -v micromamba >/dev/null 2>&1; then
  echo "micromamba не найден — установи: brew install micromamba" >&2
  return 1 2>/dev/null || exit 1
fi

eval "$(micromamba shell hook -s bash)"
micromamba activate "$ROS_ENV_NAME" || {
  echo "нет окружения '$ROS_ENV_NAME' — создай: tools/setup_ros.sh" >&2
  return 1 2>/dev/null || exit 1
}

# DDS замкнут на loopback — как на судейском стенде (ROS2.md)
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
