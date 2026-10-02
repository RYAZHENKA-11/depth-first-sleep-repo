#!/usr/bin/env bash
# Запуск уровня в Webots (macOS / Linux). Требует ./setup.sh и установленный Webots R2025a (INSTALL.md).
#   ./run.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
VBIN="$HERE/.venv/bin"
[ -x "$VBIN/python3" ] || { echo "нет .venv — сначала ./setup.sh" >&2; exit 1; }
# Webots зовёт контроллер как python3 из PATH — ставим venv первым, чтобы это был наш python с numpy+MNN.
export PATH="$VBIN:$PATH"
SITE="$(ls -d "$HERE"/.venv/lib/python3.*/site-packages 2>/dev/null | head -1 || true)"
[ -n "${SITE:-}" ] && export PYTHONPATH="${SITE}${PYTHONPATH:+:$PYTHONPATH}"

# ROS 2 Humble from .ros-ros/
ROSROOT="$HERE/.ros-ros/humble"
if [ -d "$ROSROOT" ]; then
  export PYTHONPATH="${ROSROOT}/local/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
  export PYTHONPATH="${ROSROOT}/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
  # Debian-библиотеки, на которых собран ROS (spdlog/fmt/console_bridge), и pyenv libpython3.10.
  LD_PY="$( "$VBIN/python3" -c 'import sys; print(sys.base_prefix)' 2>/dev/null || true )/lib"
  export LD_LIBRARY_PATH="${ROSROOT}/lib/x86_64-linux-gnu:${ROSROOT}/lib:$HERE/.ros-ros/deb-libs${LD_PY:+:$LD_PY}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export AMENT_PREFIX_PATH="${ROSROOT}${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  export ROS_LOCALHOST_ONLY=1
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
else
  echo "предупреждение: нет $ROSROOT — import rclpy на хосте работать не будет" >&2
fi

WEBOTS="${WEBOTS:-}"
if [ -z "$WEBOTS" ]; then
  for c in /Applications/Webots.app/Contents/MacOS/webots /usr/local/bin/webots /usr/bin/webots /snap/bin/webots "$(command -v webots 2>/dev/null || true)"; do
    [ -n "$c" ] && [ -x "$c" ] && WEBOTS="$c" && break
  done
fi
[ -n "$WEBOTS" ] && [ -x "$WEBOTS" ] || { echo "Webots не найден. Установи R2025a (INSTALL.md) или задай WEBOTS=/путь/к/webots ./run.sh" >&2; exit 1; }
exec "$WEBOTS" "$HERE/worlds/level2.wbt"
