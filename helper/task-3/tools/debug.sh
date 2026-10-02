#!/usr/bin/env bash
# Единая точка входа для отладки: Webots + RViz2 + телеметрия контроллера.
#
#   tools/debug.sh                 Webots (GUI) + RViz2
#   tools/debug.sh --no-rviz       только Webots
#   tools/debug.sh --headless 120  без окон, 120 секунд, только логи (для быстрых прогонов)
#   tools/debug.sh --quiet         без телеметрии контроллера в консоли
#
# Ctrl+C гасит всё разом. Логи прогона — в logs/ рядом со скриптом.
# без set -u: скрипты активации conda обращаются к неустановленным переменным
set -o pipefail
HERE="$(cd "$(dirname "$0")/.." && pwd)"
cd "$HERE"

RVIZ=1; HEADLESS=0; DURATION=0; DEBUG_ENV=1
while [ $# -gt 0 ]; do
  case "$1" in
    --no-rviz)  RVIZ=0 ;;
    --headless) HEADLESS=1; RVIZ=0
                case "${2:-}" in ''|*[!0-9]*) ;; *) DURATION="$2"; shift ;; esac ;;
    --quiet)    DEBUG_ENV=0 ;;
    -h|--help)  sed -n '2,11p' "$0"; exit 0 ;;
    *) echo "неизвестный аргумент: $1 (см. --help)" >&2; exit 1 ;;
  esac
  shift
done

# shellcheck source=/dev/null
source "$HERE/tools/ros_env.sh" || exit 1

for mod in rclpy numpy MNN; do
  python3 -c "import $mod" 2>/dev/null || {
    echo "в ROS-окружении нет модуля $mod — запусти tools/setup_ros.sh" >&2; exit 1; }
done

WEBOTS="${WEBOTS:-}"
if [ -z "$WEBOTS" ]; then
  for c in /Applications/Webots.app/Contents/MacOS/webots /usr/local/bin/webots \
           /usr/bin/webots /snap/bin/webots "$(command -v webots 2>/dev/null || true)"; do
    [ -n "$c" ] && [ -x "$c" ] && WEBOTS="$c" && break
  done
fi
[ -x "${WEBOTS:-}" ] || { echo "Webots не найден: задай WEBOTS=/путь/к/webots" >&2; exit 1; }

[ "$DEBUG_ENV" = 1 ] && export GO2_DEBUG=1 || unset GO2_DEBUG
mkdir -p "$HERE/logs"
RUN_LOG="$HERE/logs/run.log"
PIDS=()

cleanup() {
  echo ""
  echo "[debug] останавливаю…"
  for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null; done
  sleep 2
  for pid in "${PIDS[@]:-}"; do kill -9 "$pid" 2>/dev/null; done
  # Nav2 и slam_toolbox поднимает сам контроллер — добиваем, если пережили Webots
  pkill -f "nav2_launch.py" 2>/dev/null
  pkill -f "slam_launch.py" 2>/dev/null
  echo "[debug] лог прогона: $RUN_LOG"
}
trap cleanup EXIT INT TERM

echo "[debug] ROS $ROS_DISTRO, python3=$(command -v python3)"
echo "[debug] Webots: $WEBOTS"
echo "[debug] телеметрия контроллера: $([ "$DEBUG_ENV" = 1 ] && echo включена || echo выключена)"

if [ "$HEADLESS" = 1 ]; then
  "$WEBOTS" --batch --minimize --mode=fast --stdout --stderr worlds/level3.wbt \
    > "$RUN_LOG" 2>&1 &
else
  # --batch: Webots не сохраняет мир при выходе. Без него закрытие окна записывает
  # текущее положение робота в level3.wbt, и следующий прогон стартует не со старта.
  "$WEBOTS" --batch --mode=fast --stdout --stderr worlds/level3.wbt \
    2>&1 | tee "$RUN_LOG" &
fi
PIDS+=($!)

if [ "$RVIZ" = 1 ]; then
  # RViz стартует сразу: топики подхватятся, как только контроллер начнёт публиковать
  rviz2 -d "$HERE/tools/go2.rviz" > "$HERE/logs/rviz.log" 2>&1 &
  PIDS+=($!)
  echo "[debug] RViz2 запущен (лог: logs/rviz.log)"
fi

if [ "$DURATION" -gt 0 ]; then
  echo "[debug] прогон $DURATION с…"
  sleep "$DURATION"
else
  echo "[debug] работает. Ctrl+C — остановить всё."
  wait -n 2>/dev/null || wait
fi
