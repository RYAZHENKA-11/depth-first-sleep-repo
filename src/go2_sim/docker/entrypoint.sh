#!/bin/bash
set -e
source /opt/ros/lyrical/setup.bash

cd /tmp
if ! colcon --log-base /tmp/log build --base-paths /ws/src \
        --build-base /tmp/build --install-base /tmp/install > /tmp/build.log 2>&1; then
    cat /tmp/build.log
    exit 1
fi
source /tmp/install/setup.bash

if [ "${HEADLESS:-0}" != "1" ]; then
    Xvfb :1 -screen 0 1600x900x24 -nolisten tcp > /tmp/xvfb.log 2>&1 &
    sleep 1
    matchbox-window-manager -use_titlebar no > /tmp/wm.log 2>&1 &
    x11vnc -display :1 -forever -shared -nopw -quiet -rfbport 5900 > /tmp/vnc.log 2>&1 &
    websockify --web /usr/share/novnc 6080 localhost:5900 > /tmp/novnc.log 2>&1 &
    echo "RViz: http://localhost:6080/vnc.html?autoconnect=true&resize=scale"
fi

if [ "$#" -gt 0 ] && [ "$1" = "bash" ]; then
    exec "$@"
fi
exec ros2 launch go2_sim sim.launch.py "$@"
