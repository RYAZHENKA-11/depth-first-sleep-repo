FROM ros:lyrical-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-lyrical-rviz2 \
        ros-lyrical-robot-state-publisher \
        python3-yaml \
        xvfb x11vnc novnc websockify matchbox-window-manager scrot \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GO2_MESH_DIR=/ws/helper/task-3/common/protos/dae

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
EXPOSE 6080
ENTRYPOINT ["/entrypoint.sh"]
