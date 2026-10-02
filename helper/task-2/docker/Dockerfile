FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3.10-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_LOCALHOST_ONLY=1

WORKDIR /workspace


