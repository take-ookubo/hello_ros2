# syntax=docker/dockerfile:1
# ROS 2 Jazzy (Ubuntu 24.04) with Gazebo Harmonic and the noVNC desktop.
# Tesseract is not built here yet; the Humble image in ../../Dockerfile keeps it.
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion git curl wget build-essential cmake ninja-build \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    python3-pip python3-numpy \
    ros-jazzy-turtlesim ros-jazzy-urdf-tutorial \
    ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro \
    xvfb x11vnc fluxbox xterm novnc websockify supervisor x11-utils \
    ffmpeg xdotool mesa-utils feh \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Harmonic, the ROS 2 bridge, ros2_control, and MoveIt 2.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
    ros-jazzy-moveit ros-jazzy-moveit-ros-perception \
    && rm -rf /var/lib/apt/lists/*

RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
    && rosdep update --rosdistro jazzy

RUN printf '%s\n' \
    'source /opt/ros/jazzy/setup.bash' \
    '[ ! -f /workspace/install/setup.bash ] || source /workspace/install/setup.bash' \
    >> /root/.bashrc

WORKDIR /workspace
COPY docker/entrypoint.sh /usr/local/bin/ros2-dev-entrypoint
COPY docker/supervisord.conf /etc/supervisor/ros2-dev.conf
RUN chmod +x /usr/local/bin/ros2-dev-entrypoint

ENTRYPOINT ["/usr/local/bin/ros2-dev-entrypoint"]
CMD ["supervisord", "-n", "-c", "/etc/supervisor/ros2-dev.conf"]
