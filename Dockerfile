# syntax=docker/dockerfile:1
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion git curl wget build-essential cmake ninja-build \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    python3-pip python3-numpy \
    ros-humble-turtlesim ros-humble-urdf-tutorial \
    ros-humble-joint-state-publisher-gui \
    xvfb x11vnc fluxbox xterm novnc websockify supervisor x11-utils \
    && rm -rf /var/lib/apt/lists/*

RUN if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then rosdep init; fi \
    && rosdep update

WORKDIR /tesseract_ws
RUN mkdir -p src

# Release 0.35.0. Its bundled manifest still references the pre-consolidation
# 0.33 packages; use the corrected, versioned manifest below instead.
ARG TESSERACT_ROS2_REF=2b66f06ef39517f2b670b32ea8850731c230ad23
RUN git clone https://github.com/tesseract-robotics/tesseract_ros2.git src/tesseract_ros2 \
    && git -C src/tesseract_ros2 checkout "${TESSERACT_ROS2_REF}" \
    && test -s src/tesseract_ros2/dependencies.repos \
    && git -C src/tesseract_ros2 rev-parse HEAD

COPY docker/tesseract-dependencies.repos /tmp/tesseract-dependencies.repos
RUN vcs import src < /tmp/tesseract-dependencies.repos

# Taskflow has no ROS package manifest matching the lowercase rosdep key.
# Install its pinned source explicitly rather than skipping a missing dependency.
RUN cmake -S src/taskflow -B /tmp/taskflow-build \
       -DTF_BUILD_TESTS=OFF -DTF_BUILD_EXAMPLES=OFF -DTF_BUILD_BENCHMARKS=OFF \
    && cmake --install /tmp/taskflow-build \
    && touch src/taskflow/COLCON_IGNORE \
    && rm -rf /tmp/taskflow-build

# osqp-eigen is not packaged for Jammy; build OSQP and its Eigen wrapper.
RUN git clone --depth 1 --recurse-submodules --branch v0.6.3 https://github.com/osqp/osqp.git /tmp/osqp \
    && cmake -S /tmp/osqp -B /tmp/osqp-build \
       -DOSQP_BUILD_TESTS=OFF -DOSQP_BUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/osqp-build --parallel 2 \
    && cmake --install /tmp/osqp-build \
    && git clone --depth 1 --branch v0.8.1 https://github.com/gbionics/osqp-eigen.git /tmp/osqp-eigen \
    && cmake -S /tmp/osqp-eigen -B /tmp/osqp-eigen-build -DOSQP_EIGEN_BUILD_TESTS=OFF \
    && cmake --build /tmp/osqp-eigen-build --parallel 2 \
    && cmake --install /tmp/osqp-eigen-build \
    && rm -rf /tmp/osqp /tmp/osqp-build /tmp/osqp-eigen /tmp/osqp-eigen-build

# Match the upstream Jammy dependency repositories. Use a scoped signing key
# for Gazebo rather than the deprecated apt-key command.
RUN apt-get update \
    && apt-get install -y --no-install-recommends software-properties-common \
    && add-apt-repository -y ppa:levi-armstrong/tesseract-robotics \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.key \
       -o /usr/share/keyrings/gazebo-archive-keyring.asc \
    && printf '%s\n' \
       'deb [signed-by=/usr/share/keyrings/gazebo-archive-keyring.asc] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main' \
       > /etc/apt/sources.list.d/gazebo-stable.list

COPY docker/rosdep-jammy.yaml /etc/ros/rosdep/rosdep-jammy.yaml
RUN printf '%s\n' 'yaml file:///etc/ros/rosdep/rosdep-jammy.yaml' \
       > /etc/ros/rosdep/sources.list.d/00-tesseract-jammy.list \
    && rosdep update --rosdistro humble

RUN source /opt/ros/humble/setup.bash \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src --rosdistro humble -y \
    && rm -rf /var/lib/apt/lists/*

# The distro/PPA cereal export can resolve its include prefix to /include.
# Install the official header-only release with a correctly generated export.
# Keep this after rosdep so earlier dependency-install layers remain cached.
RUN git clone --depth 1 --branch v1.3.2 https://github.com/USCiLab/cereal.git /tmp/cereal \
    && test "$(git -C /tmp/cereal rev-parse HEAD)" = ebef1e929807629befafbb2918ea1a08c7194554 \
    && cmake -S /tmp/cereal -B /tmp/cereal-build \
       -DJUST_INSTALL_CEREAL=ON -DCEREAL_INSTALL=ON \
       -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_INSTALL_LIBDIR=lib \
    && cmake --install /tmp/cereal-build \
    && rm -rf /tmp/cereal /tmp/cereal-build

# Fail early if the imported target or serialization headers are unusable.
COPY docker/check-cereal /tmp/check-cereal
RUN cmake -S /tmp/check-cereal -B /tmp/check-cereal-build \
       -Dcereal_DIR=/usr/local/lib/cmake/cereal \
    && cmake --build /tmp/check-cereal-build --parallel 2 \
    && /tmp/check-cereal-build/check-cereal \
    && rm -rf /tmp/check-cereal /tmp/check-cereal-build

RUN source /opt/ros/humble/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build --executor sequential --cmake-args \
       -DCMAKE_BUILD_TYPE=Release \
       -Dcereal_DIR=/usr/local/lib/cmake/cereal \
       -DTESSERACT_BUILD_FCL=OFF -DBUILD_RENDERING=OFF -DBUILD_STUDIO=OFF

# Fail Docker build if expected installed packages are not discoverable.
RUN source /opt/ros/humble/setup.bash \
    && source /tesseract_ws/install/setup.bash \
    && ros2 pkg prefix tesseract_msgs \
    && ros2 pkg prefix tesseract_rosutils \
    && test -d /tesseract_ws/install

# Gazebo Fortress and the ROS 2 bridge for the hello_gazebo example.
# Installed after the Tesseract build so its cached layers are reused.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*

RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    '[ ! -f /tesseract_ws/install/setup.bash ] || source /tesseract_ws/install/setup.bash' \
    '[ ! -f /workspace/install/setup.bash ] || source /workspace/install/setup.bash' \
    >> /root/.bashrc

WORKDIR /workspace
COPY docker/entrypoint.sh /usr/local/bin/ros2-dev-entrypoint
COPY docker/supervisord.conf /etc/supervisor/ros2-dev.conf
RUN chmod +x /usr/local/bin/ros2-dev-entrypoint

ENTRYPOINT ["/usr/local/bin/ros2-dev-entrypoint"]
CMD ["supervisord", "-n", "-c", "/etc/supervisor/ros2-dev.conf"]

