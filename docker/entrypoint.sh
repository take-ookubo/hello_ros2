#!/usr/bin/env bash
set -e

# ROS setup scripts may read unset variables; do not enable nounset here.
# ROS_DISTRO is set by the osrf/ros base image (humble or jazzy).
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f /tesseract_ws/install/setup.bash ]]; then
    source /tesseract_ws/install/setup.bash
fi
if [[ -f /workspace/install/setup.bash ]]; then
    source /workspace/install/setup.bash
fi

exec "$@"
