#!/usr/bin/env bash
set -e

# ROS setup scripts may read unset variables; do not enable nounset here.
source /opt/ros/humble/setup.bash
source /tesseract_ws/install/setup.bash
if [[ -f /workspace/install/setup.bash ]]; then
    source /workspace/install/setup.bash
fi

exec "$@"
