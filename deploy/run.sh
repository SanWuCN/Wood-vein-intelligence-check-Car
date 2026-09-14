#!/bin/bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
if [ -f /opt/tros/humble/setup.bash ]; then source /opt/tros/humble/setup.bash; fi
source /home/wheeltec/wheeltec_ros2/install/setup.bash
export ROS_LOCALHOST_ONLY=0
exec /usr/bin/python3 backend/app.py "$@"
