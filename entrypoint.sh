#!/bin/bash
set -e

# setup ros2 environment
source "${ROS_ROOT}/install/setup.bash"
# create a new shell
exec su -pl $USER --shell=/bin/bash
