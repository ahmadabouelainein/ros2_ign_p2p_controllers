#!/bin/sh

. /opt/ros/humble/setup.sh
cd /ws/ros2_ign_diffdrive
git fetch
git pull
colcon build 
. install/setup.sh

exec "$@"
