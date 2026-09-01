#!/usr/bin/env bash
set -e

ARGS=("$@")
set --
source /opt/ros/noetic/setup.bash
source /home/kzy/xd-uavsystem-test/devel/setup.bash
set -- "${ARGS[@]}"

exec /usr/bin/python3 /home/kzy/xd-uavsystem-test/src/add_red_box_scripts/gm_control_to_gazebo_typhoon_gimbal.py "$@"
