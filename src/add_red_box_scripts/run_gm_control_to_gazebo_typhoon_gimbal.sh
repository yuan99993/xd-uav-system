#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CATKIN_WORKSPACE="$(cd "${REPOSITORY_ROOT}/../.." && pwd)"

ARGS=("$@")
set --
source /opt/ros/noetic/setup.bash
source "${CATKIN_WORKSPACE}/devel/setup.bash"
set -- "${ARGS[@]}"

exec /usr/bin/python3 "${SCRIPT_DIR}/gm_control_to_gazebo_typhoon_gimbal.py" "$@"
