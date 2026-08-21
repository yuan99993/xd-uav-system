#!/usr/bin/env bash
set -euo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
inferred_workspace="$(cd "$(dirname "${script_path}")/../../../../.." && pwd)"
workspace="${XD_UAV_WS:-${inferred_workspace}}"
runtime="${XD_UAV_DEMO_RUNTIME:-/tmp/xd_uav_ego_demo}"
pid_file="${runtime}/roslaunch.pid"
log_file="${runtime}/roslaunch.log"

setup_ros() {
  source /opt/ros/noetic/setup.bash
  source "${workspace}/devel/setup.bash"
  export ROS_HOME="${runtime}/ros_home"
}

is_running() {
  [[ -f "${pid_file}" ]] && kill -0 "$(<"${pid_file}")" 2>/dev/null
}

case "${1:-}" in
  start)
    mkdir -p "${runtime}" "${runtime}/ros_home/log"
    if is_running; then
      echo "demo already running (pid $(<"${pid_file}"))"
      exit 0
    fi
    setup_ros
    setsid roslaunch xd_uav_system_integration ego_obstacle_demo.launch \
      gui:="${GUI:-true}" rviz:="${RVIZ:-true}" \
      </dev/null >"${log_file}" 2>&1 &
    echo "$!" >"${pid_file}"
    echo "demo starting; it will take off automatically when ready"
    echo "log: ${log_file}"
    echo "after takeoff, send: ${0} goal 6 0"
    ;;
  goal)
    [[ $# -eq 4 ]] || { echo "usage: $0 goal X Y Z"; exit 2; }
    setup_ros
    rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
      "{header: {frame_id: uav1/odom}, pose: {position: {x: $2, y: $3, z: $4}, orientation: {w: 1.0}}}"
    ;;
  land)
    setup_ros
    rosservice call /uav1/control_manager/land
    ;;
  status)
    if is_running; then
      echo "demo running (pid $(<"${pid_file}"))"
      setup_ros
      rostopic echo -n 1 /uav1/mavros/state
    else
      echo "demo not running"
      exit 1
    fi
    ;;
  stop)
    if is_running; then
      kill -- "-$(<"${pid_file}")"
      echo "requested demo shutdown"
    else
      echo "demo not running"
    fi
    ;;
  *)
    echo "usage: $0 {start|goal X Y Z|status|land|stop}"
    exit 2
    ;;
esac

