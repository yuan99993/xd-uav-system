#!/usr/bin/env bash
set -euo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
inferred_workspace="$(cd "$(dirname "${script_path}")/../../../../.." && pwd)"
workspace="${XD_UAV_WS:-${inferred_workspace}}"
runtime="${XD_UAV_DEMO_RUNTIME:-/tmp/xd_uav_ego_demo}"
pid_file="${runtime}/roslaunch.pid"
log_file="${runtime}/roslaunch.log"
ready_timeout="${XD_UAV_DEMO_READY_TIMEOUT:-120}"

setup_ros() {
  source /opt/ros/noetic/setup.bash
  source "${workspace}/devel/setup.bash"
  export ROS_HOME="${runtime}/ros_home"
  export ROS_HOSTNAME=localhost
  export ROS_MASTER_URI=http://localhost:11311
}

is_running() {
  [[ -f "${pid_file}" ]] || return 1
  local pid stat args
  pid="$(<"${pid_file}")"
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  stat="$(ps -p "$pid" -o stat= 2>/dev/null || true)"
  args="$(ps -p "$pid" -o args= 2>/dev/null || true)"
  [[ -n "$stat" && "$stat" != Z* \
     && "$args" == *"roslaunch xd_uav_planning ego_obstacle_demo.launch"* ]]
}

wait_ready() {
  local pid="$1"
  local deadline=$((SECONDS + ready_timeout))
  local state nodes
  echo "waiting for MAVROS, estimator, manager, EGO odometry and automatic takeoff..."
  while ((SECONDS < deadline)); do
    kill -0 "$pid" 2>/dev/null || {
      echo "demo roslaunch exited during startup; log: ${log_file}" >&2
      tail -n 60 "${log_file}" >&2 || true
      return 1
    }
    state="$(timeout -k 1 3 rostopic echo -n 1 /uav1/mavros/state 2>/dev/null || true)"
    nodes="$(rosnode list 2>/dev/null || true)"
    if grep -q '^connected: True$' <<<"$state" \
      && grep -q '^armed: True$' <<<"$state" \
      && timeout -k 1 3 rostopic echo -n 1 /uav1/state_estimator/state_valid 2>/dev/null | grep -q '^data: True$' \
      && timeout -k 1 3 rostopic echo -n 1 /uav1/state_estimator/localization_valid 2>/dev/null | grep -q '^data: True$' \
      && timeout -k 1 3 rostopic echo -n 1 /uav1/single_tf_manager/local_alignment_valid 2>/dev/null | grep -q '^data: True$' \
      && timeout -k 1 3 rostopic echo -n 1 /drone_0_ego/odometry 2>/dev/null | grep -q '^header:' \
      && rosservice list 2>/dev/null | grep -q '^/uav1/control_manager/land$' \
      && grep -q '^/drone_0_ego_planner_node$' <<<"$nodes" \
      && grep -q '^/uav1/ego_bridge$' <<<"$nodes"; then
      echo "demo ready and airborne; publish a goal to activate EGO ownership"
      return 0
    fi
    sleep 1
  done
  echo "demo readiness timed out after ${ready_timeout}s; log: ${log_file}" >&2
  tail -n 80 "${log_file}" >&2 || true
  return 1
}

case "${1:-}" in
  start)
    mkdir -p "${runtime}" "${runtime}/ros_home/log"
    if is_running; then
      echo "demo already running (pid $(<"${pid_file}"))"
      exit 0
    fi
    rm -f "${pid_file}"
    setup_ros
    setsid roslaunch xd_uav_planning ego_obstacle_demo.launch \
      gui:="${GUI:-true}" rviz:="${RVIZ:-true}" \
      </dev/null >"${log_file}" 2>&1 &
    echo "$!" >"${pid_file}"
    demo_pid="$!"
    echo "demo starting; automatic takeoff is enabled"
    echo "log: ${log_file}"
    wait_ready "$demo_pid"
    echo "send a goal with: ${0} goal 6 0 1"
    ;;
  goal)
    [[ $# -eq 4 ]] || { echo "usage: $0 goal X Y Z"; exit 2; }
    is_running || { echo "demo not running" >&2; exit 1; }
    setup_ros
    timeout -k 1 10 rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
      "{header: {frame_id: world}, pose: {position: {x: $2, y: $3, z: $4}, orientation: {w: 1.0}}}"
    ;;
  land)
    is_running || { echo "demo not running" >&2; exit 1; }
    setup_ros
    timeout -k 1 10 rosservice call /uav1/control_manager/land
    ;;
  status)
    if is_running; then
      echo "demo running (pid $(<"${pid_file}"))"
      setup_ros
      timeout -k 1 5 rostopic echo -n 1 /uav1/mavros/state
    else
      rm -f "${pid_file}"
      echo "demo not running"
      exit 1
    fi
    ;;
  stop)
    if is_running; then
      demo_pid="$(<"${pid_file}")"
      setup_ros
      # This script owns the dedicated localhost:11311 graph.  Let roslaunch,
      # the MRS spawner and Gazebo run their shutdown handlers before using
      # process-group signals as a fallback.
      timeout -k 1 8 rosnode kill -a >/dev/null 2>&1 || true
      kill -INT -- "-${demo_pid}" 2>/dev/null || true
      for _ in {1..80}; do
        kill -0 "$demo_pid" 2>/dev/null || break
        sleep 0.25
      done
      if kill -0 "$demo_pid" 2>/dev/null; then
        kill -TERM -- "-${demo_pid}" 2>/dev/null || true
        sleep 1
      fi
      if kill -0 "$demo_pid" 2>/dev/null; then
        kill -KILL -- "-${demo_pid}" 2>/dev/null || true
      fi
      rm -f "${pid_file}"
      echo "demo stopped"
    else
      rm -f "${pid_file}"
      echo "demo not running"
    fi
    ;;
  *)
    echo "usage: $0 {start|goal X Y Z|status|land|stop}"
    exit 2
    ;;
esac
