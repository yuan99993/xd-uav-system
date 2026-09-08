#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
single_script="${script_dir}/ego_obstacle_demo.sh"
runtime="${XD_UAV_MULTI_RUNTIME:-/tmp/xd_uav_ego_multi_demo}"
pid_file="${runtime}/roslaunch.pid"
log_file="${runtime}/roslaunch.log"
ready_timeout="${XD_UAV_DEMO_READY_TIMEOUT:-180}"

resolve_setup() {
  local directory candidate resolved=""
  if [[ -n "${XD_UAV_WS:-}" ]]; then
    for candidate in "${XD_UAV_WS}/devel/setup.bash" \
                     "${XD_UAV_WS}/install/setup.bash"; do
      [[ -r "${candidate}" ]] && { echo "${candidate}"; return 0; }
    done
    return 1
  fi

  # rosrun normally inherits the active Catkin overlay. Prefer that explicit
  # environment over guessing from this script's (possibly installed) path.
  if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
    while IFS= read -r candidate; do
      [[ "${candidate}" == /opt/ros/* ]] && continue
      [[ -r "${candidate}/setup.bash" ]] && {
        echo "${candidate}/setup.bash"
        return 0
      }
    done < <(tr ':' '\n' <<<"${CMAKE_PREFIX_PATH}")
  fi

  # Direct execution may not have an overlay environment. Keep walking so an
  # old nested repository devel space cannot hide the enclosing workspace.
  directory="${script_dir}"
  while [[ "${directory}" != / ]]; do
    for candidate in "${directory}/devel/setup.bash" \
                     "${directory}/install/setup.bash"; do
      if [[ -d "${directory}/src" && -r "${candidate}" ]]; then
        resolved="${candidate}"
        break
      fi
    done
    directory="$(dirname "${directory}")"
  done
  [[ -n "${resolved}" ]] && { echo "${resolved}"; return 0; }
  return 1
}

if ! setup_file="$(resolve_setup)"; then
  echo "cannot find devel/setup.bash or install/setup.bash; set XD_UAV_WS" >&2
  exit 1
fi

setup_ros() {
  source /opt/ros/noetic/setup.bash
  source "${setup_file}"
  export ROS_HOME="${runtime}/ros_home"
  export ROS_HOSTNAME=localhost
  export ROS_MASTER_URI=http://localhost:11311
}

gazebo_master_in_use() {
  local uri="${GAZEBO_MASTER_URI:-http://localhost:11345}"
  python3 - "${uri}" <<'PY'
import socket
import sys
from urllib.parse import urlparse

target = urlparse(sys.argv[1])
host = target.hostname or "localhost"
port = target.port or 11345
try:
    addresses = socket.getaddrinfo(host, port, type=socket.SOCK_STREAM)
except socket.gaierror:
    sys.exit(1)
for family, socktype, protocol, _, address in addresses:
    try:
        sock = socket.socket(family, socktype, protocol)
    except OSError:
        continue
    sock.settimeout(0.25)
    try:
        try:
            if sock.connect_ex(address) == 0:
                sys.exit(0)
        except OSError:
            pass
    finally:
        sock.close()
sys.exit(1)
PY
}

is_multi_running() {
  [[ -f "${pid_file}" ]] || return 1
  local pid args stat
  pid="$(<"${pid_file}")"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  stat="$(ps -p "${pid}" -o stat= 2>/dev/null || true)"
  args="$(ps -p "${pid}" -o args= 2>/dev/null || true)"
  [[ -n "${stat}" && "${stat}" != Z* &&
     "${args}" == *"ego_multi_obstacle_demo.launch"* ]]
}

wait_multi_ready() {
  local pid="$1" checker checker_pid
  echo "waiting for three closed-loop PX4/EGO chains (owner, forwarding, valid command and motion)..."
  checker="${script_dir}/wait_ego_swarm_ready.py"
  timeout -k 2 "$((ready_timeout + 5))" "${checker}" \
    --timeout "${ready_timeout}" --minimum-horizontal-motion 2.00 &
  checker_pid="$!"
  while kill -0 "${checker_pid}" 2>/dev/null; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${checker_pid}" 2>/dev/null || true
      wait "${checker_pid}" 2>/dev/null || true
      echo "multi demo exited during startup; log: ${log_file}" >&2
      tail -n 100 "${log_file}" >&2 || true
      return 1
    fi
    if [[ -r "${log_file}" ]] && grep -Eq \
        'Unable to start server\[bind: Address already in use\]|PX4 vehicle spawn rejected: Gazebo model state topic not found' \
        "${log_file}"; then
      kill -TERM "${checker_pid}" 2>/dev/null || true
      wait "${checker_pid}" 2>/dev/null || true
      echo "multi demo startup failed: Gazebo did not start (its master port may be occupied)" >&2
      echo "log: ${log_file}" >&2
      tail -n 100 "${log_file}" >&2 || true
      return 1
    fi
    sleep 0.5
  done
  if wait "${checker_pid}"; then
    kill -0 "${pid}" 2>/dev/null || {
      echo "multi demo exited immediately after readiness; log: ${log_file}" >&2
      return 1
    }
    echo "multi demo ready: uav1..uav3 have EGO ownership, valid control and verified motion"
    return 0
  fi
  echo "multi readiness failed; log: ${log_file}" >&2
  tail -n 100 "${log_file}" >&2 || true
  return 1
}

cleanup_failed_multi_start() {
  local result="$?"
  trap - EXIT INT TERM
  echo "cleaning up failed multi-demo startup..." >&2
  multi_stop
  exit "${result}"
}

multi_stop() {
  if is_multi_running; then
    local pid
    pid="$(<"${pid_file}")"
    setup_ros
    timeout -k 1 12 rosnode kill -a >/dev/null 2>&1 || true
    kill -INT -- "-${pid}" 2>/dev/null || true
    for _ in {1..100}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.25
    done
    kill -0 "${pid}" 2>/dev/null && kill -TERM -- "-${pid}" 2>/dev/null || true
    sleep 1
    kill -0 "${pid}" 2>/dev/null && kill -KILL -- "-${pid}" 2>/dev/null || true
  fi
  rm -f "${pid_file}"
  echo "multi demo stopped"
}

case "${1:-}" in
  start)
    case "${2:-single}" in
      single) exec "${single_script}" start ;;
      multi|swarm)
        [[ "${3:-3}" == "3" ]] || { echo "only the validated three-UAV profile is supported" >&2; exit 2; }
        mkdir -p "${runtime}" "${runtime}/ros_home/log"
        is_multi_running && { echo "multi demo already running (pid $(<"${pid_file}"))"; exit 0; }
        setup_ros
        if rosnode list >/dev/null 2>&1; then
          echo "an existing ROS graph is using ${ROS_MASTER_URI}; stop it before multi start" >&2
          exit 1
        fi
        if gazebo_master_in_use; then
          echo "Gazebo master ${GAZEBO_MASTER_URI:-http://localhost:11345} is already in use; stop the existing Gazebo simulation before multi start" >&2
          exit 1
        fi
        setsid roslaunch xd_uav_planning ego_multi_obstacle_demo.launch \
          gui:="${GUI:-true}" rviz:="${RVIZ:-true}" \
          </dev/null >"${log_file}" 2>&1 &
        echo "$!" >"${pid_file}"
        echo "three-UAV EGO demo starting in the Gazebo obstacle world"
        echo "log: ${log_file}"
        trap cleanup_failed_multi_start EXIT INT TERM
        wait_multi_ready "$!"
        trap - EXIT INT TERM
        ;;
      *) echo "usage: $0 start {single|swarm 3}" >&2; exit 2 ;;
    esac
    ;;
  goal)
    [[ $# -eq 5 ]] || { echo "usage: $0 goal UAV X Y Z" >&2; exit 2; }
    [[ "$2" =~ ^uav([1-3])$ ]] || { echo "UAV must be uav1, uav2 or uav3" >&2; exit 2; }
    uav_id="${BASH_REMATCH[1]}"
    is_multi_running || { echo "multi demo not running" >&2; exit 1; }
    setup_ros
    ego_id=$((uav_id - 1))
    timeout -k 1 10 rostopic pub -1 "/drone_${ego_id}_planning/goal" geometry_msgs/PoseStamped \
      "{header: {frame_id: world}, pose: {position: {x: $3, y: $4, z: $5}, orientation: {w: 1.0}}}"
    ;;
  goals)
    [[ "${2:-}" == formation ]] || { echo "usage: $0 goals formation" >&2; exit 2; }
    is_multi_running || { echo "multi demo not running" >&2; exit 1; }
    setup_ros
    for spec in "0 -4" "1 0" "2 4"; do
      read -r id y <<<"${spec}"
      uav_id=$((id + 1))
      rostopic pub -1 "/drone_${id}_planning/goal" geometry_msgs/PoseStamped \
        "{header: {frame_id: world}, pose: {position: {x: 12, y: ${y}, z: 1.5}, orientation: {w: 1.0}}}" >/dev/null
    done
    echo "formation goals published"
    ;;
  status)
    if is_multi_running; then
      setup_ros
      for uav in uav1 uav2 uav3; do
        echo "[${uav}]"
        timeout -k 1 4 rostopic echo -n 1 "/${uav}/mavros/state" | grep -E 'connected:|armed:|mode:'
      done
    else
      exec "${single_script}" status
    fi
    ;;
  land)
    target="${2:-all}"
    is_multi_running || { exec "${single_script}" land; }
    setup_ros
    for uav in uav1 uav2 uav3; do
      [[ "${target}" == all || "${target}" == "${uav}" ]] || continue
      timeout -k 1 10 rosservice call "/${uav}/control_manager/land"
    done
    ;;
  stop)
    if is_multi_running; then multi_stop; else exec "${single_script}" stop; fi
    ;;
  *)
    echo "usage: $0 {start [single|swarm 3]|goal UAV X Y Z|goals formation|status|land [UAV|all]|stop}"
    exit 2
    ;;
esac
