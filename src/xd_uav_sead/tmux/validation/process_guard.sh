#!/usr/bin/env bash
set -euo pipefail

mode="${1:-check}"
patterns=(
  '/opt/ros/noetic/share/px4/px4/px4 .* -w sitl_uav[123]'
  '/opt/ros/noetic/lib/mavros/mavros_node .*uav[123]-mavros'
  '/opt/ros/noetic/bin/roslaunch mrs_uav_gazebo_simulation simulation.launch'
  'gzserver .*mrs_gazebo_common_resources/worlds/grass_plane.world'
  'gzclient .*gazebo_ros_api_plugin'
)

list_matches() {
  local pattern
  for pattern in "${patterns[@]}"; do
    pgrep -af "$pattern" || true
  done | sort -n -u
}

case "$mode" in
  check)
    matches="$(list_matches)"
    if [[ -n "$matches" ]]; then
      echo "检测到会占用本验证端口/实例的残留进程：" >&2
      echo "$matches" >&2
      echo "请先运行 ./kill.sh；start.sh 已拒绝叠加启动。" >&2
      exit 1
    fi
    ;;
  cleanup)
    mapfile -t pids < <(list_matches | awk '{print $1}')
    if ((${#pids[@]})); then
      echo "停止本验证链识别到的派生 PX4/MAVROS/Gazebo 进程: ${pids[*]}"
      kill "${pids[@]}" 2>/dev/null || true
      sleep 4
    fi
    remaining="$(list_matches)"
    if [[ -n "$remaining" ]]; then
      echo "以下进程未能正常退出：" >&2
      echo "$remaining" >&2
      exit 1
    fi
    ;;
  *) echo "用法: $0 {check|cleanup}" >&2; exit 2 ;;
esac
