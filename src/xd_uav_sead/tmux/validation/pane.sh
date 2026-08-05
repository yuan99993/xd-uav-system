#!/usr/bin/env bash
set -euo pipefail

role="${1:?缺少 pane 角色}"
scenario="${SEAD_VALIDATION_SCENARIO:?缺少 SEAD_VALIDATION_SCENARIO}"
runtime="${SEAD_VALIDATION_RUNTIME:-/home/promise/catkin_ws/src/xd-uavsystem-test/.codex-tmp/sead_validation_offsets.env}"
run_id="${SEAD_VALIDATION_RUN_ID:?缺少 SEAD_VALIDATION_RUN_ID}"
config="/home/promise/catkin_ws/src/xd-uavsystem-test/src/xd_uav_sead/config/sead_x500_three_uav.yaml"

wait_ros() {
  until rosparam get /rosversion >/dev/null 2>&1; do sleep 1; done
}

wait_spawner() {
  until rosservice list 2>/dev/null | grep -q '^/mrs_drone_spawner/spawn$'; do sleep 1; done
}

hold_pane() {
  echo
  echo "$1"
  exec bash
}

case "$role" in
  roscore)
    exec roscore
    ;;
  gazebo)
    case "$scenario" in
      v3|v4|v5)
        wait_ros
        exec roslaunch mrs_uav_gazebo_simulation simulation.launch gui:=true
        ;;
      *) hold_pane "$scenario 不需要 Gazebo。" ;;
    esac
    ;;
  spawner)
    case "$scenario" in
      v3) wait_spawner; rosservice call /mrs_drone_spawner/spawn '1 --x500'; hold_pane "uav1 spawn 已完成。" ;;
      v4) wait_spawner; rosservice call /mrs_drone_spawner/spawn '1 2 --x500'; hold_pane "uav1/uav2 spawn 已完成。" ;;
      v5)
        wait_spawner
        rosservice call /mrs_drone_spawner/spawn '1 2 3 --x500'
        echo "三机 spawn 已排队，自动等待 PX4/MAVROS odom 并计算偏移..."
        python3 "${SEAD_VALIDATION_DIR}/compute_offsets.py"
        hold_pane "偏移已生成；三套控制链正在自动启动。"
        ;;
      *) hold_pane "$scenario 不需要 PX4/Gazebo spawner。" ;;
    esac
    ;;
  uav1|uav2|uav3)
    uav="$role"
    index="${uav#uav}"
    wait_ros
    case "$scenario" in
      v3)
        [[ "$uav" == "uav1" ]] || hold_pane "$scenario 不使用 $uav。"
        until rostopic list 2>/dev/null | grep -q "^/$uav/mavros/state$"; do sleep 1; done
        exec roslaunch xd_uav_sead sead_xd_control.launch UAV_NAME:="$uav" config:="$config" shared_frame_enabled:=false
        ;;
      v4)
        [[ "$uav" != "uav3" ]] || hold_pane "$scenario 不使用 uav3。"
        until rostopic list 2>/dev/null | grep -q "^/$uav/mavros/state$"; do sleep 1; done
        exec roslaunch xd_uav_sead sead_xd_control.launch UAV_NAME:="$uav" config:="$config" shared_frame_enabled:=false
        ;;
      v5)
        echo "等待 spawner pane 自动生成本轮偏移文件..."
        until [[ -s "$runtime" ]] && grep -qx "VALIDATION_RUN_ID=$run_id" "$runtime"; do sleep 1; done
        # shellcheck disable=SC1090
        source "$runtime"
        eval "offset_x=\${U${index}_X}; offset_y=\${U${index}_Y}; offset_z=\${U${index}_Z}"
        exec roslaunch xd_uav_sead sead_xd_control.launch UAV_NAME:="$uav" config:="$config" shared_frame_enabled:=true shared_frame_offset_x:="$offset_x" shared_frame_offset_y:="$offset_y" shared_frame_offset_z:="$offset_z"
        ;;
      v6)
        [[ "$uav" == "uav1" ]] || hold_pane "$scenario 不使用 $uav。"
        exec roslaunch xd_uav_sead sead_onboard.launch UAV_NAME:="$uav" use_simulation:=true sead_runtime_mode:=dpga_sead sead_control_mode:=position_waypoint
        ;;
      v7)
        exec roslaunch xd_uav_sead sead_onboard.launch UAV_NAME:="$uav" use_simulation:=true sead_runtime_mode:=dpga_sead sead_control_mode:=position_waypoint
        ;;
      v8)
        exec roslaunch xd_uav_sead sead_onboard.launch UAV_NAME:="$uav" use_simulation:=true sead_runtime_mode:=simple_strike simple_strike_control_mode:=position_waypoint
        ;;
    esac
    ;;
  *) echo "未知 pane 角色: $role"; exit 2 ;;
esac
