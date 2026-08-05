#!/usr/bin/env bash
set -u

scenario="${SEAD_VALIDATION_SCENARIO:-unknown}"
script_dir="${SEAD_VALIDATION_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
export scenario script_dir

takeoff_all() {
  local count=1
  [[ "$scenario" == "v4" ]] && count=2
  [[ "$scenario" == "v5" ]] && count=3
  local i
  for ((i=1; i<=count; i++)); do rosservice call "/uav${i}/control_manager/takeoff" '{}'; done
}

waypoint() {
  [[ "$scenario" == "v3" ]] || { echo "waypoint 只用于 v3"; return 1; }
  [[ $# -eq 3 ]] || { echo "用法: waypoint X Y Z"; return 2; }
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=waypoint \
    _x:="$1" _y:="$2" _z:="$3" _radius:=1.0
}

land_all() {
  local i attempt output accepted failures=0
  for i in 1 2 3; do
    rosservice list 2>/dev/null | grep -q "^/uav${i}/control_manager/land$" || continue
    accepted=0
    # A loaded three-UAV simulation can briefly report stale MAVROS/controller
    # inputs.  LAND is idempotent, so bounded retries are safer than silently
    # leaving one aircraft airborne after a single transient rejection.
    for attempt in 1 2 3 4 5; do
      output="$(rosservice call "/uav${i}/control_manager/land" '{}' 2>&1)" || true
      printf '%s\n' "$output"
      if grep -q '^success: True$' <<<"$output"; then
        accepted=1
        echo "uav${i}: 降落请求已接受（第 ${attempt} 次）"
        break
      fi
      ((attempt < 5)) && sleep 1
    done
    if ((accepted == 0)); then
      echo "uav${i}: 降落请求连续 5 次未接受；保持现场并检查 manager/estimator/MAVROS 状态。" >&2
      ((failures += 1))
    fi
  done
  ((failures == 0))
}

auto_offsets() {
  [[ "$scenario" == "v5" ]] || { echo "auto_offsets 只用于 v5"; return 1; }
  echo "重新等待三机 PX4/MAVROS odom并计算偏移；正常情况下 spawner pane 已自动执行过。"
  python3 "$script_dir/compute_offsets.py"
}

trail_all() {
  local i
  for i in 1 2 3; do
    rosrun xd_uav_sead mock_gcs.py _uav_name:="uav${i}" _cmd:=formation_config _shape:=TRAIL _spacing:=4.0 _standoff:=12.0 _safe_sep:=2.0 _alt_step:=0.5
  done
}

formation_point() {
  [[ $# -eq 2 ]] || { echo "用法: formation_point X Y"; return 2; }
  local i
  for i in 1 2 3; do
    rosrun xd_uav_sead mock_gcs.py _uav_name:="uav${i}" _cmd:=formation_point _x:="$1" _y:="$2" _z:=3.5 _loiter_radius:=8.0
  done
}

airspace_demo() {
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=airspace_clear
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=airspace_zone _zone_id:=1 _minAlt:=0.0 _maxAlt:=20.0 _points_json:='[[5,-2],[9,-2],[9,2],[5,2]]'
}

dpga_demo() {
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=sead_mission _targets_json:='[[20,0]]' _uav_type:=1 _velocity:=5.0 _Rmin:=5.0 _waypoint_radius:=2
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav2 _cmd:=sead_mission _targets_json:='[[20,0]]' _uav_type:=2 _velocity:=5.0 _Rmin:=5.0 _waypoint_radius:=2
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav3 _cmd:=sead_mission _targets_json:='[[20,0]]' _uav_type:=3 _velocity:=5.0 _Rmin:=5.0 _waypoint_radius:=2
}

dpga_insert() {
  [[ "$scenario" == "v7" ]] || { echo "dpga_insert 只用于 v7"; return 1; }
  [[ $# -eq 2 ]] || { echo "用法: dpga_insert X Y"; return 2; }
  rosrun xd_uav_sead mock_gcs.py _uav_name:=uav1 _cmd:=task_insert \
    _x:="$1" _y:="$2" _task_type:=0
}

strike_demo() {
  local i
  for i in 1 2 3; do
    rosrun xd_uav_sead mock_gcs.py _uav_name:="uav${i}" _cmd:=sead_mission _targets_json:='[[20,-8],[24,0],[20,8]]' _uav_type:=2 _velocity:=5.0 _Rmin:=5.0 _waypoint_radius:=2
  done
}

visualize() {
  local output_root="/home/promise/catkin_ws/src/xd-uavsystem-test/.codex-tmp/visualizations"
  local offset_file="${SEAD_VALIDATION_RUNTIME:-/home/promise/catkin_ws/src/xd-uavsystem-test/.codex-tmp/sead_validation_offsets.env}"
  local run_id="${SEAD_VALIDATION_RUN_ID:-}"
  rosrun xd_uav_sead sead_validation_visualizer.py \
    _scenario:="$scenario" _output_root:="$output_root" \
    _offset_file:="$offset_file" _expected_run_id:="$run_id" &
  echo "可视化窗口启动中（PID $!）；V5 自动使用本轮 offset。"
  echo "关闭窗口时保存 PNG/JSON/CSV，V5 额外保存 offsets.env 到 $output_root。"
}

help_sead() {
  echo "scenario=$scenario"
  echo "可用短命令："
  echo "  takeoff_all               v3/v4/v5 起飞"
  echo "  waypoint X Y Z            v3 下发单机航点"
  echo "  land_all                  所有已启动 UAV 降落"
  echo "  auto_offsets              v5 仅在自动计算失败时手动重试"
  echo "  trail_all                 v5 下发 TRAIL 配置"
  echo "  formation_point X Y       v5 下发集结点"
  echo "  airspace_demo             v6 下发示例禁飞区"
  echo "  dpga_demo                 v7 下发三机分配任务"
  echo "  dpga_insert X Y           v7 动态插入目标"
  echo "  strike_demo               v8 下发三目标任务"
  echo "  visualize                 v3-v8 实时显示并保存验证证据"
  echo "  help_sead                 再次显示帮助"
  echo
  echo "切换窗口：Ctrl+B 后按 n/p；直接跳转：Ctrl+B 后按 0/1/2。"
}

export -f takeoff_all waypoint land_all auto_offsets trail_all formation_point airspace_demo dpga_demo dpga_insert strike_demo visualize help_sead
help_sead
exec bash --noprofile
