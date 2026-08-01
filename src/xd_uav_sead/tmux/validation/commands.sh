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

land_all() {
  local i
  for i in 1 2 3; do
    rosservice list 2>/dev/null | grep -q "^/uav${i}/control_manager/land$" && rosservice call "/uav${i}/control_manager/land" '{}'
  done
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

strike_demo() {
  local i
  for i in 1 2 3; do
    rosrun xd_uav_sead mock_gcs.py _uav_name:="uav${i}" _cmd:=sead_mission _targets_json:='[[20,-8],[24,0],[20,8]]' _uav_type:=2 _velocity:=5.0 _Rmin:=5.0 _waypoint_radius:=2
  done
}

help_sead() {
  echo "scenario=$scenario"
  echo "可用短命令："
  echo "  takeoff_all               v3/v4/v5 起飞"
  echo "  land_all                  所有已启动 UAV 降落"
  echo "  auto_offsets              v5 仅在自动计算失败时手动重试"
  echo "  trail_all                 v5 下发 TRAIL 配置"
  echo "  formation_point X Y       v5 下发集结点"
  echo "  airspace_demo             v6 下发示例禁飞区"
  echo "  dpga_demo                 v7 下发三机分配任务"
  echo "  strike_demo               v8 下发三目标任务"
  echo "  help_sead                 再次显示帮助"
  echo
  echo "切换窗口：Ctrl+B 后按 n/p；直接跳转：Ctrl+B 后按 0/1/2。"
}

export -f takeoff_all land_all auto_offsets trail_all formation_point airspace_demo dpga_demo strike_demo help_sead
help_sead
exec bash --noprofile
