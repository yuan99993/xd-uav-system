#!/usr/bin/env bash
set -u

scenario="${SEAD_VALIDATION_SCENARIO:-unknown}"
echo "SEAD validation: $scenario"
echo "每 2 秒刷新；Ctrl+C 停止刷新后可在本 pane 执行普通 ROS 检查。"
sleep 2

while true; do
  clear
  echo "scenario=$scenario  wall=$(date '+%F %T')"
  echo
  for uav in uav1 uav2 uav3; do
    if rostopic list 2>/dev/null | grep -q "^/$uav/mavros/state$"; then
      echo "[$uav MAVROS]"
      timeout 2 rostopic echo -n 1 "/$uav/mavros/state" 2>/dev/null | grep -E 'connected:|armed:|mode:|system_status:' || true
      echo "[$uav estimator]"
      timeout 2 rostopic echo -n 1 "/$uav/state_estimator/state_valid" 2>/dev/null | grep 'data:' || true
      timeout 2 rostopic echo -n 1 "/$uav/state_estimator/localization_valid" 2>/dev/null | grep 'data:' || true
      echo "[$uav manager]"
      timeout 2 rostopic echo -n 1 "/$uav/control_manager/status" 2>/dev/null | grep 'data:' || true
      echo
    fi
  done
  sleep 2
done
