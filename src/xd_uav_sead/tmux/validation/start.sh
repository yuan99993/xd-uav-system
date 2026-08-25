#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scenario="${1:-}"
[[ $# -gt 0 ]] && shift
attach_mode=""
v9_profile="nominal"
v9_zone_half_size=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-attach)
      attach_mode="--no-attach"
      shift
      ;;
    --profile)
      [[ $# -ge 2 ]] || { echo "--profile 缺少参数" >&2; exit 2; }
      v9_profile="$2"
      shift 2
      ;;
    --zone-half-size)
      [[ $# -ge 2 ]] || { echo "--zone-half-size 缺少参数（单位 m）" >&2; exit 2; }
      v9_zone_half_size="$2"
      shift 2
      ;;
    --zone-half-size=*)
      v9_zone_half_size="${1#*=}"
      shift
      ;;
    nominal|relaxed1|relaxed2)
      v9_profile="$1"
      shift
      ;;
    "")
      shift
      ;;
    *)
      echo "未知参数: $1" >&2
      exit 2
      ;;
  esac
done

wait_flight_ready() {
  local count="$1"
  local timeout_s="${SEAD_VALIDATION_READY_TIMEOUT:-180}"
  local stable_required="${SEAD_VALIDATION_READY_STABLE_COUNT:-3}"
  local deadline=$((SECONDS + timeout_s))
  local stable=0
  local pending=""
  local i state node_list

  # start.sh may be called from a shell which has not sourced ROS yet.
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  # shellcheck disable=SC1091
  source /home/promise/catkin_ws/devel/setup.bash
  export ROS_HOSTNAME=localhost
  export ROS_MASTER_URI=http://localhost:11311

  echo "等待 ${count} 架 UAV 的 MAVROS、estimator 和 manager 连续就绪..."
  while ((SECONDS < deadline)); do
    pending=""
    node_list="$(rosnode list 2>/dev/null || true)"
    for ((i=1; i<=count; i++)); do
      state="$(timeout -k 1 3 rostopic echo -n 1 "/uav${i}/mavros/state" 2>/dev/null || true)"
      if ! grep -q '^connected: True$' <<<"$state"; then
        pending+=" uav${i}:mavros"
      fi
      if ! timeout -k 1 3 rostopic echo -n 1 "/uav${i}/state_estimator/state_valid" 2>/dev/null | grep -q '^data: True$'; then
        pending+=" uav${i}:state"
      fi
      if ! timeout -k 1 3 rostopic echo -n 1 "/uav${i}/state_estimator/localization_valid" 2>/dev/null | grep -q '^data: True$'; then
        pending+=" uav${i}:localization"
      fi
      if ! rosservice list 2>/dev/null | grep -q "^/uav${i}/control_manager/takeoff$"; then
        pending+=" uav${i}:manager"
      fi
    done
    if [[ "$scenario" == "v9" ]]; then
      if ! grep -q '^/uav1/sead_onboard$' <<<"$node_list"; then
        pending+=" uav1:sead"
      fi
      if ! grep -q '^/fixedwing_nofly_acceptance$' <<<"$node_list"; then
        pending+=" acceptance"
      fi
    fi

    if [[ -z "$pending" ]]; then
      ((stable += 1))
      if ((stable >= stable_required)); then
        echo "验证链已就绪：${count} 架 UAV 连续 ${stable_required} 次通过 readiness 检查。"
        return 0
      fi
    else
      stable=0
    fi
    echo "readiness ${stable}/${stable_required}; waiting:${pending:- stability}"
    sleep 1
  done

  echo "等待验证链就绪超时（${timeout_s}s）；最后未就绪:${pending:- unknown}" >&2
  echo "tmux 会话已保留供诊断：tmux -L sead-validation attach -t sead-validation" >&2
  echo "诊断后执行 $script_dir/kill.sh 清理。" >&2
  return 1
}

wait_fixedwing_result() {
  # Gazebo can run below real time under load.  The acceptance node measures
  # its mission deadline in simulation time, so leave enough wall-clock time
  # for its latched result to arrive.
  local timeout_s="${SEAD_FIXEDWING_ACCEPTANCE_TIMEOUT:-360}"
  local deadline=$((SECONDS + timeout_s))
  local result=""
  echo "v9 已起飞执行：等待动态禁飞区重规划、真实轨迹绕飞、到达目标并返航盘旋..."
  while ((SECONDS < deadline)); do
    result="$(timeout -k 1 4 rostopic echo -n 1 /uav1/sead/fixedwing_acceptance/result 2>/dev/null || true)"
    # rostopic renders std_msgs/String as YAML and escapes the JSON quotes.
    # Match the decoded field shape instead of requiring unescaped JSON.
    if grep -Eq 'success[^[:alnum:]]+true' <<<"$result"; then
      echo "固定翼动态禁飞区验收通过：$result"
      return 0
    fi
    if grep -Eq 'success[^[:alnum:]]+false' <<<"$result"; then
      echo "固定翼动态禁飞区验收失败：$result" >&2
      return 1
    fi
    sleep 2
  done
  echo "固定翼动态禁飞区验收等待超时（${timeout_s}s）。" >&2
  return 1
}

check_fixedwing_plugins() {
  local plugin_dir="/home/promise/PX4-Autopilot/build/px4_sitl_default/build_gazebo"
  local plugin
  for plugin in \
    libgazebo_airspeed_plugin.so \
    libgazebo_barometer_plugin.so \
    libgazebo_groundtruth_plugin.so \
    libgazebo_imu_plugin.so \
    libgazebo_magnetometer_plugin.so \
    libgazebo_mavlink_interface.so \
    libgazebo_motor_model.so \
    libLiftDragPlugin.so \
    libForceVisual.so; do
    if [[ ! -f "$plugin_dir/$plugin" ]]; then
      echo "v9 缺少 PX4 官方 Gazebo 插件: $plugin_dir/$plugin" >&2
      echo "请在该目录用 ninja -j2 补齐官方构建产物。" >&2
      return 1
    fi
  done
}

case "$scenario" in
  v3|v4|v5|v6|v7|v8|v9) ;;
  *)
    echo "用法: $0 {v3|v4|v5|v6|v7|v8|v9} [--no-attach] [--profile nominal|relaxed1|relaxed2] [--zone-half-size M]"
    echo "v3=单机  v4=两机并发  v5=三机编队  v6=Airspace  v7=DPGA  v8=SimpleStrike  v9=固定翼动态禁飞区"
    exit 2
    ;;
esac

if [[ "$scenario" == "v9" ]]; then
  check_fixedwing_plugins
  case "$v9_profile" in
    nominal|relaxed1|relaxed2) ;;
    *) echo "v9 profile 必须是 nominal|relaxed1|relaxed2" >&2; exit 2 ;;
  esac
  if [[ -n "$v9_zone_half_size" ]]; then
    if [[ ! "$v9_zone_half_size" =~ ^[0-9]+([.][0-9]+)?$ ]] \
      || ! awk -v value="$v9_zone_half_size" 'BEGIN { exit !(value > 0.0) }'; then
      echo "--zone-half-size 必须是大于 0 的米数" >&2
      exit 2
    fi
  fi
fi

if tmux -L sead-validation has-session -t sead-validation 2>/dev/null; then
  echo "sead-validation 已在运行。先执行 ./kill.sh，或直接连接："
  echo "tmux -L sead-validation attach -t sead-validation"
  exit 1
fi

"$script_dir/process_guard.sh" check

export SEAD_VALIDATION_SCENARIO="$scenario"
export SEAD_FIXEDWING_PROFILE="$v9_profile"
export SEAD_FIXEDWING_ZONE_HALF_SIZE="$v9_zone_half_size"
if [[ "$scenario" == "v9" && "$attach_mode" != "--no-attach" ]]; then
  auto_visualize_default="true"
else
  auto_visualize_default="false"
fi
export SEAD_VALIDATION_AUTO_VISUALIZE="${SEAD_VALIDATION_AUTO_VISUALIZE:-$auto_visualize_default}"
export SEAD_VALIDATION_DIR="$script_dir"
export SEAD_VALIDATION_RUNTIME="/home/promise/catkin_ws/src/xd-uavsystem-test/.codex-tmp/sead_validation_offsets.env"
export SEAD_VALIDATION_RUN_ID="$(date +%s)-$$"
# PX4 SITL persists parameters and runtime state below ROS_HOME. Give every
# validation run a fresh root so a previous crash or different airframe cannot
# alter runway acceleration, mixers, failsafes, or the generated ULog.
export SEAD_VALIDATION_ROS_HOME="/tmp/sead_validation_${SEAD_VALIDATION_RUN_ID}"
export ROS_HOME="$SEAD_VALIDATION_ROS_HOME"
mkdir -p "$ROS_HOME/log"

if [[ "$scenario" == "v5" ]]; then
  rm -f "$SEAD_VALIDATION_RUNTIME"
fi

cd "$script_dir"
tmuxinator start -p ./session.yml </dev/null

case "$scenario" in
  v3) wait_flight_ready 1 ;;
  v4) wait_flight_ready 2 ;;
  v5) wait_flight_ready 3 ;;
  v9)
    wait_flight_ready 1
    if [[ "$attach_mode" == "--no-attach" ]]; then
      wait_fixedwing_result
    else
      echo "v9 已起飞执行；正在进入 tmux，验收器将在仿真窗口中继续运行..."
    fi
    ;;
esac

if [[ "$attach_mode" != "--no-attach" ]]; then
  tmux -L sead-validation attach -t sead-validation
else
  echo "sead-validation 已在后台启动。"
fi
