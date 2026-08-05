#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scenario="${1:-}"
attach_mode="${2:-}"

wait_flight_ready() {
  local count="$1"
  local timeout_s="${SEAD_VALIDATION_READY_TIMEOUT:-180}"
  local stable_required="${SEAD_VALIDATION_READY_STABLE_COUNT:-3}"
  local deadline=$((SECONDS + timeout_s))
  local stable=0
  local pending=""
  local i state

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

case "$scenario" in
  v3|v4|v5|v6|v7|v8) ;;
  *)
    echo "用法: $0 {v3|v4|v5|v6|v7|v8}"
    echo "v3=单机  v4=两机并发  v5=三机编队  v6=Airspace  v7=DPGA  v8=SimpleStrike"
    exit 2
    ;;
esac

if tmux -L sead-validation has-session -t sead-validation 2>/dev/null; then
  echo "sead-validation 已在运行。先执行 ./kill.sh，或直接连接："
  echo "tmux -L sead-validation attach -t sead-validation"
  exit 1
fi

"$script_dir/process_guard.sh" check

export SEAD_VALIDATION_SCENARIO="$scenario"
export SEAD_VALIDATION_DIR="$script_dir"
export SEAD_VALIDATION_RUNTIME="/home/promise/catkin_ws/src/xd-uavsystem-test/.codex-tmp/sead_validation_offsets.env"
export SEAD_VALIDATION_RUN_ID="$(date +%s)-$$"

if [[ "$scenario" == "v5" ]]; then
  rm -f "$SEAD_VALIDATION_RUNTIME"
fi

cd "$script_dir"
tmuxinator start -p ./session.yml </dev/null

case "$scenario" in
  v3) wait_flight_ready 1 ;;
  v4) wait_flight_ready 2 ;;
  v5) wait_flight_ready 3 ;;
esac

if [[ "$attach_mode" != "--no-attach" ]]; then
  tmux -L sead-validation attach -t sead-validation
else
  echo "sead-validation 已在后台启动。"
fi
