#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
scenario="${1:-}"
attach_mode="${2:-}"

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
if [[ "$attach_mode" != "--no-attach" ]]; then
  tmux -L sead-validation attach -t sead-validation
else
  echo "sead-validation 已在后台启动。"
fi
