#!/usr/bin/env bash
set -u

if tmux -L sead-validation has-session -t sead-validation 2>/dev/null; then
  tmux -L sead-validation kill-session -t sead-validation
  echo "sead-validation tmux 会话已停止。"
else
  echo "sead-validation tmux 会话未运行。"
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$script_dir/process_guard.sh" cleanup

echo "清理后核对："
pgrep -af 'roscore|rosmaster|roslaunch|gzserver|gzclient|px4|mavros|sead_onboard|control_manager_node|controller_node|state_estimator' || true
