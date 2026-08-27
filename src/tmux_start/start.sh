#!/usr/bin/env bash

set -euo pipefail

SCRIPT_PATH="$(readlink -f "$0")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
SESSION_NAME="three_uav_px4"
SOCKET_NAME="mrs"

cd "$SCRIPT_DIR"

# Keep the selected PX4 1.13.2 checkout ahead of other PX4 installations.
source /opt/ros/noetic/setup.bash
source /home/kzy/xd-uavsystem-test/devel/setup.bash
export ROS_PACKAGE_PATH="/home/kzy/PX4-1.13.2/PX4-Autopilot:/home/kzy/PX4-1.13.2/PX4-Autopilot/Tools/sitl_gazebo:${ROS_PACKAGE_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"
unset LIBGL_ALWAYS_SOFTWARE

if tmux -L "$SOCKET_NAME" has-session -t "$SESSION_NAME" 2>/dev/null; then
  echo "tmux session '$SESSION_NAME' is already running; attaching to it."
else
  tmuxinator start -p ./session_four_multi_px4.yml
fi

if [[ -z "${TMUX:-}" ]]; then
  exec tmux -L "$SOCKET_NAME" attach-session -t "$SESSION_NAME"
else
  exec tmux detach-client -E "tmux -L $SOCKET_NAME attach-session -t $SESSION_NAME"
fi
