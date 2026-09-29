#!/usr/bin/env bash

set -euo pipefail

SCRIPT_PATH="$(readlink -f "$0")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
XD_UAV_REPO_ROOT="$(readlink -f "$SCRIPT_DIR/../..")"
CATKIN_WORKSPACE="$(readlink -f "$XD_UAV_REPO_ROOT/../..")"
PX4_AUTOPILOT_ROOT="${PX4_AUTOPILOT_ROOT:-$(readlink -f "$CATKIN_WORKSPACE/../PX4-Autopilot")}"
if [[ -z "${SAR_PYTHON_EXECUTABLE:-}" ]]; then
  if [[ -x "$XD_UAV_REPO_ROOT/.venv-sar-gpu/bin/python" ]]; then
    SAR_PYTHON_EXECUTABLE="$XD_UAV_REPO_ROOT/.venv-sar-gpu/bin/python"
  else
    SAR_PYTHON_EXECUTABLE="/usr/bin/python3"
  fi
fi
SESSION_ARGUMENT="${1:-$SCRIPT_DIR/session_four_fixed_px4.yml}"
SESSION_PATH="$(readlink -f "$SESSION_ARGUMENT")"
if [[ ! -f "$SESSION_PATH" ]]; then
  echo "tmux session file does not exist: $SESSION_ARGUMENT" >&2
  exit 2
fi
SESSION_NAME="$(sed -n 's/^name:[[:space:]]*//p' "$SESSION_PATH" | head -n1)"
if [[ -z "$SESSION_NAME" ]]; then
  echo "tmux session file has no top-level name: $SESSION_PATH" >&2
  exit 2
fi
SOCKET_NAME="$(sed -n "s/^socket_name:[[:space:]]*//p" "$SESSION_PATH" | head -n1)"
if [[ -z "$SOCKET_NAME" ]]; then
  echo "tmux session file has no top-level socket_name: $SESSION_PATH" >&2
  exit 2
fi

for required_file in /opt/ros/noetic/setup.bash "$CATKIN_WORKSPACE/devel/setup.bash" "$PX4_AUTOPILOT_ROOT/package.xml"; do
  if [[ ! -f "$required_file" ]]; then
    echo "required file does not exist: $required_file" >&2
    exit 2
  fi
done

while IFS= read -r px4_launch_file; do
  [[ -z "$px4_launch_file" ]] && continue
  resolved_px4_launch="$(find "$PX4_AUTOPILOT_ROOT" -type f -name "$px4_launch_file" -print -quit)"
  if [[ -z "$resolved_px4_launch" ]]; then
    echo "selected session requires a PX4 launch file not present under PX4_AUTOPILOT_ROOT: $px4_launch_file" >&2
    echo "point PX4_AUTOPILOT_ROOT to the matching PX4 checkout before starting this session" >&2
    exit 2
  fi
done < <(sed -nE "s/.*roslaunch[[:space:]]+px4[[:space:]]+([^[:space:]]+\.launch).*/\1/p" "$SESSION_PATH" | sort -u)

if [[ ! -x "$SAR_PYTHON_EXECUTABLE" ]]; then
  echo "SAR Python executable is not executable: $SAR_PYTHON_EXECUTABLE" >&2
  exit 2
fi
if ! command -v tmuxinator >/dev/null 2>&1; then
  echo "tmuxinator is not installed or not on PATH" >&2
  exit 2
fi

export XD_UAV_REPO_ROOT CATKIN_WORKSPACE PX4_AUTOPILOT_ROOT
export SAR_PYTHON_EXECUTABLE

cd "$SCRIPT_DIR"

# Keep the selected PX4 1.13.2 checkout ahead of other PX4 installations.
source /opt/ros/noetic/setup.bash
source "$CATKIN_WORKSPACE/devel/setup.bash"
export ROS_PACKAGE_PATH="$PX4_AUTOPILOT_ROOT:$PX4_AUTOPILOT_ROOT/Tools/sitl_gazebo:${ROS_PACKAGE_PATH:-}"
export DISPLAY="${DISPLAY:-:0}"
unset LIBGL_ALWAYS_SOFTWARE

if tmux -L "$SOCKET_NAME" has-session -t "$SESSION_NAME" 2>/dev/null; then
  echo "tmux session '$SESSION_NAME' is already running; attaching to it."
else
  tmuxinator start -p "$SESSION_PATH"
fi

if [[ -z "${TMUX:-}" ]]; then
  exec tmux -L "$SOCKET_NAME" attach-session -t "$SESSION_NAME"
else
  exec tmux detach-client -E "tmux -L $SOCKET_NAME attach-session -t $SESSION_NAME"
fi
