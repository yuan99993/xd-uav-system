#!/usr/bin/env bash

set -euo pipefail

SCRIPT_PATH="$(readlink -f "$0")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
WORKSPACE_DIR="$(readlink -f "$SCRIPT_DIR/../..")"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <functional-session.yml>" >&2
  echo "Pass a session YAML that starts only the nodes needed on this board." >&2
  exit 2
fi

SESSION_ARGUMENT="$1"
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

SOCKET_NAME="${TMUX_SOCKET_NAME:-$(sed -n 's/^socket_name:[[:space:]]*//p' "$SESSION_PATH" | head -n1)}"
SOCKET_NAME="${SOCKET_NAME:-mrs}"

# This board uses the workspace's ROS packages; it does not run PX4 or Gazebo.
source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"
export DISPLAY="${DISPLAY:-:0}"

# Reuse ROS_MASTER_URI exported by the caller (for example from ~/.bashrc).
ROS_MASTER_URI="${ROS_MASTER_URI:-}"
if [[ -z "$ROS_MASTER_URI" ]]; then
  echo "ROS_MASTER_URI is empty. Export it in ~/.bashrc and start this script from a shell that loaded it." >&2
  exit 2
fi
if [[ ! "$ROS_MASTER_URI" =~ ^http://([^/:]+)(:[0-9]+)?/?$ ]]; then
  echo "Invalid ROS_MASTER_URI: $ROS_MASTER_URI" >&2
  echo "Expected a URI such as http://192.168.1.10:11311" >&2
  exit 2
fi
ROS_MASTER_HOST="${BASH_REMATCH[1]}"
if [[ "$ROS_MASTER_HOST" == "localhost" || "$ROS_MASTER_HOST" == 127.* || "$ROS_MASTER_HOST" == "0.0.0.0" ]]; then
  echo "ROS_MASTER_URI points to this machine/localhost; set it to the other board's address." >&2
  exit 2
fi

# Prefer an explicitly exported ROS_IP. Otherwise use the source address of
# the route to the ROS master, so this board advertises its reachable address.
if [[ -z "${ROS_IP:-}" ]] && command -v ip >/dev/null 2>&1; then
  ROS_IP="$(ip -4 route get "$ROS_MASTER_HOST" 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}' || true)"
fi
if [[ -z "${ROS_IP:-}" ]]; then
  echo "Could not determine this board's ROS_IP. Export ROS_IP and retry." >&2
  exit 2
fi

export ROS_MASTER_URI ROS_IP
unset ROS_HOSTNAME

# Catch session files that would silently switch back to a local ROS master.
if grep -Eq 'ROS_MASTER_URI[^#]*(localhost|127\.0\.0\.1)|ROS_HOSTNAME[^#]*localhost' "$SESSION_PATH"; then
  echo "Session YAML overrides the remote ROS settings with localhost: $SESSION_PATH" >&2
  echo "Remove its localhost ROS_MASTER_URI/ROS_HOSTNAME assignments." >&2
  exit 2
fi
if grep -Eq '^[[:space:]]*-[[:space:]]*roscore:' "$SESSION_PATH"; then
  echo "Session YAML starts a local roscore; remove that window because roscore runs on another board." >&2
  exit 2
fi

cd "$SCRIPT_DIR"

# Update the tmux server environment when it is already running, so newly
# created sessions inherit the remote ROS master settings.
if tmux -L "$SOCKET_NAME" list-sessions >/dev/null 2>&1; then
  tmux -L "$SOCKET_NAME" set-environment -g ROS_MASTER_URI "$ROS_MASTER_URI"
  tmux -L "$SOCKET_NAME" set-environment -g ROS_IP "$ROS_IP"
  tmux -L "$SOCKET_NAME" set-environment -gu ROS_HOSTNAME
fi

if tmux -L "$SOCKET_NAME" has-session -t "$SESSION_NAME" 2>/dev/null; then
  echo "tmux session '$SESSION_NAME' is already running; attaching to it."
  echo "Its existing panes keep their current ROS environment; restart the session to apply changes."
else
  tmuxinator start -p "$SESSION_PATH"
fi

if [[ -z "${TMUX:-}" ]]; then
  exec tmux -L "$SOCKET_NAME" attach-session -t "$SESSION_NAME"
else
  exec tmux detach-client -E "tmux -L $SOCKET_NAME attach-session -t $SESSION_NAME"
fi
