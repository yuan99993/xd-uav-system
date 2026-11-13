#!/usr/bin/env bash

set -euo pipefail

SCRIPT_PATH="$(readlink -f "$0")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
WORKSPACE_DIR="$(readlink -f "$SCRIPT_DIR/../..")"
CONFIG_PATH="${1:-$SCRIPT_DIR/manager.yaml}"

source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"

if [[ -z "${ROS_MASTER_URI:-}" ]]; then
  echo "ROS_MASTER_URI is empty; export the simulation host's ROS master URI first." >&2
  exit 2
fi
if [[ ! "$ROS_MASTER_URI" =~ ^http://([^/:]+)(:[0-9]+)?/?$ ]]; then
  echo "Invalid ROS_MASTER_URI: $ROS_MASTER_URI" >&2
  exit 2
fi
ROS_MASTER_HOST="${BASH_REMATCH[1]}"
if [[ "$ROS_MASTER_HOST" == "localhost" || "$ROS_MASTER_HOST" == "0.0.0.0" || "$ROS_MASTER_HOST" == 127.* ]]; then
  echo "ROS_MASTER_URI must point to the simulation host, not localhost." >&2
  exit 2
fi

if [[ -z "${ROS_IP:-}" ]] && command -v ip >/dev/null 2>&1; then
  ROS_IP="$(ip -4 route get "$ROS_MASTER_HOST" 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") {print $(i+1); exit}}' || true)"
fi
if [[ -z "${ROS_IP:-}" ]]; then
  echo "Could not determine ROS_IP; export the IP address reachable by the simulation host." >&2
  exit 2
fi
export ROS_MASTER_URI ROS_IP
unset ROS_HOSTNAME

exec python3 "$SCRIPT_DIR/manager.py" --config "$CONFIG_PATH"
