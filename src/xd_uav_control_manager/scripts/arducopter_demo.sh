#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
DEFAULT_CATKIN_WS="$(cd -- "$PACKAGE_DIR/../../../.." && pwd)"
CATKIN_WS="${CATKIN_WS:-$DEFAULT_CATKIN_WS}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$CATKIN_WS/third_party/ardupilot-Copter-4.7.1}"
GAZEBO_PLUGIN_DIR="${GAZEBO_PLUGIN_DIR:-$CATKIN_WS/third_party/ardupilot_gazebo-classic}"
PARAM_FILE="$PACKAGE_DIR/config/sitl/arducopter.parm"
GAZEBO_PARAM_FILE="$PACKAGE_DIR/config/sitl/arducopter_gazebo_classic.parm"
RUNTIME_DIR="/tmp/xd_uav_arducopter_demo_${UID}"
PID_FILE="$RUNTIME_DIR/pids"
SITL_LOG="$RUNTIME_DIR/arducopter.log"
GAZEBO_LOG="$RUNTIME_DIR/gazebo.log"

SCRIPT_PID=0
GAZEBO_PID=0
SITL_PID=0
ROS_PID=0
CLEANING=false

usage() {
  cat <<'EOF'
Usage: arducopter_demo.sh {start [--headless]|status|stop}

  start   Start Gazebo Classic 11, Copter 4.7.1 SITL and the ROS control stack.
          Use --headless for the built-in quad physics without Gazebo.
          The command stays in the foreground; Ctrl-C stops the whole demo.
  status  Show whether the managed demo processes are still running.
  stop    Stop a demo started by this script from another terminal.
EOF
}

pid_running() {
  local pid="${1:-0}"
  [[ "$pid" =~ ^[0-9]+$ ]] && ((pid > 1)) && kill -0 "$pid" 2>/dev/null
}

pid_matches() {
  local pid="${1:-0}"
  local pattern="${2:-}"
  local command_line=""
  if ! pid_running "$pid" || [[ ! -r "/proc/$pid/cmdline" ]]; then
    return 1
  fi
  command_line="$(tr '\0' ' ' <"/proc/$pid/cmdline")"
  [[ "$command_line" == *"$pattern"* ]]
}

read_pid_file() {
  SCRIPT_PID=0
  GAZEBO_PID=0
  SITL_PID=0
  ROS_PID=0
  if [[ -r "$PID_FILE" ]]; then
    read -r SCRIPT_PID GAZEBO_PID SITL_PID ROS_PID <"$PID_FILE" || true
  fi
}

write_pid_file() {
  printf '%s %s %s %s\n' \
    "$SCRIPT_PID" "$GAZEBO_PID" "$SITL_PID" "$ROS_PID" >"$PID_FILE"
}

terminate_group() {
  local pid="${1:-0}"
  if pid_running "$pid"; then
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in {1..20}; do
      pid_running "$pid" || return 0
      sleep 0.1
    done
    kill -TERM -- "-$pid" 2>/dev/null || true
  fi
}

cleanup() {
  if [[ "$CLEANING" == true ]]; then
    return
  fi
  CLEANING=true
  trap - INT TERM EXIT
  terminate_group "$ROS_PID"
  terminate_group "$SITL_PID"
  terminate_group "$GAZEBO_PID"
  rm -f -- "$PID_FILE"
  echo "ArduCopter demo stopped."
}

show_status() {
  read_pid_file
  local active=false
  if pid_matches "$SCRIPT_PID" "arducopter_demo.sh"; then
    echo "demo supervisor: running (pid $SCRIPT_PID)"
    active=true
  else
    echo "demo supervisor: stopped"
  fi
  if pid_matches "$GAZEBO_PID" "gazebo"; then
    echo "Gazebo Classic: running (pid $GAZEBO_PID)"
    active=true
  else
    echo "Gazebo Classic: stopped"
  fi
  if pid_matches "$SITL_PID" "sim_vehicle.py"; then
    echo "ArduCopter SITL: running (pid $SITL_PID)"
    active=true
  else
    echo "ArduCopter SITL: stopped"
  fi
  if pid_matches "$ROS_PID" "roslaunch"; then
    echo "ROS control stack: running (pid $ROS_PID)"
    active=true
  else
    echo "ROS control stack: stopped"
  fi
  echo "runtime directory: $RUNTIME_DIR"
  [[ "$active" == true ]]
}

start_demo() {
  local physics_mode="gazebo"
  if [[ "${1:-}" == "--headless" ]]; then
    physics_mode="headless"
  elif [[ -n "${1:-}" ]]; then
    echo "Unknown start option: $1" >&2
    usage >&2
    exit 2
  fi

  mkdir -p -- "$RUNTIME_DIR"
  read_pid_file
  if pid_matches "$SCRIPT_PID" "arducopter_demo.sh" ||
      pid_matches "$GAZEBO_PID" "gazebo" ||
      pid_matches "$SITL_PID" "sim_vehicle.py" ||
      pid_matches "$ROS_PID" "roslaunch"; then
    echo "A managed demo is already running. Use 'status' or 'stop'." >&2
    exit 1
  fi

  local ros_setup="/opt/ros/noetic/setup.bash"
  local workspace_setup="$CATKIN_WS/devel/setup.bash"
  local sitl_binary="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
  local sim_vehicle="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
  for required in "$ros_setup" "$workspace_setup" "$sitl_binary" \
      "$sim_vehicle" "$PARAM_FILE"; do
    if [[ ! -e "$required" ]]; then
      echo "Required file is missing: $required" >&2
      exit 1
    fi
  done

  local sitl_frame="quad"
  if [[ "$physics_mode" == "gazebo" ]]; then
    local gazebo_setup="/usr/share/gazebo/setup.sh"
    local gazebo_plugin="$GAZEBO_PLUGIN_DIR/build/libArduPilotPlugin.so"
    local gazebo_world="$GAZEBO_PLUGIN_DIR/worlds/iris_ardupilot.world"
    for required in "$gazebo_setup" "$gazebo_plugin" "$gazebo_world" \
        "$GAZEBO_PARAM_FILE"; do
      if [[ ! -e "$required" ]]; then
        echo "Required Gazebo Classic file is missing: $required" >&2
        exit 1
      fi
    done
    sitl_frame="gazebo-iris"
  fi

  # shellcheck disable=SC1090
  source "$ros_setup"
  # shellcheck disable=SC1090
  source "$workspace_setup"

  SCRIPT_PID=$$
  write_pid_file
  trap cleanup INT TERM EXIT

  : >"$SITL_LOG"
  mkdir -p -- "$RUNTIME_DIR/sitl"
  if [[ "$physics_mode" == "gazebo" ]]; then
    : >"$GAZEBO_LOG"
    set +u
    # shellcheck disable=SC1091
    source /usr/share/gazebo/setup.sh
    set -u
    export GAZEBO_MODEL_PATH="$GAZEBO_PLUGIN_DIR/models:$GAZEBO_PLUGIN_DIR/models_gazebo:${GAZEBO_MODEL_PATH:-}"
    export GAZEBO_RESOURCE_PATH="$GAZEBO_PLUGIN_DIR/worlds:${GAZEBO_RESOURCE_PATH:-}"
    export GAZEBO_PLUGIN_PATH="$GAZEBO_PLUGIN_DIR/build:${GAZEBO_PLUGIN_PATH:-}"
    setsid gazebo --verbose \
      "$GAZEBO_PLUGIN_DIR/worlds/iris_ardupilot.world" \
      >"$GAZEBO_LOG" 2>&1 &
    GAZEBO_PID=$!
    write_pid_file
    sleep 2
    if ! pid_running "$GAZEBO_PID"; then
      echo "Gazebo Classic exited during startup:" >&2
      tail -n 60 "$GAZEBO_LOG" >&2 || true
      exit 1
    fi
    echo "Gazebo Classic 11 started (pid $GAZEBO_PID)."
    echo "Gazebo log: $GAZEBO_LOG"
  fi

  cd -- "$ARDUPILOT_DIR"
  local extra_param_args=()
  if [[ "$physics_mode" == "gazebo" ]]; then
    extra_param_args+=("--add-param-file=$GAZEBO_PARAM_FILE")
  fi
  setsid env -u DISPLAY /usr/bin/python3 "$sim_vehicle" \
    -v ArduCopter -f "$sitl_frame" -N --no-mavproxy -w \
    --use-dir "$RUNTIME_DIR/sitl" \
    --add-param-file="$PARAM_FILE" \
    "${extra_param_args[@]}" >"$SITL_LOG" 2>&1 &
  SITL_PID=$!
  write_pid_file

  sleep 2
  if ! pid_running "$SITL_PID"; then
    echo "ArduCopter SITL exited during startup:" >&2
    tail -n 40 "$SITL_LOG" >&2 || true
    exit 1
  fi

  echo "ArduCopter SITL started (pid $SITL_PID)."
  echo "SITL log: $SITL_LOG"
  echo "Starting MAVROS, localization and control for uav1."
  echo "Keep this terminal open; press Ctrl-C to stop the whole demo."

  setsid roslaunch xd_uav_control_manager \
    arducopter_sitl_system.launch UAV_NAME:=uav1 &
  ROS_PID=$!
  write_pid_file

  local ros_status=0
  wait "$ROS_PID" || ros_status=$?
  ROS_PID=0
  cleanup
  return "$ros_status"
}

stop_demo() {
  read_pid_file
  if pid_matches "$SCRIPT_PID" "arducopter_demo.sh"; then
    kill -TERM "$SCRIPT_PID"
    for _ in {1..50}; do
      pid_running "$SCRIPT_PID" || break
      sleep 0.1
    done
  fi
  if pid_matches "$ROS_PID" "roslaunch"; then
    terminate_group "$ROS_PID"
  fi
  if pid_matches "$SITL_PID" "sim_vehicle.py"; then
    terminate_group "$SITL_PID"
  fi
  if pid_matches "$GAZEBO_PID" "gazebo"; then
    terminate_group "$GAZEBO_PID"
  fi
  rm -f -- "$PID_FILE"
  echo "ArduCopter demo stop requested."
}

case "${1:-}" in
  start)
    start_demo "${2:-}"
    ;;
  status)
    show_status
    ;;
  stop)
    stop_demo
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
