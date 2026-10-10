#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
runtime="${XD_UAV_TERRAIN_SWARM_RUNTIME:-/tmp/xd_uav_terrain_swarm_sitl_${UID}}"
pid_file="${runtime}/roslaunch.pid"
log_file="${runtime}/roslaunch.log"
gazebo_pid_file="${runtime}/gazebo_roslaunch.pid"
gazebo_log_file="${runtime}/gazebo_roslaunch.log"
gui_pid_file="${runtime}/gazebo_gui.pid"
gui_log_file="${runtime}/gazebo_gui.log"
manage_web="${MANAGE_WEB:-true}"

if [[ "${manage_web}" != true && "${manage_web}" != false ]]; then
  echo "MANAGE_WEB must be true or false" >&2
  exit 2
fi

resolve_setup() {
  local directory candidate resolved=""
  directory="${script_dir}"
  while [[ "${directory}" != / ]]; do
    for candidate in "${directory}/devel/setup.bash" "${directory}/install/setup.bash"; do
      if [[ -d "${directory}/src" && -r "${candidate}" ]]; then
        resolved="${candidate}"
      fi
    done
    directory="$(dirname "${directory}")"
  done
  [[ -n "${resolved}" ]] && { echo "${resolved}"; return 0; }
  return 1
}

setup_ros() {
  source /opt/ros/noetic/setup.bash
  source "${setup_file}"
  export ROS_HOME="${runtime}/ros_home"
  export ROS_HOSTNAME=localhost
  export ROS_MASTER_URI=http://localhost:11311
}

is_running() {
  [[ -s "${pid_file}" ]] || return 1
  local pid args stat
  pid="$(<"${pid_file}")"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  stat="$(ps -p "${pid}" -o stat= 2>/dev/null || true)"
  args="$(ps -p "${pid}" -o args= 2>/dev/null || true)"
  [[ -n "${stat}" && "${stat}" != Z* &&
     "${args}" == *"terrain_fixedwing_swarm_sitl.launch"* ]]
}

is_gazebo_launcher_running() {
  [[ -s "${gazebo_pid_file}" ]] || return 1
  local pid args stat
  pid="$(<"${gazebo_pid_file}")"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  stat="$(ps -p "${pid}" -o stat= 2>/dev/null || true)"
  args="$(ps -p "${pid}" -o args= 2>/dev/null || true)"
  [[ -n "${stat}" && "${stat}" != Z* &&
     "${args}" == *"gazebo_ros"*"empty_world.launch"* ]]
}

is_gazebo_running() {
  is_gazebo_launcher_running || return 1
  local pid
  pid="$(<"${gazebo_pid_file}")"
  ps --ppid "${pid}" -o stat=,args= | awk '
    $1 !~ /^Z/ && $0 ~ /gazebo_ros\/gzserver/ {
      found = 1
    }
    END { exit !found }
  '
}

is_gui_running() {
  [[ -s "${gui_pid_file}" ]] || return 1
  local pid stat
  pid="$(<"${gui_pid_file}")"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  stat="$(ps -p "${pid}" -o stat= 2>/dev/null || true)"
  [[ -n "${stat}" && "${stat}" != Z* ]]
}

fleet_names() {
  timeout -k 1 4 rostopic echo -n 1 /terrain_swarm/fleet 2>/dev/null | \
    awk '/^[[:space:]]*uav_name:/ { gsub(/"/, "", $2); print $2 }'
}

follow_vehicle() {
  local vehicle="${1:-}" camera
  [[ "${vehicle}" =~ ^[a-z][a-z0-9_]*$ ]] || {
    echo "view requires a valid online aircraft name, for example uav1" >&2
    return 2
  }
  is_gazebo_running || {
    echo "Gazebo server is not running" >&2
    return 1
  }
  is_gui_running || {
    echo "Gazebo GUI is not running; start with GUI=true" >&2
    return 1
  }
  fleet_names | grep -Fxq "${vehicle}" || {
    echo "aircraft is not present in the active fleet: ${vehicle}" >&2
    return 1
  }
  camera="$(gz camera -l 2>/dev/null | sed -n '1p')"
  [[ -n "${camera}" ]] || {
    echo "Gazebo GUI camera is not ready" >&2
    return 1
  }
  gz camera -c "${camera}" -f "${vehicle}"
  echo "Gazebo camera is following ${vehicle}"
}

stop_group() {
  local target_file="$1" pid
  [[ -s "${target_file}" ]] || return 0
  pid="$(<"${target_file}")"
  if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -INT -- "-${pid}" 2>/dev/null || true
    for _ in {1..80}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.25
    done
    kill -0 "${pid}" 2>/dev/null && \
      kill -TERM -- "-${pid}" 2>/dev/null || true
  fi
  rm -f "${target_file}"
}

if ! setup_file="$(resolve_setup)"; then
  echo "cannot find workspace setup.bash" >&2
  exit 1
fi

web_root="${XD_UAV_SWARM_WEB_ROOT:-$(readlink -f "${script_dir}/../../../web")}"
web_launcher="${web_root}/fixedwing_mission_web.sh"

stop_all() {
  local -a vehicles=()
  if is_running || is_gazebo_launcher_running; then
    setup_ros
  fi
  if is_running; then
    mapfile -t vehicles < <(fleet_names)
    if [[ "${#vehicles[@]}" -eq 0 ]]; then
      vehicles=(uav1 uav2 uav3)
    fi
    for uav in "${vehicles[@]}"; do
      timeout -k 1 8 rosservice call "/${uav}/control_manager/land" >/dev/null 2>&1 || true
    done
  fi
  if is_running || is_gazebo_launcher_running; then
    timeout -k 1 12 rosnode kill -a >/dev/null 2>&1 || true
  fi
  stop_group "${pid_file}"
  stop_group "${gui_pid_file}"
  stop_group "${gazebo_pid_file}"
  if [[ "${manage_web}" == true ]]; then
    MISSION_WEB_RUNTIME_DIR="${runtime}/web" "${web_launcher}" stop >/dev/null 2>&1 || true
  fi
  echo "fixed-wing SITL runtime stopped"
}

case "${1:-}" in
  start)
    is_running && { echo "fixed-wing SITL runtime already running"; exit 0; }
    if [[ "${manage_web}" == true ]]; then
      [[ -x "${web_launcher}" ]] || { echo "missing Web launcher: ${web_launcher}" >&2; exit 1; }
    fi
    mkdir -p "${runtime}/ros_home/log"
    setup_ros
    if rosnode list >/dev/null 2>&1; then
      echo "an existing ROS graph is using ${ROS_MASTER_URI}; stop it first" >&2
      exit 1
    fi
    if [[ "${manage_web}" == true ]]; then
      MISSION_WEB_RUNTIME_DIR="${runtime}/web" "${web_launcher}" start
    fi
    trap stop_all EXIT INT TERM
    planning_share="$(rospack find xd_uav_planning)"
    default_world="${planning_share}/worlds/davosdorf.world"
    world_file="${GAZEBO_WORLD_FILE:-${default_world}}"
    [[ -r "${world_file}" ]] || {
      echo "Gazebo world is unavailable: ${world_file}" >&2
      exit 1
    }
    if [[ "$(readlink -f "${world_file}")" == "$(readlink -f "${default_world}")" ]]; then
      terrain_model="${planning_share}/models/davosdorf_terrain"
      canonical_dem="$(rospack find terrain_navigation)/../resources/davosdorf.tif"
      python3 "${planning_share}/scripts/tools/verify_gazebo_dem.py" \
        --source "${canonical_dem}" \
        --prepared "${terrain_model}/materials/heightmaps/davosdorf_gazebo.tif" \
        --manifest "${terrain_model}/terrain-manifest.json" \
        --model-sdf "${terrain_model}/model.sdf"
    fi
    export GAZEBO_MODEL_PATH="${planning_share}/models:$(rospack find px4)/Tools/sitl_gazebo/models:${GAZEBO_MODEL_PATH:-}"
    # GDAL 3 follows authority-axis order by default, while Gazebo Classic's
    # DEM loader predates that change and expects traditional GIS X/Y order.
    # Without both settings EPSG:21781 is interpreted as lat/lon-swapped and
    # Gazebo silently stretches the terrain horizontally.
    export OSR_DEFAULT_AXIS_MAPPING_STRATEGY=TRADITIONAL_GIS_ORDER
    export OGR_CT_FORCE_TRADITIONAL_GIS_ORDER=YES
    # Starting gzclient concurrently with gzserver is unreliable on WSL and
    # can leave a healthy headless simulation behind a crashed GUI.  Bring up
    # the server first, then attach the optional client after its ROS API is
    # ready.
    setsid roslaunch gazebo_ros empty_world.launch \
      world_name:="${world_file}" gui:=false paused:=false \
      </dev/null >"${gazebo_log_file}" 2>&1 &
    echo "$!" >"${gazebo_pid_file}"
    gazebo_ready=false
    for _ in {1..120}; do
      is_gazebo_launcher_running || break
      if is_gazebo_running && \
          timeout -k 1 2 rosservice call /gazebo/get_world_properties \
          >/dev/null 2>&1; then
        gazebo_ready=true
        break
      fi
      sleep 0.25
    done
    [[ "${gazebo_ready}" == true ]] || {
      echo "Gazebo did not become ready; see ${gazebo_log_file}" >&2
      exit 1
    }
    if [[ "${GUI:-false}" == true ]]; then
      # The gazebo_ros wrapper also injects the ROS API plugin into gzclient.
      # That GUI-side plugin is unnecessary for rendering and can segfault on
      # WSL while the server-side API is still healthy.
      setsid gzclient \
        </dev/null >"${gui_log_file}" 2>&1 &
      echo "$!" >"${gui_pid_file}"
      sleep 3
      is_gui_running || {
        echo "Gazebo GUI exited during startup; see ${gui_log_file}" >&2
        exit 1
      }
    fi
    run_acceptance="${RUN_ACCEPTANCE:-false}"
    auto_spawn_initial="${AUTO_SPAWN_INITIAL:-${run_acceptance}}"
    spatial_only_blocks_release="${SPATIAL_ONLY_BLOCKS_RELEASE:-false}"
    setsid roslaunch xd_uav_planning terrain_fixedwing_swarm_sitl.launch \
      start_gazebo:=false gui:="${GUI:-false}" \
      run_acceptance:="${run_acceptance}" \
      auto_spawn_initial:="${auto_spawn_initial}" \
      spatial_only_blocks_release:="${spatial_only_blocks_release}" \
      </dev/null >"${log_file}" 2>&1 &
    echo "$!" >"${pid_file}"
    sleep 3
    is_running || { echo "SITL launch failed; see ${log_file}" >&2; exit 1; }
    if [[ "${GUI:-false}" == true && "${auto_spawn_initial}" == true ]]; then
      for _ in {1..80}; do
        if timeout -k 1 2 rosservice call /gazebo/get_world_properties \
            2>/dev/null | grep -q -- '- uav2'; then
          follow_vehicle uav2
          break
        fi
        sleep 0.25
      done
    fi
    trap - EXIT INT TERM
    echo "fixed-wing SITL runtime started"
    if [[ "${manage_web}" == true ]]; then
      echo "Web map and planning: http://127.0.0.1:5173/"
      echo "Web mission control: http://127.0.0.1:5173/swarm.html"
    else
      echo "Web: externally managed; start src/web/fixedwing_mission_web.sh separately"
    fi
    echo "ROS log: ${log_file}"
    if [[ "${auto_spawn_initial}" != true ]]; then
      echo "Fleet: empty; add aircraft from the Web console"
    fi
    echo "View: '$0 view AIRCRAFT_NAME' (for example: uav1)"
    echo "Use '$0 status' and '$0 logs'; stop with '$0 stop'."
    ;;
  status)
    vehicles=()
    if ! is_running; then
      echo "fixed-wing SITL runtime: stopped"
      exit 1
    fi
    setup_ros
    echo "fixed-wing SITL runtime: running (PID $(<"${pid_file}"))"
    if is_gazebo_running; then
      echo "Gazebo: running (PID $(<"${gazebo_pid_file}"))"
    else
      echo "Gazebo: stopped"
    fi
    if is_gui_running; then
      echo "Gazebo GUI: running (PID $(<"${gui_pid_file}"))"
      echo "View: $0 view AIRCRAFT_NAME"
    else
      echo "Gazebo GUI: stopped"
    fi
    if [[ "${manage_web}" == true ]]; then
      MISSION_WEB_RUNTIME_DIR="${runtime}/web" "${web_launcher}" status || true
    else
      echo "Web: externally managed"
    fi
    mapfile -t vehicles < <(fleet_names)
    for uav in "${vehicles[@]}"; do
      echo "[${uav}]"
      timeout -k 1 4 rostopic echo -n 1 "/${uav}/mavros/state" 2>/dev/null | \
        grep -E 'connected:|armed:|mode:' || true
    done
    timeout -k 1 4 rostopic echo -n 1 "/terrain_swarm/sitl_acceptance/result" 2>/dev/null || true
    ;;
  logs)
    tail -n 120 "${log_file}"
    ;;
  view)
    follow_vehicle "${2:-}"
    ;;
  stop)
    stop_all
    ;;
  restart)
    restart_gui="${GUI:-false}"
    is_gui_running && restart_gui=true
    # Give the HTTP response that requested this detached restart time to
    # reach the browser before the backend process is stopped.
    sleep 1
    stop_all
    GUI="${restart_gui}" "$0" start
    ;;
  *)
    echo "usage: $0 {start|status|logs|view AIRCRAFT_NAME|stop|restart}" >&2
    exit 2
    ;;
esac
