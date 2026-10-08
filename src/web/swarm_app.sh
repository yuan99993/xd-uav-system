#!/usr/bin/env bash
set -euo pipefail

WEB_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRONTEND_ROOT="${WEB_ROOT}/frontend"
BACKEND_PORT="${MISSION_WEB_BACKEND_PORT:-${SWARM_BACKEND_PORT:-8000}}"
FRONTEND_PORT="${MISSION_WEB_FRONTEND_PORT:-${SWARM_FRONTEND_PORT:-5173}}"
RUNTIME_DIR="${MISSION_WEB_RUNTIME_DIR:-${SWARM_RUNTIME_DIR:-/tmp/xd-uav-fixedwing-mission-web-${UID}}}"
BACKEND_PID_FILE="${RUNTIME_DIR}/backend.pid"
FRONTEND_PID_FILE="${RUNTIME_DIR}/frontend.pid"
BACKEND_LOG="${RUNTIME_DIR}/backend.log"
FRONTEND_LOG="${RUNTIME_DIR}/frontend.log"

usage() {
  echo "Usage: $0 {start|status|logs|stop}"
}

pid_alive() {
  local pid_file="$1"
  [[ -s "${pid_file}" ]] || return 1
  local pid
  pid="$(<"${pid_file}")"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 1
  kill -0 "${pid}" 2>/dev/null
}

pid_matches_component() {
  local name="$1"
  local pid="$2"
  local args
  args="$(ps -p "${pid}" -o args= 2>/dev/null || true)"
  case "${name}" in
    backend)
      [[ "${args}" == *" -m uvicorn swarm_app.main:app "* &&
         "${args}" == *"--app-dir ${WEB_ROOT}/backend"* &&
         "${args}" == *"--host 127.0.0.1 --port ${BACKEND_PORT}"* ]]
      ;;
    frontend)
      [[ "${args}" == *"${FRONTEND_ROOT}/node_modules/.bin/vite"* &&
         "${args}" == *"--config ${FRONTEND_ROOT}/swarm.vite.config.js"* &&
         "${args}" == *"--host 127.0.0.1 --port ${FRONTEND_PORT}"* ]]
      ;;
    *) return 1 ;;
  esac
}

find_component_pid() {
  local name="$1"
  local pid
  while read -r pid; do
    [[ "${pid}" =~ ^[0-9]+$ ]] || continue
    if pid_matches_component "${name}" "${pid}"; then
      echo "${pid}"
      return 0
    fi
  done < <(ps -eo pid=)
  return 1
}

# Older launchers and interrupted cleanup can leave a healthy project process
# behind after its PID file disappears. Recover only processes whose complete
# command line matches this Web root, component and configured port.
component_running() {
  local name="$1"
  local pid_file="$2"
  local pid=""
  if pid_alive "${pid_file}"; then
    pid="$(<"${pid_file}")"
    if pid_matches_component "${name}" "${pid}"; then
      return 0
    fi
    rm -f "${pid_file}"
  fi
  pid="$(find_component_pid "${name}" || true)"
  [[ -n "${pid}" ]] || return 1
  mkdir -p "${RUNTIME_DIR}"
  echo "${pid}" >"${pid_file}"
}

resolve_python() {
  if [[ -n "${SWARM_PYTHON:-}" ]]; then
    [[ -x "${SWARM_PYTHON}" ]] || {
      echo "SWARM_PYTHON is not executable: ${SWARM_PYTHON}" >&2
      return 1
    }
    PYTHON_BIN="${SWARM_PYTHON}"
    return
  fi

  local conda_bin="${SWARM_CONDA:-}"
  if [[ -z "${conda_bin}" ]] && command -v conda >/dev/null 2>&1; then
    conda_bin="$(command -v conda)"
  fi
  if [[ -z "${conda_bin}" ]]; then
    local task_user_home
    task_user_home="${HOME:-}"
    for candidate in \
      "${task_user_home}/miniconda3/bin/conda" \
      "${task_user_home}/anaconda3/bin/conda"; do
      if [[ -x "${candidate}" ]]; then
        conda_bin="${candidate}"
        break
      fi
    done
  fi

  if [[ -x "${conda_bin}" ]]; then
    PYTHON_BIN="$("${conda_bin}" run -n terrain-nav-web python -c 'import sys; print(sys.executable)' 2>/dev/null | tail -n 1)"
    [[ -x "${PYTHON_BIN}" ]] && return
  fi

  echo "Named environment terrain-nav-web was not found. Create environment.yml or set SWARM_PYTHON." >&2
  return 1
}

check_runtime() {
  resolve_python
  "${PYTHON_BIN}" -c 'import fastapi, numpy, osgeo, pydantic, uvicorn' >/dev/null
  VITE_BIN="${FRONTEND_ROOT}/node_modules/.bin/vite"
  [[ -x "${VITE_BIN}" ]] || {
    echo "Frontend dependencies are absent. Run the documented npm ci once; no package was installed." >&2
    return 1
  }
}

wait_for_url() {
  local url="$1"
  local pid_file="$2"
  local attempts=40
  while (( attempts > 0 )); do
    pid_alive "${pid_file}" || return 1
    if curl --noproxy '*' --silent --fail --max-time 1 "${url}" \
        >/dev/null 2>&1; then
      pid_alive "${pid_file}" || return 1
      return 0
    fi
    sleep 0.25
    attempts=$((attempts - 1))
  done
  return 1
}

stop_one() {
  local name="$1"
  local pid_file="$2"
  if ! component_running "${name}" "${pid_file}"; then
    rm -f "${pid_file}"
    echo "${name}: stopped"
    return
  fi

  local pid
  pid="$(<"${pid_file}")"
  kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  for _ in {1..20}; do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.1
  done
  rm -f "${pid_file}"
  echo "${name}: stopped"
}

start_all() {
  if component_running backend "${BACKEND_PID_FILE}" ||
      component_running frontend "${FRONTEND_PID_FILE}"; then
    echo "Fixed-wing mission Web is already running; use '$0 status'." >&2
    return 1
  fi
  check_runtime
  mkdir -p "${RUNTIME_DIR}"

  setsid "${PYTHON_BIN}" -m uvicorn swarm_app.main:app \
    --app-dir "${WEB_ROOT}/backend" --host 127.0.0.1 --port "${BACKEND_PORT}" \
    >"${BACKEND_LOG}" 2>&1 &
  echo "$!" >"${BACKEND_PID_FILE}"

  setsid "${VITE_BIN}" --config "${FRONTEND_ROOT}/swarm.vite.config.js" \
    --host 127.0.0.1 --port "${FRONTEND_PORT}" \
    >"${FRONTEND_LOG}" 2>&1 &
  echo "$!" >"${FRONTEND_PID_FILE}"

  if ! wait_for_url "http://127.0.0.1:${BACKEND_PORT}/api/swarm/health" "${BACKEND_PID_FILE}"; then
    echo "Backend did not become healthy; see ${BACKEND_LOG}." >&2
    stop_all
    return 1
  fi
  if ! wait_for_url "http://127.0.0.1:${FRONTEND_PORT}/" "${FRONTEND_PID_FILE}" ||
      ! wait_for_url "http://127.0.0.1:${FRONTEND_PORT}/swarm.html" "${FRONTEND_PID_FILE}"; then
    echo "Frontend did not become healthy; see ${FRONTEND_LOG}." >&2
    stop_all
    return 1
  fi

  echo "Fixed-wing mission Web is ready:"
  echo "  map and planning: http://127.0.0.1:${FRONTEND_PORT}/"
  echo "  mission control:  http://127.0.0.1:${FRONTEND_PORT}/swarm.html"
  echo "  API:  http://127.0.0.1:${BACKEND_PORT}/api/swarm/health"
  echo "  logs: ${RUNTIME_DIR}"
}

status_all() {
  local result=0
  if component_running backend "${BACKEND_PID_FILE}"; then
    echo "backend: running (PID $(<"${BACKEND_PID_FILE}"), port ${BACKEND_PORT})"
  else
    echo "backend: stopped"
    result=1
  fi
  if component_running frontend "${FRONTEND_PID_FILE}"; then
    echo "frontend: running (PID $(<"${FRONTEND_PID_FILE}"), port ${FRONTEND_PORT})"
  else
    echo "frontend: stopped"
    result=1
  fi
  return "${result}"
}

logs_all() {
  mkdir -p "${RUNTIME_DIR}"
  touch "${BACKEND_LOG}" "${FRONTEND_LOG}"
  tail -n 80 "${BACKEND_LOG}" "${FRONTEND_LOG}"
}

stop_all() {
  stop_one frontend "${FRONTEND_PID_FILE}"
  stop_one backend "${BACKEND_PID_FILE}"
}

case "${1:-}" in
  start) start_all ;;
  status) status_all ;;
  logs) logs_all ;;
  stop) stop_all ;;
  *) usage; exit 2 ;;
esac
