#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
repo_root="$(dirname "${script_dir}")"
web_launcher="${repo_root}/src/web/fixedwing_mission_web.sh"
runtime_launcher="${repo_root}/src/xd_uav_planning/scripts/runtime/fixedwing_sitl_runtime.sh"

usage() {
  cat <<'EOF'
Usage:
  scripts/fixedwing_mission.sh start [--gui]
  scripts/fixedwing_mission.sh web {start|status|logs|stop}
  scripts/fixedwing_mission.sh sim {start|status|logs|stop|restart} [--gui]
  scripts/fixedwing_mission.sh view AIRCRAFT_NAME
  scripts/fixedwing_mission.sh {status|logs|stop}
EOF
}

run_sim() {
  local action="$1"
  shift
  local gui=false
  if [[ "${1:-}" == --gui ]]; then
    gui=true
    shift
  fi
  [[ "$#" -eq 0 ]] || { usage >&2; return 2; }
  GUI="${gui}" "${runtime_launcher}" "${action}"
}

case "${1:-}" in
  start)
    shift
    gui=false
    if [[ "${1:-}" == --gui ]]; then
      gui=true
      shift
    fi
    [[ "$#" -eq 0 ]] || { usage >&2; exit 2; }
    web_started=false
    if ! "${web_launcher}" status >/dev/null 2>&1; then
      "${web_launcher}" start
      web_started=true
    fi
    if ! GUI="${gui}" "${runtime_launcher}" start; then
      [[ "${web_started}" == false ]] || "${web_launcher}" stop
      exit 1
    fi
    echo "Open http://127.0.0.1:5173/"
    ;;
  web)
    shift
    case "${1:-}" in
      start|status|logs|stop) "${web_launcher}" "$1" ;;
      *) usage >&2; exit 2 ;;
    esac
    ;;
  sim)
    shift
    case "${1:-}" in
      start|status|logs|stop|restart)
        action="$1"
        shift
        run_sim "${action}" "$@"
        ;;
      *) usage >&2; exit 2 ;;
    esac
    ;;
  view)
    [[ -n "${2:-}" && -z "${3:-}" ]] || { usage >&2; exit 2; }
    "${runtime_launcher}" view "$2"
    ;;
  status)
    web_result=0
    sim_result=0
    echo "[Web]"
    "${web_launcher}" status || web_result=$?
    echo "[Simulation / execution]"
    "${runtime_launcher}" status || sim_result=$?
    (( web_result == 0 && sim_result == 0 ))
    ;;
  logs)
    echo "[Web]"
    "${web_launcher}" logs
    echo "[Simulation / execution]"
    "${runtime_launcher}" logs
    ;;
  stop)
    "${runtime_launcher}" stop || true
    "${web_launcher}" stop || true
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
