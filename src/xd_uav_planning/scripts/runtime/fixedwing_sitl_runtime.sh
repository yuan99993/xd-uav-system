#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
demo_launcher="${script_dir}/../demo/terrain_swarm_sitl_demo.sh"

export MANAGE_WEB=false
exec "${demo_launcher}" "$@"
