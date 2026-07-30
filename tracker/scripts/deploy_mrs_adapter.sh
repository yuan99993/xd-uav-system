#!/usr/bin/env bash
#
# Prepare tracker/follower for a different ROS1 Noetic + PX4/MAVROS/MRS
# workspace without editing either package's versioned configuration.

# ROS Noetic's setup scripts read ROS_DISTRO before assigning its default,
# so nounset can only be enabled after the ROS environment is loaded.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
ROS_SETUP="/opt/ros/noetic/setup.bash"

UAV_NAME="uav1"
MAVROS_NS=""
TRACKER_ERROR_TOPIC="/tracker_node/normalized_error"
ENABLE_MAVROS_OUTPUT="true"
ENABLE_MRS_OUTPUT="false"
MRS_VELOCITY_TOPIC=""
MRS_FRAME=""
VISUALIZATION_FRAME="map"
BUILD_WORKSPACE="true"
VERIFY_ONLY="false"
START_STACK="false"

usage() {
  cat <<'EOF'
Usage:
  bash src/tracker/scripts/deploy_mrs_adapter.sh [options]

Creates a local endpoint mapping at <workspace>/.mrs_adapter/follower_adapter.yaml,
optionally builds tracker/follower, and checks their ROS interfaces.

Options:
  --uav-name NAME              MRS UAV name (default: uav1)
  --mavros-ns TOPIC           MAVROS namespace (default: /mavros)
  --tracker-error-topic TOPIC Tracker NormalizedError topic
                              (default: /tracker_node/normalized_error)
  --enable-mavros-output BOOL Publish MAVROS velocity setpoints (default: true)
  --enable-mrs-output BOOL    Publish MRS VelocityReferenceStamped (default: false)
  --mrs-velocity-topic TOPIC  MRS velocity input topic
                              (default: /<uav>/control_manager/velocity_reference_in)
  --mrs-frame FRAME           MRS velocity frame (default: <uav>/fcu)
  --visualization-frame FRAME RViz frame_id (default: map)
  --no-build                  Skip catkin build
  --verify                    Build/check only; do not start nodes
  --launch                    After success, start tracker and follower together
  -h, --help                  Show this help

Examples:
  # PX4 SITL/MAVROS at /mavros, no MRS controller transport:
  bash src/tracker/scripts/deploy_mrs_adapter.sh

  # MRS UAV named uav3 with MAVROS under /uav3/mavros:
  bash src/tracker/scripts/deploy_mrs_adapter.sh \
    --uav-name uav3 --mavros-ns /uav3/mavros --enable-mrs-output true
EOF
}

require_bool() {
  case "$1" in
    true|false) ;;
    *)
      echo "Error: expected true or false, got '$1'." >&2
      exit 2
      ;;
  esac
}

require_absolute_topic() {
  if [[ "$1" != /* ]]; then
    echo "Error: ROS topic/namespace must start with '/': '$1'." >&2
    exit 2
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --uav-name) UAV_NAME="$2"; shift 2 ;;
    --mavros-ns) MAVROS_NS="$2"; shift 2 ;;
    --tracker-error-topic) TRACKER_ERROR_TOPIC="$2"; shift 2 ;;
    --enable-mavros-output) ENABLE_MAVROS_OUTPUT="$2"; shift 2 ;;
    --enable-mrs-output) ENABLE_MRS_OUTPUT="$2"; shift 2 ;;
    --mrs-velocity-topic) MRS_VELOCITY_TOPIC="$2"; shift 2 ;;
    --mrs-frame) MRS_FRAME="$2"; shift 2 ;;
    --visualization-frame) VISUALIZATION_FRAME="$2"; shift 2 ;;
    --no-build) BUILD_WORKSPACE="false"; shift ;;
    --verify) VERIFY_ONLY="true"; shift ;;
    --launch) START_STACK="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "Error: unknown option '$1'." >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Error: ROS Noetic was not found at ${ROS_SETUP}." >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE}/src/tracker/package.xml" ||
      ! -f "${WORKSPACE}/src/follower/package.xml" ]]; then
  echo "Error: this script must remain under <workspace>/src/tracker/scripts/." >&2
  exit 1
fi

MAVROS_NS="${MAVROS_NS:-/${UAV_NAME}/mavros}"
if [[ "${UAV_NAME}" == "uav1" && "${MAVROS_NS}" == "/uav1/mavros" ]]; then
  # Preserve the common standalone PX4/MAVROS default unless the user asks
  # for a per-UAV MAVROS namespace explicitly.
  MAVROS_NS="/mavros"
fi
MAVROS_NS="${MAVROS_NS%/}"
MRS_VELOCITY_TOPIC="${MRS_VELOCITY_TOPIC:-/${UAV_NAME}/control_manager/velocity_reference_in}"
MRS_FRAME="${MRS_FRAME:-${UAV_NAME}/fcu}"

require_bool "${ENABLE_MAVROS_OUTPUT}"
require_bool "${ENABLE_MRS_OUTPUT}"
require_absolute_topic "${MAVROS_NS}"
require_absolute_topic "${TRACKER_ERROR_TOPIC}"
require_absolute_topic "${MRS_VELOCITY_TOPIC}"

if [[ "${ENABLE_MAVROS_OUTPUT}" != "true" && "${ENABLE_MRS_OUTPUT}" != "true" ]]; then
  echo "Error: enable at least one velocity output transport." >&2
  exit 2
fi

source "${ROS_SETUP}"
set -u

if [[ "${BUILD_WORKSPACE}" == "true" ]]; then
  echo "Building tracker and follower in ${WORKSPACE} ..."
  cd "${WORKSPACE}"
  catkin build tracker follower
fi

if [[ ! -f "${WORKSPACE}/devel/setup.bash" ]]; then
  echo "Error: ${WORKSPACE}/devel/setup.bash is missing. Run catkin build first." >&2
  exit 1
fi
source "${WORKSPACE}/devel/setup.bash"

for package in tracker follower mavros_msgs; do
  if ! rospack find "${package}" >/dev/null 2>&1; then
    echo "Error: required ROS package '${package}' is unavailable in this environment." >&2
    exit 1
  fi
done

if [[ "${ENABLE_MRS_OUTPUT}" == "true" ]]; then
  if ! rosmsg show mrs_msgs/VelocityReferenceStamped >/dev/null 2>&1; then
    echo "Error: MRS output is enabled, but mrs_msgs/VelocityReferenceStamped is unavailable." >&2
    exit 1
  fi
fi

ADAPTER_DIR="${WORKSPACE}/.mrs_adapter"
ADAPTER_CONFIG="${ADAPTER_DIR}/follower_adapter.yaml"
mkdir -p "${ADAPTER_DIR}"
cat > "${ADAPTER_CONFIG}" <<EOF
# Generated by deploy_mrs_adapter.sh. This workspace-local file is safe to edit.
tracker_error_topic: "${TRACKER_ERROR_TOPIC}"
mavros_namespace: "${MAVROS_NS}"
enable_mavros_output: ${ENABLE_MAVROS_OUTPUT}
mavros_velocity_topic: "${MAVROS_NS}/setpoint_velocity/cmd_vel"
enable_mrs_velocity_reference: ${ENABLE_MRS_OUTPUT}
mrs_velocity_reference_topic: "${MRS_VELOCITY_TOPIC}"
mrs_velocity_frame: "${MRS_FRAME}"
visualization_frame: "${VISUALIZATION_FRAME}"
EOF

echo
echo "Adapter configuration created:"
echo "  ${ADAPTER_CONFIG}"
echo "Tracker input:"
echo "  ${TRACKER_ERROR_TOPIC}"
echo "MAVROS velocity output:"
echo "  ${MAVROS_NS}/setpoint_velocity/cmd_vel (enabled: ${ENABLE_MAVROS_OUTPUT})"
echo "MRS velocity output:"
echo "  ${MRS_VELOCITY_TOPIC} (enabled: ${ENABLE_MRS_OUTPUT}, frame: ${MRS_FRAME})"

if [[ "${VERIFY_ONLY}" == "true" ]]; then
  echo
  echo "Static deployment verification passed."
  exit 0
fi

if [[ "${START_STACK}" == "true" ]]; then
  echo
  echo "Starting tracker and follower. This only starts ROS nodes; it does not arm PX4."
  exec roslaunch follower tracker_follower_mrs_adapter.launch \
    adapter_config:="${ADAPTER_CONFIG}"
fi

echo
echo "Start commands:"
echo "  source /opt/ros/noetic/setup.bash"
echo "  source \"${WORKSPACE}/devel/setup.bash\""
echo "  roslaunch tracker tracker.launch"
echo "  roslaunch follower follower_mrs_adapter.launch adapter_config:=\"${ADAPTER_CONFIG}\""
echo
echo "Or start both nodes together:"
echo "  roslaunch follower tracker_follower_mrs_adapter.launch adapter_config:=\"${ADAPTER_CONFIG}\""
