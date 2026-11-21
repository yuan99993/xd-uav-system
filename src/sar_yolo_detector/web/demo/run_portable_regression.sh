#!/usr/bin/env bash
set -euo pipefail
# Run from a sourced ROS workspace with the model environment installed.
package_dir="$(rospack find sar_yolo_detector)"
model_python="${DFINE_PYTHON:-python3}"
ros_python="${ROS_PYTHON:-/usr/bin/python3}"
"$ros_python" "$package_dir/web/demo/test_soft_tracks.py"
PYTHONPATH="$package_dir/test:$package_dir/python:${PYTHONPATH:-}" "$model_python" -m unittest \
  test_box_fusion test_selected_detector_gate test_reselection_refresh
