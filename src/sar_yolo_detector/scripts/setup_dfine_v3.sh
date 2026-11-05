#!/usr/bin/env bash
set -euo pipefail
package_dir="$(cd "$(dirname "$0")/.." && pwd)"
if [[ ! -f "$package_dir/requirements-dfine-v3.txt" ]]; then
  package_dir="$(rospack find sar_yolo_detector)"
fi
python_bin="${DFINE_PYTHON:-python3.10}"
runtime_dir="${SAR_DFINE_ENV:-${XDG_DATA_HOME:-$HOME/.local/share}/sar_yolo_detector/dfine-v3}"
torch_index="${DFINE_TORCH_INDEX:-https://download.pytorch.org/whl/cu124}"
"$python_bin" -c 'import sys; assert sys.version_info[:2] == (3, 10), "Use Python 3.10 for this runtime"'
"$python_bin" -m venv "$runtime_dir"
"$runtime_dir/bin/python" -m pip install --upgrade pip
# Fetch the selected torch builds from the PyTorch index. Their declared
# dependencies are resolved by the normal package index in the next step.
"$runtime_dir/bin/python" -m pip install --no-deps torch==2.6.0 torchvision==0.21.0 --index-url "$torch_index"
"$runtime_dir/bin/python" -m pip install -r "$package_dir/requirements-dfine-v3.txt"
# Ultralytics declares opencv-python. Install the same-version contrib wheel
# last so the shared cv2 namespace contains the actual KCF/CSRT implementations.
"$runtime_dir/bin/python" -m pip install --force-reinstall --no-deps opencv-contrib-python==4.10.0.84
"$runtime_dir/bin/python" -m pip check
"$runtime_dir/bin/python" "$package_dir/scripts/check_dfine_v3.py" --device "${DFINE_DEVICE:-cuda}" --load-model
printf 'Runtime ready. Export SAR_DFINE_ENV=%q before roslaunch.\n' "$runtime_dir"
