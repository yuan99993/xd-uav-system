import runpy
import sys
from pathlib import Path

viewer = Path(__file__).with_name("fusion_web_preview_display.py")
sys.argv = [str(viewer), *sys.argv[2:]]
runpy.run_path(str(viewer), run_name="__main__")
