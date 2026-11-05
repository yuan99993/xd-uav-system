"""Atomic mission telemetry output, including WSL/NTFS sharing retries."""
import json
import time
from pathlib import Path

def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + ".tmp")
    encoded=json.dumps(value, ensure_ascii=False, indent=2) + "\n"
    for attempt in range(100):
        try:
            temp.write_text(encoded)
            temp.replace(path)
            return
        except PermissionError:
            # NTFS readers may briefly deny rename/delete sharing across WSL.
            if attempt==99:raise
            time.sleep(.005)
