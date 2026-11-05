"""UE capture execution locks; no training or workspace-root imports."""
import contextlib
import fcntl
import json
import os
import time
from pathlib import Path
from xd_uav_track.mission_io import write_json

ROOT=Path(os.environ.get("SAR_AIRGROUND_WORKLOAD_ROOT", str(Path(os.environ.get("ROS_HOME",str(Path.home()/".ros")))/"sar_airground")))

@contextlib.contextmanager
def workload_lock(kind, shared=False, lock_path=None):
    # Same lock for training and UE bridge. Download, audit and offline CPU
    # preparation do not occupy the GPU. A separately started UE editor must
    # still be closed manually before training.
    if kind=='train' and lock_path is None:
        registry=ROOT/'runtime/ue_editor_sources.json'
        for name in json.loads(registry.read_text()) if registry.exists() else []:
            source=Path(name)
            try:heartbeat=json.loads(source.read_text())['heartbeat']
            except (OSError,ValueError,KeyError):continue
            if abs(time.time()-heartbeat)<15:
                raise RuntimeError('Close the UE editor before starting training: '+str(source))
    lock = Path(lock_path) if lock_path else ROOT / "runtime" / "airground_gpu.lock"
    lock.parent.mkdir(parents=True,exist_ok=True)
    with lock.open("a+") as stream:
        try:
            fcntl.flock(stream, (fcntl.LOCK_SH if shared else fcntl.LOCK_EX) | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("Training/simulation already occupies the airground workload lock")
        if not shared:
            stream.seek(0)
            stream.truncate()
        stream.write(kind + "\n")
        stream.flush()
        try:
            yield stream
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)
