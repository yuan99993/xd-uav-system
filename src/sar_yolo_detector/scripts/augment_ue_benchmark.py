#!/usr/bin/python3
"""Install and request the additive, owned benchmark-map copy in a running UE editor."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid
import rospkg


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--project',required=True,type=Path)
    p.add_argument('--timeout',type=float,default=180);a=p.parse_args()
    package=Path(rospkg.RosPack().get_path('sar_yolo_detector'))
    subprocess.check_call([sys.executable,str(Path(__file__).resolve().with_name('install_ue_capture.py')),'--project',str(a.project)])
    root=a.project/'Saved/AirGroundExchange'
    for op in ['reload','augment_benchmark']:
        req=dict(op=op,id=int(time.time()*1000),session=uuid.uuid4().hex)
        tmp=root/'request.benchmark.tmp';tmp.write_text(json.dumps(req));tmp.replace(root/'request.json')
        deadline=time.monotonic()+a.timeout
        while time.monotonic()<deadline:
            time.sleep(.1)
            try:result=json.loads((root/'response.json').read_text())
            except (OSError,ValueError):continue
            if result.get('id')==req['id']:
                if not result.get('ok'):raise RuntimeError(result.get('error','UE benchmark construction failed'))
                break
        else:raise RuntimeError('UE benchmark operation timed out: '+op)
    result=json.loads((a.project/'Scripts/AirGround/benchmark_build_result.json').read_text())
    if result['base_sha256_before']!=result['base_sha256_after']:raise RuntimeError('Original map changed')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
