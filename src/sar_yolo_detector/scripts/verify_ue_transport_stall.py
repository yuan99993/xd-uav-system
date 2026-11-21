#!/usr/bin/python3
"""Exercise the real UE native sender across a deliberately slow TCP reader.

Only the test client's receive thread is paused. Capture-age rejection and
latest-frame replacement remain enabled; no perception/control flags change.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import rospkg


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--project',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--pause-sec',type=float,default=.75)
    a=p.parse_args()
    if not .55<=a.pause_sec<=.9:p.error('pause-sec must be between .55 and .9 seconds')
    package=Path(rospkg.RosPack().get_path('sar_yolo_detector'))
    sys.path.insert(0,str(package/'tools/ue'))
    import ue_native_bridge
    from stream_protocol import SocketWire
    original=SocketWire.receive;marks=dict(frame_count=0,pause_executed=False)
    def receive(wire):
        if marks['frame_count']>=30 and not marks['pause_executed']:
            marks['pause_executed']=True;marks['pause_start_wall']=time.time()
            time.sleep(a.pause_sec);marks['pause_end_wall']=time.time()
        meta,payload=original(wire)
        if meta.get('kind')=='frame':marks['frame_count']+=1
        return meta,payload
    SocketWire.receive=receive
    sys.argv=['ue_native_bridge','--project',str(a.project),'--scene',str(package/'config/ue/benchmark_cases/civil_static_air_h60.json'),
        '--out',str(a.output),'--seconds','12','--width','1920','--height','1080','--fps','30',
        '--camera','down','--stream-codec','jpeg','--stream-only','--transport','tcp','--stream-connection','direct','--pose-source','kinematic']
    try:ue_native_bridge.main()
    finally:SocketWire.receive=original
    summary=json.loads((a.output/'capture_summary.json').read_text())
    frames=json.loads((a.output/'frame_timing.json').read_text());end=marks.get('pause_end_wall',float('inf'))
    recovered=[v for v in frames if end<=v['publish_wall']<=end+2 and v['delivery_age_ms']<300]
    result=dict(passed=marks['pause_executed'] and not summary['errors'] and len(recovered)>0 and summary['delivered_frames']>=60,
        injection=marks,recovered_fresh_frames_within_two_seconds=len(recovered),delivered_frames=summary['delivered_frames'],
        stale_frames_rejected=summary['dropped_stale'],latest_frames_replaced=summary['dropped_latest'],errors=summary['errors'])
    (a.output/'stall_acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not result['passed']:raise RuntimeError('Native slow-reader acceptance failed')

if __name__=='__main__':main()
