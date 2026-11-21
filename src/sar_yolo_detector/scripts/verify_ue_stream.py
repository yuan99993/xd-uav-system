#!/usr/bin/env python3
"""Measure real camera/control/web cadence without publishing substitute images."""
import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import time
from urllib.request import urlopen
import numpy as np
import rospy
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sar_yolo_detector.msg import SmartTrackerState
sys.path.insert(0,str(Path(rospkg.RosPack().get_path('sar_yolo_detector'))/'tools/ue'))
from gimbal_selection import current_visual_measurement


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--namespace',default='ue_test');p.add_argument('--instance',default='dfine_v3')
    p.add_argument('--web-url',default='http://127.0.0.1:8787');p.add_argument('--seconds',type=float,default=20.)
    p.add_argument('--cpp-probe',action='store_true',help='Use the C++ image subscriber to avoid Python/GIL observer drops')
    p.add_argument('--require-control',action='store_true');p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(rospy.myargv()[1:]);a.output.parent.mkdir(parents=True,exist_ok=True)
    probe_path=a.output.with_suffix('.cpp.json');probe=None
    if a.cpp_probe:
        probe=subprocess.Popen(['rosrun','sar_yolo_detector','sar_camera_stream_probe',
            '_namespace:='+a.namespace,'_instance:='+a.instance,'_seconds:='+str(a.seconds),
            '_output:='+str(probe_path.resolve())])
    rospy.init_node('verify_ue_stream',anonymous=True)
    counts=Counter();ages=[];tokens=set();prefix='/'+a.namespace.strip('/')
    def image(m):counts['images']+=1
    def feedback(m):counts['feedback']+=1
    def state(m):
        counts['states']+=1
        if m.has_selection and current_visual_measurement(dict(measurement_current=m.measurement_current,
            control_ready=m.control_measurement_ready,association_method=m.association_method,
            prediction_only=m.prediction_only,stale=m.data_is_stale,tentative=m.tentative)):
            counts['current_visual']+=1;ages.append((rospy.Time.now()-m.header.stamp).to_sec()*1000.)
    subs=[rospy.Subscriber(prefix+'/gimbal/state',String,feedback,queue_size=1),
        rospy.Subscriber(prefix+'/sar_yolo_detector/'+a.instance+'/state',SmartTrackerState,state,queue_size=1)]
    if not a.cpp_probe:subs.append(rospy.Subscriber(prefix+'/fixed_camera/image_raw',Image,image,queue_size=1,buff_size=16*1024*1024,tcp_nodelay=True))
    start=time.monotonic()
    while time.monotonic()-start<a.seconds and not rospy.is_shutdown():
        try:
            with urlopen(a.web_url+'/api/telemetry',timeout=2) as r:f=json.load(r)
            if f.get('ready'):tokens.add(f['frame_token'])
        except (OSError,ValueError):counts['web_errors']+=1
        time.sleep(.05)
    duration=time.monotonic()-start
    result=dict(passed=False,duration_sec=duration,counts=dict(counts),
        image_fps=counts['images']/duration,state_fps=counts['states']/duration,
        current_visual_fps=counts['current_visual']/duration,feedback_fps=counts['feedback']/duration,
        web_fps=len(tokens)/duration,capture_to_visual_p95_ms=float(np.percentile(ages,95)) if ages else None)
    if probe is not None:
        if probe.wait(timeout=5)!=0:raise RuntimeError('C++ camera probe failed')
        measured=json.loads(probe_path.read_text());measured.pop('duration_sec',None)
        result.update(measured);result['measurement_backend']='roscpp'
    result['passed']=result['image_fps']>=20 and result['feedback_fps']>=10 and counts['web_errors']==0
    if a.require_control:
        maximum=result.get('max_capture_to_visual_ms',max(ages) if ages else None)
        result['passed'] &= result['current_visual_fps']>=10 and maximum is not None and maximum<1200
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    for sub in subs:sub.unregister()
    print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)

if __name__=='__main__':main()
