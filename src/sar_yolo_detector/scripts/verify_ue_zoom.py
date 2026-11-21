#!/usr/bin/env python3
"""Real UE closed-loop 90->60->90 lens acceptance, with capture-stamped K.

Requires allow_fov_control:=true. Never publishes images or target truth.
"""
import argparse
import base64
from collections import Counter
import json
import math
from pathlib import Path
import sys
import threading
import time
from urllib.request import Request,urlopen
import rospy
import rospkg
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64,String
from sar_yolo_detector.msg import SmartTrackerState
sys.path.insert(0,str(Path(rospkg.RosPack().get_path('sar_yolo_detector'))/'tools/ue'))
from gimbal_selection import current_visual_measurement


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--namespace',default='ue_test');p.add_argument('--instance',default='dfine_v3')
    p.add_argument('--web-url',default='http://127.0.0.1:8787');p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(rospy.myargv()[1:]);a.output.mkdir(parents=True,exist_ok=True)
    rospy.init_node('verify_ue_zoom',anonymous=True)
    prefix='/'+a.namespace.strip('/');infos={};states=[];traces=[];evidence=[];phases=[]
    data_lock=threading.RLock()
    result=dict(passed=False,scope='Real UE virtual lens and image-space tracking; no optical hardware or flight-speed claim')
    def info(m):
        fx=float(m.K[0]);fov=math.degrees(2*math.atan(m.width/(2*fx))) if fx>0 else float('nan')
        value=dict(stamp=m.header.stamp.to_sec(),frame_id=m.header.frame_id,
            width=m.width,height=m.height,fx=fx,fy=m.K[4],cx=m.K[2],cy=m.K[5],fov=fov)
        with data_lock:infos[m.header.stamp.to_nsec()]=value
    def state(m):
        valid=current_visual_measurement(dict(measurement_current=m.measurement_current,
            control_ready=m.control_measurement_ready,association_method=m.association_method,
            prediction_only=m.prediction_only,stale=m.data_is_stale,tentative=m.tentative)) and m.has_selection
        value=dict(stamp=m.header.stamp.to_sec(),stamp_ns=m.header.stamp.to_nsec(),
            wall=time.time(),valid=valid,id=m.selected_track_id,frame_id=m.header.frame_id,
            box=[m.selected_bbox.center.x,m.selected_bbox.center.y,m.selected_bbox.size_x,m.selected_bbox.size_y],
            source=m.association_method)
        with data_lock:states.append(value)
    subs=[rospy.Subscriber(prefix+'/fixed_camera/camera_info',CameraInfo,info,queue_size=100),
        rospy.Subscriber(prefix+'/sar_yolo_detector/'+a.instance+'/state',SmartTrackerState,state,queue_size=100),
        rospy.Subscriber(prefix+'/sar_yolo_smart_tracker_'+a.instance+'/pipeline_trace',String,
                         lambda m:traces.append(json.loads(m.data)),queue_size=10),
        rospy.Subscriber(prefix+'/sar_yolo_smart_tracker_'+a.instance+'/fusion_evidence',String,
                         lambda m:evidence.append(json.loads(m.data)),queue_size=50)]
    lens=rospy.Publisher(prefix+'/camera/fov_target_deg',Float64,queue_size=1)
    def request(body=None,image=False):
        route='/api/command' if body is not None else '/api/frame' if image else '/api/telemetry'
        with urlopen(Request(a.web_url+route,data=None if body is None else json.dumps(body).encode(),
                     headers={'Content-Type':'application/json'}),timeout=5) as r:return json.load(r)
    def picture(name):
        f=request(image=True)
        (a.output/(name+'.png')).write_bytes(base64.b64decode(f['png'].split(',')[-1]))
        f.pop('png',None);(a.output/(name+'.json')).write_text(json.dumps(f,indent=2)+'\n')
    def observe(seconds):
        seen=set();valid=0;deadline=time.monotonic()+seconds
        while time.monotonic()<deadline and not rospy.is_shutdown():
            f=request();token=f.get('frame_token')
            if token and token not in seen:
                seen.add(token);valid+=int(bool(f.get('control_target_valid')))
            time.sleep(.05)
        return dict(updates=len(seen),valid=valid,ratio=valid/max(1,len(seen)))
    selected=False;window_start=None
    try:
        deadline=time.monotonic()+180
        while time.monotonic()<deadline and not rospy.is_shutdown():
            try:
                f=request()
                if f.get('ready') and f.get('objects') and infos and lens.get_num_connections():break
            except (OSError,ValueError):pass
            time.sleep(.1)
        else:raise RuntimeError('UE camera/model/FOV command subscriber not ready')
        lens.publish(Float64(data=30.))  # Outside the validated lens range: must be rejected.
        time.sleep(.3)
        assert abs(list(infos.values())[-1]['fov']-90.)<.1,'Invalid FOV command moved the lens'
        assert request(dict(action='mode',mode='fusion_tracker',color_name='roi')).get('success')
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            f=request();boxes=[v for v in f.get('objects',[]) if v.get('kind')=='detector_track'
                and v['box'][0]>4 and v['box'][1]>4 and v['box'][0]+v['box'][2]<f['width']-4
                and v['box'][1]+v['box'][3]<f['height']-4]
            if f.get('mode')=='fusion_tracker' and boxes:
                target=max(boxes,key=lambda b:b.get('confidence',0)/(1+
                    ((b['box'][0]+b['box'][2]/2-f['width']/2)/(f['width']/2))**2+
                    ((b['box'][1]+b['box'][3]/2-f['height']/2)/(f['height']/2))**2))
                box=target['box']
                result['selected_initial_box']=box
                result['selection_rule']='Fully visible detector ROI; confidence divided by one plus squared normalized centre distance'
                try:selected=bool(request(dict(action='click',frame_token=f['frame_token'],point=[box[0]+box[2]/2,box[1]+box[3]/2])).get('success'))
                except OSError:pass
                if selected:break
            time.sleep(.1)
        assert selected,'No real detector target selected'
        deadline=time.monotonic()+15
        while time.monotonic()<deadline:
            f=request();box=f.get('selected_box')
            if box and f.get('control_target_valid') and math.hypot((box[0]-f['width']/2)/(f['width']/2),(box[1]-f['height']/2)/(f['height']/2))<.12:break
            time.sleep(.1)
        else:raise RuntimeError('Target did not recenter before lens test')
        window_start=time.time();phases.append(dict(target_fov=90.,start=window_start,**observe(2)))
        picture('before_90')
        for target,name in [(60.,'zoomed_60'),(90.,'returned_90')]:
            start=time.time();lens.publish(Float64(data=target));phase=observe(8.)
            phases.append(dict(target_fov=target,start=start,**phase));picture(name)
        window_end=time.time()
        usable=[v for v in states if window_start<=v['wall']<=window_end and v['valid']]
        assert len(usable)>=100,'Insufficient actual current visual observations'
        ids={v['id'] for v in usable};assert len(ids)==1,'Selected public ID changed during lens motion'
        assert all(v['ratio']>=.75 for v in phases),'Sustained lock below 75% during lens phase'
        calibrated=[];errors=[]
        for v in usable:
            c=infos.get(v['stamp_ns'])
            if c is None:continue
            assert c['frame_id']==v['frame_id'],'Calibration frame mismatch'
            assert abs(c['fy']-c['fx'])<1e-5 and abs(c['cx']-c['width']/2)<1e-5 and abs(c['cy']-c['height']/2)<1e-5,'Invalid camera K'
            calibrated.append(c)
            errors.append(math.hypot((v['box'][0]-c['width']/2)/(c['width']/2),(v['box'][1]-c['height']/2)/(c['height']/2)))
        assert len(calibrated)>=.95*len(usable),'Current measurements lack their capture calibration'
        angles=[v['fov'] for v in calibrated]
        assert min(angles)<60.1 and max(angles)>89.9,'Measured K did not traverse both FOV endpoints'
        assert calibrated[-1]['fov']>89.9,'Lens did not return to 90 degrees'
        errors.sort();p95=errors[int((len(errors)-1)*.95)]
        assert p95<.18,'Target did not remain centred during zoom'
        counts=Counter()
        for row in traces:
            for name,value in row.get('stage_counts_by_mode',{}).get('fusion_tracker',{}).items():counts[name]=max(counts[name],value)
        for name in ['camera_calibration_update','fusion_lens_template_refresh','fusion_color_lens_update']:
            assert counts[name]>=10,'Required lens adaptation stage missing: '+name
        result.update(passed=True,phases=phases,selected_id=next(iter(ids)),current_visual_measurements=len(usable),
            calibrated_measurements=len(calibrated),min_fov_deg=min(angles),max_fov_deg=max(angles),
            pixel_center_p95_norm=p95,algorithm_stage_counts=dict(counts),window_start=window_start,window_end=window_end)
    except Exception as error:
        result.update(error=str(error),phases=phases,window_start=window_start);raise
    finally:
        lens.publish(Float64(data=90.))
        if selected:
            try:request(dict(action='clear'))
            except (OSError,ValueError):pass
        with data_lock:
            saved=[('result',dict(result)),('calibration',dict(infos)),('states',list(states)),('trace',list(traces)),('fusion',list(evidence))]
        for name,data in saved:
            (a.output/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
        for sub in subs:sub.unregister()
        print(json.dumps(result,indent=2),flush=True)

if __name__=='__main__':main()
