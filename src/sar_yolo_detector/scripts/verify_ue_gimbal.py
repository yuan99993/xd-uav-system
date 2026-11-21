#!/usr/bin/env python3
"""Verify a running UE camera/ROS/web gimbal chain in a dedicated test namespace.

Selects a real detector box via the web, records visual and measured actuator
feedback, then clears selection. Does not read UE target truth or control flight.
"""
import argparse
import base64
import json
import math
from pathlib import Path
import time
from urllib.request import Request,urlopen
from urllib.error import HTTPError
import sys
import rospkg
sys.path.insert(0,str(Path(rospkg.RosPack().get_path("sar_yolo_detector"))/"tools/ue"))
from gimbal_selection import current_visual_measurement
import rospy
from std_msgs.msg import String
from sar_yolo_detector.msg import SmartTrackerState


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--namespace',default='ue_test')
    p.add_argument('--instance',default='dfine_v3')
    p.add_argument('--web-url',default='http://127.0.0.1:8787')
    p.add_argument('--require-mission-stage',default='',help='Wait for this actual mission stage before selecting')
    p.add_argument('--manual-roi',nargs=4,type=float,metavar=('X','Y','W','H'),help='Manually chosen actual image ROI; does not assign a class or read simulator truth')
    p.add_argument('--selection-region',nargs=4,type=float,metavar=('X','Y','W','H'),help='Only select requested-class detection centres inside this image region')
    p.add_argument('--class-name',default='',help='Require this detected class; never relabel a selected object')
    p.add_argument('--seconds',type=float,default=20.)
    p.add_argument('--startup-timeout',type=float,default=180.)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(rospy.myargv()[1:]);a.output.mkdir(parents=True,exist_ok=True)
    rospy.init_node('verify_ue_gimbal',anonymous=True)
    samples=[];states=[];geometry=[];fusion=[];prefix='/'+a.namespace.strip('/')
    subscriptions=[rospy.Subscriber(prefix+'/sar_yolo_smart_tracker_'+a.instance+'/fusion_evidence',String,
        lambda m:fusion.append(json.loads(m.data)),queue_size=100),rospy.Subscriber(prefix+'/target_geometry',String,
        lambda m:geometry.append(json.loads(m.data)),queue_size=100),rospy.Subscriber(prefix+'/gimbal/state',String,
        lambda m:samples.append(json.loads(m.data)),queue_size=100),
        rospy.Subscriber(prefix+'/sar_yolo_detector/'+a.instance+'/state',SmartTrackerState,
        lambda m:states.append(dict(stamp=m.header.stamp.to_sec(),current=current_visual_measurement(dict(measurement_current=m.measurement_current,control_ready=m.control_measurement_ready,
            association_method=m.association_method,prediction_only=m.prediction_only,stale=m.data_is_stale,tentative=m.tentative)),
            reason=m.freshness_reason,association=m.association_method,confidence=m.confidence,
            ready=m.control_measurement_ready,selected=m.has_selection,
            center=[m.selected_bbox.center.x,m.selected_bbox.center.y])),queue_size=50)]
    def request(body=None):
        path='/api/frame' if body is None else '/api/command'
        try:
            with urlopen(Request(a.web_url+path,data=None if body is None else json.dumps(body).encode(),
                                 headers={'Content-Type':'application/json'}),timeout=5.) as response:
                return json.load(response)
        except HTTPError as error:
            if body is None or error.code!=409:raise
            # A rejected stale frame must be refreshed and selected again;
            # never relax the server's original-capture age/identity gates.
            response=json.load(error)
            result.setdefault('rejected_selection_requests',[]).append(response)
            return response
    result=dict(passed=False,kind='UE visual gimbal smoke; no flight-control or target-speed acceptance')
    selection_started=False
    try:
        deadline=time.monotonic()+a.startup_timeout
        while time.monotonic()<deadline and not rospy.is_shutdown():
            try:
                frame=request()
                if frame.get('ready') and (frame.get('objects') or a.manual_roi) and (not a.require_mission_stage or frame.get('flight',{}).get('stage')==a.require_mission_stage) and samples and samples[-1].get('feedback_fresh'):break
            except (OSError,ValueError):pass
            time.sleep(.2)
        else:raise RuntimeError('Fresh UE camera/gimbal/web feedback not ready')
        assert request(dict(action='mode',mode='fusion_tracker',color_name='roi')).get('success')
        deadline=time.monotonic()+20
        while time.monotonic()<deadline and not rospy.is_shutdown():
            frame=request()
            if a.manual_roi and frame.get('mode')=='fusion_tracker':
                result['selection_rule']='Explicit manual ROI on actual displayed RGB capture; classification remains unknown unless detected'
                result['selected_initial_box']=a.manual_roi
                if request(dict(action='roi',frame_token=frame['frame_token'],roi=a.manual_roi)).get('success'):break
            boxes=[v for v in frame.get('objects',[]) if v.get('kind')=='detector_track' and (not a.class_name or v.get('name')==a.class_name)]
            if a.selection_region:
                rx,ry,rw,rh=a.selection_region
                boxes=[v for v in boxes if rx<=v['center'][0]<=rx+rw and ry<=v['center'][1]<=ry+rh]
            if not a.manual_roi and frame.get('mode')=='fusion_tracker' and boxes:
                visible=[v for v in boxes if v['box'][0]>1 and v['box'][1]>1 and v['box'][0]+v['box'][2]<frame['width']-1 and v['box'][1]+v['box'][3]<frame['height']-1]
                if not visible:time.sleep(.2);continue
                target=min(visible,key=lambda v:((v['center'][0]-frame['width']/2)/(frame['width']/2))**2+((v['center'][1]-frame['height']/2)/(frame['height']/2))**2)
                result['selection_rule']='Closest fully visible requested-class detection to image centre'
                result['selected_initial_box']=target['box']
                result['selected_initial_id']=target['id']
                x,y,w,h=target['box']
                if request(dict(action='click',frame_token=frame['frame_token'],point=[x+w/2,y+h/2])).get('success'):break
            time.sleep(.2)
        else:raise RuntimeError('Could not select actual detector target')
        selection_started=True;started=time.time();valid=0;updates=0;seen=set()
        width,height=frame['width'],frame['height']
        (a.output/'before_selection.png').write_bytes(base64.b64decode(frame['png'].split(',')[-1]))
        deadline=time.monotonic()+a.seconds
        while time.monotonic()<deadline and not rospy.is_shutdown():
            frame=request();token=frame.get('frame_token')
            if token and token not in seen:
                seen.add(token);updates+=1;valid+=int(bool(frame.get('control_target_valid')))
            time.sleep(.1)
        (a.output/'after_recenter.png').write_bytes(base64.b64decode(frame['png'].split(',')[-1]))
        cleared=time.time();assert request(dict(action='clear')).get('success');time.sleep(2)
        selected=[v for v in states if started<=v['stamp']<cleared and v['current'] and v['ready'] and v['selected']]
        error=[math.hypot((v['center'][0]-width/2)/(width/2),(v['center'][1]-height/2)/(height/2)) for v in selected]
        active=[v for v in samples if started<=v['wall_stamp']<cleared]
        assert len(error)>=15 and active,'Insufficient real visual/actuator measurements'
        result.update(valid_visual_measurements=len(error),initial_error_norm=sum(error[:3])/len(error[:3]),
            final_error_norm=sum(error[-5:])/len(error[-5:]),valid_web_ratio=valid/max(1,updates),
            yaw_excursion_deg=max(v['yaw_deg'] for v in active)-min(v['yaw_deg'] for v in active),
            last_feedback=samples[-1])
        assert result['final_error_norm']<.18,'Target did not recenter'
        assert result['valid_web_ratio']>=.75,'Sustained web lock below 75%'
        assert samples[-1]['mode']=='HOLD' and abs(samples[-1]['yaw_rate_deg_s'])<.1 and abs(samples[-1]['pitch_rate_deg_s'])<.1,'Clear did not brake actuator'
        for value in active:
            limit=value['limits']['max_rate_deg_s']
            assert max(abs(value['yaw_rate_deg_s']),abs(value['pitch_rate_deg_s']))<=limit+1e-5,'Actuator rate limit violated'
            actual=value.get('measured_rotation_deg',{})
            assert abs(actual.get('pitch',value['pitch_deg'])-value['pitch_deg'])<.01,'UE did not apply actuator pitch'
        if result['initial_error_norm']>.2:assert result['yaw_excursion_deg']>3 or abs(active[-1]['pitch_deg']-active[0]['pitch_deg'])>3,'Camera did not actually turn'
        observed_geometry=[v for v in geometry if v.get('measurement_valid',True) and started<=v['measurement_stamp']<cleared]
        assert len(observed_geometry)>=15,'Missing actual capture-time geometry evidence'
        assert all(not v['range_valid'] and not v['target_position_valid'] and not v['ground_projection_applied'] for v in observed_geometry),'Image-stage target acquired unsupported depth'
        if a.class_name:
            from target_geometry import target_domain
            assert all(v['target_domain']==target_domain(a.class_name) for v in observed_geometry),'Selected target domain changed'
            assert len(set(v['track_id'] for v in observed_geometry))==1,'Selected public identity changed'
            from collections import Counter
            result['classification_observations']=dict(Counter(v['class_name'] for v in observed_geometry))
        result['geometry_samples']=len(observed_geometry)
        result['class_name']=a.class_name
        result['geometry_domains']=sorted(set(v['target_domain'] for v in observed_geometry))
        (a.output/'geometry.json').write_text(json.dumps(observed_geometry,indent=2))
        result['passed']=True
    except Exception as e:
        result['error']=str(e);raise
    finally:
        if selection_started:
            try:request(dict(action='clear'))
            except (OSError,ValueError):pass
        (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        (a.output/'feedback.json').write_text(json.dumps(samples,indent=2)+'\n')
        (a.output/'fusion.json').write_text(json.dumps(fusion,indent=2)+'\n')
        (a.output/'geometry.json').write_text(json.dumps(geometry,indent=2)+'\n')
        (a.output/'visual.json').write_text(json.dumps(states,indent=2)+'\n')
        for subscription in subscriptions:subscription.unregister()
        print(json.dumps(result,indent=2))

if __name__=='__main__':main()
