#!/usr/bin/python3
"""Given-initial-ROI SOT/gimbal stress audit against separate UE projection logs.

Projection logs are read ONLY by this evaluation process, never the detector,
tracker or servo. A single initial manual ROI is supplied through the normal
web API. Results are not detector mAP or pixel-mask ground truth.
"""
import argparse
import base64
import json
import math
from pathlib import Path
import sys
import time
from urllib.request import Request,urlopen,build_opener,ProxyHandler
from urllib.parse import urlparse
from urllib.error import HTTPError
import numpy as np
import rospy
import rospkg
from std_msgs.msg import String
from sar_yolo_detector.msg import SmartTrackerState
sys.path.insert(0,str(Path(rospkg.RosPack().get_path('sar_yolo_detector'))/'tools/ue'))
from gimbal_selection import current_visual_measurement


def iou(a,b):
    x=max(a[0],b[0]);y=max(a[1],b[1]);r=min(a[0]+a[2],b[0]+b[2]);d=min(a[1]+a[3],b[1]+b[3])
    intersection=max(0,r-x)*max(0,d-y)
    return intersection/max(1.,a[2]*a[3]+b[2]*b[3]-intersection)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--record-ready',type=Path)
    p.add_argument('--project',type=Path,required=True);p.add_argument('--capture',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--namespace',default='ue_test')
    p.add_argument('--roi-padding',type=int,choices=range(9),default=0,help='Initial manual ROI padding per side, pixels')
    p.add_argument('--web-url',default='http://127.0.0.1:8787');p.add_argument('--seconds',type=float,default=20)
    a=p.parse_args(rospy.myargv()[1:]);a.output.mkdir(parents=True,exist_ok=True)
    rospy.init_node('verify_ue_stress_case',anonymous=True);ns='/'+a.namespace.strip('/')
    states=[];traces=[];feedback=[];fusion_evidence=[]
    def state(m):
        b=m.selected_bbox
        states.append(dict(received_wall=time.time(),stamp=m.header.stamp.to_sec(),id=m.selected_track_id,selected=m.has_selection,
            class_id=m.selected_class_id,class_name=m.selected_class_name,tracker_type=m.tracker_type,
            box=[b.center.x-b.size_x/2,b.center.y-b.size_y/2,b.size_x,b.size_y],
            current=current_visual_measurement(dict(measurement_current=m.measurement_current,control_ready=m.control_measurement_ready,
                association_method=m.association_method,prediction_only=m.prediction_only,stale=m.data_is_stale,tentative=m.tentative)),
            confidence=m.confidence,reason=m.freshness_reason,inference_ms=m.inference_ms))
    subs=[rospy.Subscriber(ns+'/sar_yolo_detector/dfine_v3/state',SmartTrackerState,state,queue_size=100),
        rospy.Subscriber(ns+'/sar_yolo_smart_tracker_dfine_v3/pipeline_trace',String,lambda m:traces.append(json.loads(m.data)),queue_size=100),
        rospy.Subscriber(ns+'/gimbal/state',String,lambda m:feedback.append(json.loads(m.data)),queue_size=100)]
    subs.append(rospy.Subscriber(ns+'/sar_yolo_smart_tracker_dfine_v3/fusion_evidence',String,lambda m:fusion_evidence.append(json.loads(m.data)),queue_size=100))
    command=rospy.Publisher(ns+'/benchmark/command',String,queue_size=1,latch=True)
    local_open=build_opener(ProxyHandler({})).open if urlparse(a.web_url).hostname in ('127.0.0.1','localhost','::1') else urlopen
    def request(body=None):
        try:
            with local_open(Request(a.web_url+('/api/image' if body is None else '/api/command'),
                data=None if body is None else json.dumps(body).encode(),headers={'Content-Type':'application/json'}),timeout=5) as response:
                if body is not None:return json.load(response)
                value=json.loads(base64.b64decode(response.headers['X-SAR-Frame']))
                value['png']=base64.b64encode(response.read()).decode('ascii')
                return value
        except HTTPError as error:
            if error.code==409:return json.load(error)
            if error.code==503 and body is None:return dict(ready=False)
            raise
    deadline=time.monotonic()+240;run=None
    while time.monotonic()<deadline:
        try:
            run=json.loads((a.capture/'run.json').read_text());frame=request()
            if frame.get('ready') and frame.get('flight',{}).get('stage')=='inspect_01':break
        except (OSError,ValueError):pass
        time.sleep(.2)
    else:raise RuntimeError('Camera/perception/inspection hover did not become ready')
    spec=run['scene'];case=spec['benchmark'];primary=case['primary_entity'];offset=run['clock_alignment']['ue_minus_host_sec']
    truth_file=a.project/'Saved/AirGroundExchange/BenchmarkTruth'/(run['session']+'.jsonl')
    def truth(tail=False):
        if tail:
            with truth_file.open('rb') as stream:
                size=stream.seek(0,2);stream.seek(max(0,size-131072));data=stream.read().decode()
            lines=data.splitlines()
            if size>131072:lines=lines[1:]
        else:lines=truth_file.read_text().splitlines()
        return [json.loads(v) for v in lines if v.strip()]
    if case['case_id'].startswith('civil_'):
        # Flight telemetry can announce hover while the paired image still
        # depicts the end of ascent. A stationary-camera trial must select a
        # capture AFTER hover has settled, not the earlier displayed frame.
        settled_after=time.time()+2.
        deadline=time.monotonic()+15.
        while time.monotonic()<deadline:
            frame=request();h=frame.get('capture_header',{}).get('stamp',{})
            capture_stamp=h.get('secs',0)+h.get('nsecs',0)/1e9
            if capture_stamp>=settled_after and frame.get('flight',{}).get('stage')=='inspect_01':break
            time.sleep(.1)
        else:raise RuntimeError('No post-settle camera capture at the inspection hover')
    recorded_after=0
    if a.record_ready:
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            if a.record_ready.is_file():
                recorded_after=json.loads(a.record_ready.read_text())['after_capture_nsec']/1e9
                break
            time.sleep(.1)
        else:raise RuntimeError('Preselection recorder did not become ready')
    assert request(dict(action='mode',mode='fusion_tracker',color_name='roi')).get('success')
    selected=False;deadline=time.monotonic()+25;selection_debug=[]
    while time.monotonic()<deadline:
        frame=request();h=frame.get('capture_header',{}).get('stamp',{})
        stamp=h.get('secs',0)+h.get('nsecs',0)/1e9
        if stamp<=recorded_after:
            time.sleep(.1);continue
        try:rows=truth(tail=True)
        except (OSError,ValueError):time.sleep(.2);continue
        entry=min(rows,key=lambda v:abs(v['capture_wall_start']-offset-stamp))
        diagnostic=dict(frame_age_ms=frame.get('age_ms'),delta=entry['capture_wall_start']-offset-stamp,width=frame.get('width'),height=frame.get('height'))
        selection_debug.append(diagnostic)
        target=next((v for v in entry['objects'] if v['entity_id']==primary),None)
        diagnostic['target']=target
        if abs(entry['capture_wall_start']-offset-stamp)<.0002 and target and target['on_screen'] and target['projected_world_aabb']:
            x,y,w,h=target['projected_world_aabb'];pad=a.roi_padding
            box=[x-pad,y-pad,w+2*pad,h+2*pad];x,y,w,h=box
            if min(x,y)>1 and x+w<frame['width']-1 and y+h<frame['height']-1:
                reply=request(dict(action='roi',frame_token=frame['frame_token'],roi=box))
                diagnostic['reply']=reply
                if reply.get('success'):
                    selected=True;break
        time.sleep(.1)
    (a.output/'selection_attempts.json').write_text(json.dumps(selection_debug,indent=2)+'\n')
    result=dict(case_id=case['case_id'],evaluation='given_initial_ROI; projected world AABB and centre-ray proxy, not detection mAP',
        initialized=selected,thresholds=dict(correct_state_fraction=.9,centre_error_p95=.10,maximum_gap_sec=.5,id_switches=0),
        target_peak_speed_mps=case['target_peak_speed_mps'])
    if not selected:
        result.update(executed=False,reason='No fresh, fully visible initial ROI accepted')
        for name,data in [('states',states),('trace',traces),('fusion',fusion_evidence)]:
            (a.output/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
        (a.output/'result.json').write_text(json.dumps(result,indent=2)+'\n');raise RuntimeError(result['reason'])
    (a.output/'selection.json').write_text(json.dumps(dict(initialized=True,wall_stamp=time.time(),primary_entity=primary,initial_box=box,capture_header=frame.get('capture_header'))))
    (a.output/('initial.jpg' if frame.get('image_mime')=='image/jpeg' else 'initial.png')).write_bytes(base64.b64decode(frame['png'].split(',')[-1]))
    selected_id=reply.get('selected_id')
    lock_wait_deadline=time.monotonic()+12.
    while time.monotonic()<lock_wait_deadline and not rospy.is_shutdown():
        started=time.time()
        recent={v['stamp']:v for v in states
                if started-1.<=v['stamp']<=started and v['selected'] and v['id']==selected_id}
        static=sorted(recent.values(),key=lambda v:v['stamp'])
        ratio=sum(v['current'] for v in static)/max(1,len(static))
        if (len(static)>=3 and ratio>=.9 and all(v['current'] for v in static[-3:])
                and started-static[-1]['stamp']<=.5):
            # The evaluator alone uses the projection log. A stable but
            # misplaced box must not qualify the benchmark to start.
            try:
                recent_truth = truth(tail=True)
            except (OSError, ValueError):
                recent_truth = []
            by_stamp = {round((v['capture_wall_start']-offset)*1e6): v for v in recent_truth}
            def agrees(state):
                key = round(state['stamp']*1e6)
                row = next((by_stamp[k] for k in (key,key-1,key+1) if k in by_stamp), None)
                if row is None:
                    return False
                target = next((v for v in row['objects'] if v['entity_id']==primary), None)
                return bool(target and target['on_screen']
                            and target['visibility_proxy']=='centre_ray_clear'
                            and target['projected_world_aabb']
                            and iou(state['box'], target['projected_world_aabb'])>=.25)
            if all(agrees(v) for v in static[-3:]):
                break
        time.sleep(.05)
    else:
        result.update(executed=False,motion_started_with_lock=False,
                      reason='Initial lock did not stabilize; target motion withheld')
        for name,data in [('result',result),('states',states),('trace',traces),('fusion',fusion_evidence)]:
            (a.output/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
        request(dict(action='clear'))
        return
    result['pre_motion_current_ratio']=ratio
    result['motion_started_with_lock']=True
    command.publish(String(data='start'));deadline=time.monotonic()+a.seconds
    while time.monotonic()<deadline and not rospy.is_shutdown():time.sleep(.05)
    ended=time.time();command.publish(String(data='hold'))
    # Drain valid outputs for captures at the end of the scoring window before
    # clearing selection; the capture window itself is never extended.
    time.sleep(.6);frame=request()
    (a.output/('final.jpg' if frame.get('image_mime')=='image/jpeg' else 'final.png')).write_bytes(base64.b64decode(frame['png'].split(',')[-1]))
    request(dict(action='clear'));time.sleep(.2)
    rows=[v for v in truth() if started<=v['capture_wall_start']-offset<=ended]
    index={int(round((v['capture_wall_start']-offset)*1e6)):v for v in rows}
    current={}
    for v in states:
        if started<=v['stamp']<=ended and v['received_wall']<=ended+.5:
            key=int(round(v['stamp']*1e6))
            if key not in current or v['current']:current[key]=v
    comparisons=[]
    for key,value in current.items():
        gt=next((index[k] for k in [key,key-1,key+1] if k in index),None)
        if gt is None:continue
        target=next(v for v in gt['objects'] if v['entity_id']==primary)
        point=target['centre_px'];box=target['projected_world_aabb'];error=None;overlap=0.
        if point and box and value['current']:
            c=[value['box'][0]+value['box'][2]/2,value['box'][1]+value['box'][3]/2]
            error=math.hypot((c[0]-point[0])/(gt['width']/2),(c[1]-point[1])/(gt['height']/2))
            overlap=iou(value['box'],box)
        visible=target['on_screen'] and target['visibility_proxy']=='centre_ray_clear'
        comparisons.append(dict(stamp=value['stamp'],current=value['current'],visible_proxy=visible,
            blocked_proxy=target['visibility_proxy']=='centre_ray_blocked',error_norm=error,amodal_iou=overlap,
            correct=bool(visible and value['current'] and overlap>=.25 and error is not None and error<=.18),
            id=value['id'],box_size=box[2:] if box else None,reason=value['reason']))
    visible=[v for v in comparisons if v['visible_proxy']];correct=[v for v in visible if v['correct']]
    stamps=sorted(v['stamp'] for v in correct);gaps=np.diff([started]+stamps+[ended])
    recoveries=[]
    loss_start=None
    established=False
    for value in sorted(comparisons,key=lambda v:v['stamp']):
        if value['correct']:
            if loss_start is not None:
                recoveries.append(dict(lost_at=loss_start,
                                       recovered_after_sec=value['stamp']-loss_start))
                loss_start=None
            established=True
        elif established and loss_start is None:
            loss_start=value['stamp']
    if loss_start is not None:
        recoveries.append(dict(lost_at=loss_start,recovered_after_sec=None))
    errors=[v['error_norm'] for v in correct];sizes=[v['box_size'] for v in visible if v['box_size']]
    ids=[v['id'] for v in correct];switches=sum(x!=y for x,y in zip(ids,ids[1:]))
    trace=[v for v in traces if started<=v['stamp']<=ended]
    def calls(v):return sum(t.get('backend_call',0) for t in v.get('stage_counts_by_mode',{}).values())
    rate=(calls(trace[-1])-calls(trace[0]))/(trace[-1]['stamp']-trace[0]['stamp']) if len(trace)>1 else None
    result.update(executed=True,seconds=ended-started,truth_frames=len(rows),unique_state_captures=len(current),
        matched_state_captures=len(comparisons),visible_proxy_states=len(visible),correct_current_states=len(correct),
        correct_state_fraction=len(correct)/max(1,len(visible)),
        false_lock_states=sum(v['current'] and not v['correct'] for v in comparisons),
        false_lock_fraction_of_current=sum(v['current'] and not v['correct'] for v in comparisons)/max(1,sum(v['current'] for v in comparisons)),
        recovery_events=recoveries,
        correct_observation_gaps_sec=[float(t) for t in np.diff(stamps) if t>.5],
        state_update_fps_including_lost=len(current)/(ended-started),
        detection_pipeline_fps=rate,camera_truth_fps=len(rows)/(ended-started),
        centre_error_p95_norm=float(np.percentile(errors,95)) if errors else None,
        maximum_correct_measurement_gap_sec=float(max(gaps)),id_switches=switches,
        projected_aabb_median_px=np.median(sizes,axis=0).tolist() if sizes else None,
        trusted_states_during_centre_ray_blocked=sum(v['current'] and v['blocked_proxy'] for v in comparisons),
        visibility_unknown_frames=sum(any(v['entity_id']==primary and v['visibility_proxy']=='unknown' for v in row['objects']) for row in rows),
        classification_labels=sorted(set(v['class_name'] for v in states if v['selected'])),
        local_tracker_types=sorted(set(v['tracker_type'] for v in states if v['selected'])),
        identity_numbers=sorted(set(ids)))
    result['evaluation_valid']=result['visibility_unknown_frames']==0 and len(comparisons)>0
    result['tracking_acceptance']=None if not result['evaluation_valid'] else bool(result['correct_state_fraction']>=.9 and result['centre_error_p95_norm'] is not None
        and result['centre_error_p95_norm']<=.10 and result['maximum_correct_measurement_gap_sec']<=.5 and switches==0)
    # Image centering measures the true target against the optical center.
    # The tracker-box error above measures the box against the target.
    centre_offsets=[]
    visible_intervals=[]
    occlusions=[]
    visible_start=last_visible=blocked_start=None
    for row in sorted(rows,key=lambda value:value['capture_wall_start']):
        target=next(v for v in row['objects'] if v['entity_id']==primary)
        stamp=row['capture_wall_start']-offset
        is_visible=target['on_screen'] and target['visibility_proxy']=='centre_ray_clear'
        if is_visible:
            if visible_start is None:
                visible_start=stamp
            last_visible=stamp
            u,v=target['centre_px']
            centre_offsets.append(math.hypot((u-row['width']/2)/(row['width']/2),
                                             (v-row['height']/2)/(row['height']/2)))
            if blocked_start is not None:
                first_recovery=next((sample['stamp'] for sample in sorted(correct,key=lambda v:v['stamp'])
                                     if sample['id']==selected_id and sample['stamp']>=stamp),None)
                occlusions.append(dict(blocked_from=blocked_start,visible_again=stamp,
                                       same_id_recovery_sec=first_recovery-stamp if first_recovery is not None else None))
                blocked_start=None
        else:
            if visible_start is not None:
                visible_intervals.append((visible_start,last_visible))
                visible_start=None
            if last_visible is not None and target['on_screen'] and target['visibility_proxy']=='centre_ray_blocked':
                if blocked_start is None:
                    blocked_start=stamp
    if visible_start is not None:
        visible_intervals.append((visible_start,last_visible))
    visible_gaps=[gap for left,right in visible_intervals
                  for gap in np.diff([left]+[stamp for stamp in stamps if left<=stamp<=right]+[right])]
    result.update(scoring_window_wall=[started,ended],initial_selected_id=selected_id,
                  correct_capture_stamps=stamps,
                  true_target_centre_offset_p95_norm=float(np.percentile(centre_offsets,95)) if centre_offsets else None,
                  correct_box_iou_p50=float(np.median([v['amodal_iou'] for v in correct])) if correct else None,
                  visible_capture_fraction=len(centre_offsets)/max(1,len(rows)),
                  visible_maximum_gap_sec=float(max(visible_gaps)) if visible_gaps else None,
                  occlusion_recovery=occlusions,
                  hidden_current_states=sum(v['current'] and not v['visible_proxy'] for v in comparisons))
    if case['case_id'].startswith('civil_static_air_') or case.get('static_lock_case',False):
        # Missing-target time must NOT count as a visible tracking gap. Use
        # the independent capture timeline, not only states that arrived.
        intervals=[];begin=None;last_visible=None;first_absent=None
        for row in rows:
            target=next(v for v in row['objects'] if v['entity_id']==primary)
            t=row['capture_wall_start']-offset
            is_visible=target['on_screen'] and target['visibility_proxy']=='centre_ray_clear'
            if is_visible:
                if begin is None:begin=t
                last_visible=t
            else:
                if begin is not None:intervals.append((begin,last_visible));begin=None
                if first_absent is None:first_absent=t
        if begin is not None:intervals.append((begin,last_visible))
        visible_gaps=[]
        for left,right in intervals:
            hits=[t for t in stamps if left<=t<=right]
            visible_gaps.extend(np.diff([left]+hits+[right]).tolist())
        hidden_current=sum(v['current'] and not v['visible_proxy'] for v in comparisons)
        late_controls=[] if first_absent is None else [v['received_wall']-first_absent for v in states
            if v['current'] and first_absent-.5<=v['stamp']<=ended and v['received_wall']>=first_absent]
        visible_duration=sum(right-left for left,right in intervals)
        maximum_visible_gap=max(visible_gaps,default=float('inf'))
        result['static_lock_metrics']=dict(visible_duration_sec=visible_duration,
            maximum_visible_measurement_gap_sec=maximum_visible_gap,
            current_states_when_target_absent=hidden_current,
            last_current_control_after_disappearance_sec=max(late_controls,default=0.),
            disappearance_exercised=first_absent is not None,
            initialization_confirmed_before_test=result['motion_started_with_lock'])
        result['static_lock_acceptance']=bool(result['evaluation_valid'] and result['motion_started_with_lock']
            and visible_duration>=10 and len(correct)>0 and result['correct_state_fraction']>=.9
            and result['centre_error_p95_norm']<=.10 and maximum_visible_gap<=.5 and switches==0
            and hidden_current==0 and max(late_controls,default=0.)<=.5)
    for name,data in [('result',result),('states',states),('comparison',comparisons),('trace',traces),('feedback',feedback),('fusion',fusion_evidence)]:
        (a.output/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
