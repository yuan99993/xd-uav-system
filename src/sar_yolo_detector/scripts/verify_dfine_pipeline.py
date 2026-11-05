#!/usr/bin/env python3
"""Exercise the deployed four-mode chain using a bundled UE image via ROS/Web.

Run against a dedicated test namespace with no other camera publisher. This is
an algorithm/data-flow smoke test, not a flight or accuracy benchmark.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import threading
import time
from urllib.request import Request, urlopen

import cv2
import numpy as np
import rospkg
import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros
from std_msgs.msg import String
from xd_uav_track.msg import DetectionArray, TrackStateArray
from sar_yolo_detector.pipeline_metrics import XDChainMetrics


REQUIRED = {
    'smart_tracker': ['backend_call', 'smart_manager_update'],
    'feature_tracker': ['camera_motion_update', 'manual_feature_update'],
    'color_tracker': ['camera_motion_update', 'manual_colour_update'],
    'fusion_tracker': ['backend_call', 'camera_motion_update', 'visual_flow_update',
        'fusion_smart_manager_seed', 'fusion_smart_manager_update',
        'fusion_roi_color_seed', 'fusion_roi_color_update', 'fusion_feature_update',
        'fusion_candidate_filter', 'fusion_arbiter_step', 'fusion_output_deduplication'],
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='uav1')
    parser.add_argument('--instance', default='dfine_v3')
    parser.add_argument('--web-url', default='http://127.0.0.1:8787')
    parser.add_argument('--startup-timeout', type=float, default=180.)
    parser.add_argument('--per-mode-sec', type=float, default=8.)
    parser.add_argument('--output', type=Path, default=Path('dfine_pipeline_verification.json'))
    args = parser.parse_args()
    package = Path(rospkg.RosPack().get_path('sar_yolo_detector'))
    image = cv2.imread(str(package/'test/fixtures/dfine_v3_ue.jpg'))
    assert image is not None, 'Bundled smoke image missing'
    rospy.init_node('verify_dfine_pipeline', anonymous=True)
    prefix = '/'+args.namespace.strip('/')
    traces, events, locks, bridge = [], [], Counter(), XDChainMetrics()
    trace_lock, stop = threading.Lock(), threading.Event()

    def trace(message):
        with trace_lock:
            traces.append(json.loads(message.data))
            del traces[:-512]

    subscriptions = [
        rospy.Subscriber(prefix+'/sar_yolo_smart_tracker_'+args.instance+'/pipeline_trace', String, trace, queue_size=5),
        rospy.Subscriber(prefix+'/detect/input/detections_2d', DetectionArray, bridge.receive_input, queue_size=10),
        rospy.Subscriber(prefix+'/track/tracks_by_source', TrackStateArray, bridge.receive_output, queue_size=10)]
    camera = rospy.Publisher(prefix+'/fixed_camera/image_raw', Image, queue_size=1)
    info = rospy.Publisher(prefix+'/fixed_camera/camera_info', CameraInfo, queue_size=1)
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = 'world'
    transform.child_frame_id = args.namespace+'_camera_optical'
    transform.transform.translation.z = 50.
    transform.transform.rotation.w = 1.
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transform)

    def publish():
        seq = 0
        while not stop.is_set() and not rospy.is_shutdown():
            h, w = image.shape[:2]
            # Small image-space camera jitter also exercises LK/background flow.
            shift = 2.*np.sin(seq/12.)
            frame = cv2.warpAffine(image, np.float32([[1,0,shift],[0,1,0]]), (w,h), borderMode=cv2.BORDER_REFLECT)
            message = Image(height=h, width=w, encoding='bgr8', step=w*3, data=frame.tobytes())
            message.header.seq, message.header.stamp, message.header.frame_id = seq, rospy.Time.now(), args.namespace+'_camera_optical'
            calibration = CameraInfo(header=message.header, height=h, width=w, distortion_model='plumb_bob')
            calibration.K = [740.,0.,w/2.,0.,740.,h/2.,0.,0.,1.]
            calibration.P = [740.,0.,w/2.,0.,0.,740.,h/2.,0.,0.,0.,1.,0.]
            calibration.R = [1.,0.,0.,0.,1.,0.,0.,0.,1.]
            calibration.D = [0.,0.,0.,0.,0.]
            info.publish(calibration); camera.publish(message)
            seq += 1
            stop.wait(.2)

    def request(body=None):
        path = '/api/frame' if body is None else '/api/command'
        data = None if body is None else json.dumps(body).encode()
        with urlopen(Request(args.web_url+path, data=data, headers={'Content-Type':'application/json'}), timeout=4.) as response:
            return json.load(response)

    publisher = threading.Thread(target=publish, daemon=True); publisher.start()
    report = dict(passed=False, test='ROS/Web four-mode smoke; no flight or accuracy claim', modes={})
    try:
        deadline = time.monotonic()+args.startup_timeout
        anchor = None
        while time.monotonic() < deadline:
            try:
                frame = request()
                objects = [obj for obj in frame.get('objects',[]) if obj.get('kind')=='detector_track']
                if frame.get('ready') and objects:
                    anchor = max(objects,key=lambda obj: obj.get('confidence',0.))['box']; break
            except (OSError, ValueError):
                pass
            time.sleep(.2)
        assert anchor is not None, 'No D-FINE detection reached the web within startup timeout'
        for mode in REQUIRED:
            result = request(dict(action='mode', mode=mode, color_name='roi'))
            assert result.get('success'), str(result)
            selected, last_command, seen = False, 0., set()
            deadline = time.monotonic()+max(5.,args.per_mode_sec)
            while time.monotonic() < deadline:
                frame = request()
                if frame.get('mode') != mode or not frame.get('frame_token'):
                    time.sleep(.1); continue
                token = frame['frame_token']
                if token not in seen:
                    seen.add(token)
                    events.append(dict(mode=mode,capture_header=frame.get('capture_header'),
                        selected_id=frame.get('selected_id'),selected_box=frame.get('selected_box'),
                        control_target_valid=frame.get('control_target_valid'),backend=frame.get('backend')))
                    if frame.get('selected_box') and frame.get('control_target_valid'):
                        locks[mode] += 1
                if not selected and time.monotonic()-last_command>.8:
                    action = dict(action='roi', frame_token=token, roi=anchor)
                    if mode in ('smart_tracker','fusion_tracker'):
                        candidates=[obj for obj in frame.get('objects',[]) if obj.get('kind')=='detector_track']
                        if not candidates:
                            time.sleep(.1); continue
                        box=max(candidates,key=lambda obj:obj.get('confidence',0.))['box']
                        action=dict(action='click',frame_token=token,point=[box[0]+box[2]/2.,box[1]+box[3]/2.])
                    try:
                        response=request(action); selected=bool(response.get('success'))
                    except (OSError, ValueError):
                        pass
                    last_command=time.monotonic()
                time.sleep(.1)
            report['modes'][mode]=dict(selected_via_web=selected,valid_lock_web_updates=locks[mode],web_updates=len(seen))
            assert selected and locks[mode]>0, 'No valid selection in '+mode
            print(mode, report['modes'][mode], flush=True)
        time.sleep(1.5)
        counts, backend = {}, Counter()
        with trace_lock:
            saved_traces = list(traces)
        for row in saved_traces:
            for mode, values in row['stage_counts_by_mode'].items():
                bucket=counts.setdefault(mode,Counter())
                for name,value in values.items():bucket[name]=max(bucket[name],value)
            for name,value in row['dfine_backend'].get('counts',{}).items():backend[name]=max(backend[name],value)
        for mode, required in REQUIRED.items():
            required=required+['tracked_output','xd_observation_output','selected_output']
            missing=[stage for stage in required if counts.get(mode,{}).get(stage,0)==0]
            assert not missing, mode+' skipped required stages: '+str(missing)
        for name in ('dfine_infer','configured_nms','external_tracker_update','temporal_class_consensus'):
            assert backend[name]>0, 'Backend stage missing: '+name
        assert any(row['smart_manager']['kalman_active'] and row['smart_manager']['appearance_active'] for row in saved_traces), 'Kalman/appearance model inactive'
        metrics=bridge.summary();values=metrics['counts']
        assert values.get('input_candidates',0)>0 and values.get('output_tracks',0)>0, 'Bridge/XD track output missing'
        assert values.get('classification_matches',0)>0 and values.get('classification_mismatches',0)==0, 'XD class consistency failed'
        assert all(event['backend'].startswith('dfine') for event in events
                   if event['mode'] in ('smart_tracker','fusion_tracker') and event.get('backend')), 'Unexpected detection backend'
        assert all(event['backend']=='opencv' for event in events
                   if event['mode'] in ('feature_tracker','color_tracker') and event.get('backend')), 'Manual mode did not use OpenCV'
        report.update(passed=True,stage_counts_by_mode={k:dict(v) for k,v in counts.items()},
            backend_counts=dict(backend),xd=metrics,smart_manager=saved_traces[-1]['smart_manager'])
    except Exception as error:
        report['error']=str(error)
        raise
    finally:
        stop.set(); publisher.join(timeout=3.)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        args.output.with_suffix('.trace.json').write_text(json.dumps(traces,indent=2)+'\n')
        args.output.with_suffix('.web.json').write_text(json.dumps(events,indent=2)+'\n')
        for subscription in subscriptions:subscription.unregister()
        print('Report:',args.output,flush=True)


if __name__ == '__main__':
    main()
