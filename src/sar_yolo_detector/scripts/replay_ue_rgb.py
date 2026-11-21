#!/usr/bin/python3
"""Replay only RGB + exact per-capture K/TF into an isolated perception namespace.

Recorded detector/tracker outputs, commands, MAVROS and gimbal feedback are
analysis evidence only and are NEVER republished by this tool.
"""
import argparse
import json
from pathlib import Path
import time
import rospy
import rosbag
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from sar_yolo_detector.rgb_recording import load_manifest,calibration_index,validate_frame,remap_capture


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',required=True,type=Path);p.add_argument('--namespace',default='rgb_replay')
    p.add_argument('--rate',type=float,default=1.);p.add_argument('--world-frame',default='rgb_replay_world')
    p.add_argument('--wait-for-node',default='');p.add_argument('--startup-timeout',type=float,default=240.)
    p.add_argument('--summary',required=True,type=Path);p.add_argument('--mapping',type=Path)
    p.add_argument('--selection-validation',type=Path)
    a=p.parse_args(rospy.myargv()[1:])
    if not .1<=a.rate<=4:p.error('Replay rate must be in [.1,4]')
    a.namespace=a.namespace.strip('/')
    ns='/'+a.namespace
    if ns in ('/ue_test','/uav1'):p.error('Use a dedicated replay namespace')
    value=load_manifest(a.bag);source=value['source']
    if source['image_topic'].startswith(ns+'/'):p.error('Replay must not overwrite the recorded source namespace')
    selected_original=None
    if a.selection_validation:
        validated=json.loads(a.selection_validation.read_text())
        if not validated.get('passed') or validated['bag_sha256']!=value['bag_sha256']:
            raise ValueError('Selection validation does not match the recording')
        selected_original=validated['selected_capture']['capture_nsec']
    rospy.init_node('ue_rgb_replay')
    processed=set()
    def acknowledge(message):
        evidence=json.loads(message.data)
        h=evidence.get('capture_header',{}).get('stamp',{})
        if h:
            processed.add(h['secs']*10**9+h['nsecs'])
    subscription=rospy.Subscriber(ns+'/sar_yolo_smart_tracker_dfine_v3/fusion_evidence',
                                  String,acknowledge,queue_size=32)
    if rospy.get_param('/use_sim_time',False):raise RuntimeError('Wall-time retimed replay requires /use_sim_time=false')
    image_pub=rospy.Publisher(ns+'/fixed_camera/image_raw',Image,queue_size=1)
    info_pub=rospy.Publisher(ns+'/fixed_camera/camera_info',CameraInfo,queue_size=1)
    tf_pub=rospy.Publisher('/tf',TFMessage,queue_size=32)
    source_pub=rospy.Publisher(ns+'/camera/source',String,queue_size=1,latch=True)
    source_pub.publish(String(data=json.dumps(dict(source,producer='RGB recording replay',is_replay=True,transport='rosbag_retimed_rgb',
        original_source=source,bag_sha256=value['bag_sha256'],image_topic=ns+'/fixed_camera/image_raw',
        camera_info_topic=ns+'/fixed_camera/camera_info',frame_id=a.namespace+'_down_optical'))))
    summary=dict(passed=False,modality='rgb',is_replay=True,rate=a.rate,frames=0,
        published_topics=[ns+'/fixed_camera/image_raw',ns+'/fixed_camera/camera_info','/tf',ns+'/camera/source'],
        recorded_algorithm_outputs_republished=False,flight_control_started=False)
    a.summary.parent.mkdir(parents=True,exist_ok=True);mapping=None
    selected_replay=None
    try:
        with rosbag.Bag(str(a.bag)) as bag:
            infos,transforms=calibration_index(bag,source)
            deadline=time.monotonic()+a.startup_timeout
            while not rospy.is_shutdown():
                _,subscribers,_=rospy.get_master().getSystemState()[2]
                nodes=next((nodes for topic,nodes in subscribers if topic==ns+'/fixed_camera/image_raw'),[])
                ready=(a.wait_for_node in nodes) if a.wait_for_node else bool(nodes)
                if ready and info_pub.get_num_connections()>0 and tf_pub.get_num_connections()>0:break
                if time.monotonic()>deadline:raise RuntimeError('Replay perception subscriber not ready')
                time.sleep(.05)
            if rospy.is_shutdown():raise RuntimeError('Replay interrupted before input was ready')
            origin=None;anchor=rospy.Time.now().to_nsec()+200000000;wall_anchor=time.monotonic()+.2
            previous=0;target_frame=a.namespace+'_down_optical';late=0;maximum_late=0.
            if a.mapping:
                a.mapping.parent.mkdir(parents=True,exist_ok=True);mapping=a.mapping.open('x')
            for _,image,_ in bag.read_messages(topics=[source['image_topic']]):
                original=image.header.stamp.to_nsec()
                if original<=previous:raise ValueError('Non-increasing original capture time')
                previous=original
                if origin is None:origin=original
                info=infos[original];transform=transforms[original]
                validate_frame(image,info,transform,source)
                delta=(original-origin)/1e9/a.rate;due=wall_anchor+delta
                while time.monotonic()<due and not rospy.is_shutdown():time.sleep(max(0.,min(.01,due-time.monotonic())))
                if rospy.is_shutdown():raise RuntimeError('Replay interrupted')
                delay=time.monotonic()-due
                if delay>.05:late+=1
                maximum_late=max(maximum_late,delay)
                target_nsec=anchor+int(round((original-origin)/a.rate))
                stamp=rospy.Time(target_nsec//1000000000,target_nsec%1000000000)
                if original==selected_original:
                    selected_replay=stamp.to_nsec()
                im,cal,pose=remap_capture(image,info,transform,stamp,target_frame,a.world_frame)
                tf_pub.publish(TFMessage(transforms=[pose]));info_pub.publish(cal);image_pub.publish(im)
                summary['frames']+=1
                if mapping:mapping.write(json.dumps(dict(original_capture_nsec=original,replay_capture_nsec=stamp.to_nsec(),seq=im.header.seq))+'\n')
            summary.update(passed=summary['frames']==value['summary']['frames'],
                late_frames_over_50ms=late,maximum_schedule_lateness_ms=maximum_late*1000.,
                input_span_sec=(previous-origin)/1e9 if origin else 0.)
            if not summary['passed']:raise RuntimeError('Replay frame count mismatch')
            rospy.sleep(.25)
            if selected_original is not None:
                deadline=time.monotonic()+5
                while selected_replay not in processed and time.monotonic()<deadline and not rospy.is_shutdown():
                    rospy.sleep(.05)
                summary['initial_capture_processed']=selected_replay is not None and selected_replay in processed
                summary['initial_original_capture_nsec']=selected_original
                summary['initial_replay_capture_nsec']=selected_replay
                summary['passed']=summary['passed'] and summary['initial_capture_processed']
                if not summary['passed']:
                    raise RuntimeError('Selected original capture has no replay tracker acknowledgement')
    finally:
        if mapping:mapping.close()
        a.summary.write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
