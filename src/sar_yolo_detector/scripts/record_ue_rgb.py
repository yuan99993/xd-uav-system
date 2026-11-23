#!/usr/bin/python3
"""Record RGB evidence without starting/modifying perception or flight control."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import uuid
import time
import rospy
from std_msgs.msg import String
from sar_yolo_detector.rgb_recording import validate_source,source_from_bag,write_manifest,recording_source


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--namespace',default='ue_test');p.add_argument('--instance',default='dfine_v3')
    p.add_argument('--seconds',type=float,default=75);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--ready-file',type=Path)
    p.add_argument('--capture-ack',action='store_true')
    a=p.parse_args(rospy.myargv()[1:])
    if not 1<=a.seconds<=600:p.error('record duration must be in [1,600] seconds')
    if a.output.suffix!='.bag':p.error('--output must end in .bag')
    if a.output.exists() or Path(str(a.output)+'.active').exists():p.error('Recording already exists')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    rospy.init_node('record_ue_rgb_supervisor',anonymous=True,disable_signals=a.capture_ack)
    ns='/'+a.namespace.strip('/');source_topic=ns+'/camera/source'
    source=validate_source(json.loads(rospy.wait_for_message(source_topic,String,timeout=30).data))
    if source.get('is_replay') or source['producer']!='UE.SceneCapture2D.FinalColorLDR':
        raise ValueError('Expected actual UE RGB source, not a replay or unvalidated sensor')
    if a.capture_ack:
        source=recording_source(source)
        record_source_topic=ns+'/camera/recording_source'
        record_source_publisher=rospy.Publisher(record_source_topic,String,queue_size=1,latch=True)
        record_source_publisher.publish(String(data=json.dumps(source)))
    topics=['/tf','/tf_static',source_topic,source['camera_info_topic'],source['image_topic'],
        ns+'/mission_status',ns+'/gimbal/state',ns+'/target_geometry',ns+'/benchmark/command',
        ns+'/sar_yolo_detector/'+a.instance+'/state',
        ns+'/sar_yolo_detector/'+a.instance+'/tracked_detections',
        ns+'/sar_yolo_detector/'+a.instance+'/tracked_detections/observations',
        ns+'/sar_yolo_smart_tracker_'+a.instance+'/fusion_evidence',
        ns+'/sar_yolo_smart_tracker_'+a.instance+'/pipeline_trace',
        ns+'/sar_yolo_smart_tracker_'+a.instance+'/classification_evidence',
        ns+'/detect/input/detections_2d',ns+'/track/tracks_by_source']
    if a.capture_ack:
        topics.append(record_source_topic)
        from sar_yolo_detector.capture_recording import record_acknowledged
        record_acknowledged(a.output,source,topics,a.seconds,a.ready_file,ns)
        import rosbag
        with rosbag.Bag(str(a.output)) as bag:recorded=source_from_bag(bag,record_source_topic)
        if recorded!=source:raise ValueError('RGB session changed during recording')
        print(json.dumps(write_manifest(a.output,source),ensure_ascii=False,indent=2))
        return
    command=['rosbag','record','--lz4','--buffsize=128','--duration='+str(a.seconds),
        '-O',str(a.output),'__name:=ue_rgb_record_'+uuid.uuid4().hex[:8]]+topics
    child=subprocess.Popen(command)
    try:
        if a.ready_file:
            from sensor_msgs.msg import Image
            deadline=time.monotonic()+30
            recorder_name='/'+next(v.split(':=',1)[1] for v in command if v.startswith('__name:='))
            while time.monotonic()<deadline:
                if child.poll() is not None:raise RuntimeError('Recorder exited before ready')
                # Master registration precedes TCPROS handshake. Require the
                # recorder's actual inbound transports, including evidence.
                from xmlrpc.client import ServerProxy
                code, _, uri=rospy.get_master().lookupNode(recorder_name)
                required={'/tf',source['image_topic'],source['camera_info_topic'],
                    ns+'/sar_yolo_smart_tracker_'+a.instance+'/fusion_evidence'}
                connected=set()
                if code==1:
                    try:
                        status, _, buses=ServerProxy(uri).getBusInfo(rospy.get_name())
                        if status==1:
                            connected={row[4] for row in buses if row[2]=='i' and row[5]}
                    except (OSError,ValueError):pass
                if required.issubset(connected):break
                rospy.sleep(.05)
            else:
                child.send_signal(signal.SIGINT)
                child.wait(timeout=30)
                raise RuntimeError('Recorder image subscription did not become ready')
            first=rospy.wait_for_message(source['image_topic'],Image,timeout=30)
            a.ready_file.write_text(json.dumps(dict(ready=True,after_capture_nsec=first.header.stamp.to_nsec(),
                                                   session=source.get('session')))+'\n')
        while child.poll() is None and not rospy.is_shutdown():rospy.sleep(.1)
    finally:
        if child.poll() is None:child.send_signal(signal.SIGINT)
        code=child.wait(timeout=30)
    if code not in (0,-signal.SIGINT):raise RuntimeError('RGB recorder failed: '+str(code))
    import rosbag
    with rosbag.Bag(str(a.output)) as bag:recorded=source_from_bag(bag,source_topic)
    if recorded!=source:raise ValueError('RGB session changed during recording')
    print(json.dumps(write_manifest(a.output,source),ensure_ascii=False,indent=2))

if __name__=='__main__':main()
