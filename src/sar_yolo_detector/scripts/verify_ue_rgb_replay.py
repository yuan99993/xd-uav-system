#!/usr/bin/python3
"""Read-only input provenance/K/TF and flight-isolation audit for RGB replay."""
import argparse
import json
from pathlib import Path
import struct
import time
import threading
import rospy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--namespace',default='rgb_replay')
    p.add_argument('--seconds',type=float,default=30);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args(rospy.myargv()[1:]);ns='/'+a.namespace.strip('/')
    rospy.init_node('verify_rgb_replay',anonymous=True)
    images={};infos={};poses={};first=[];recording=[True];lock=threading.RLock()
    def image(msg):
        seq,sec,nsec,size=struct.unpack_from('<IIII',msg._buff)
        frame=msg._buff[16:16+size].decode()
        with lock:
            if not recording[0]:return
            images[(sec*1000000000+nsec,frame)]=seq
            if not first:first.append(time.monotonic())
    def info(msg):infos[(msg.header.stamp.to_nsec(),msg.header.frame_id)]=(msg.width,msg.height,list(msg.K))
    def tf(msg):
        for v in msg.transforms:
            if v.child_frame_id==a.namespace.strip('/')+'_down_optical':
                poses[(v.header.stamp.to_nsec(),v.child_frame_id)]=v.header.frame_id
    subs=[rospy.Subscriber('/tf',TFMessage,tf,queue_size=100),
        rospy.Subscriber(ns+'/fixed_camera/camera_info',CameraInfo,info,queue_size=100),
        rospy.Subscriber(ns+'/fixed_camera/image_raw',rospy.AnyMsg,image,queue_size=100,buff_size=8*1024*1024)]
    source=json.loads(rospy.wait_for_message(ns+'/camera/source',String,timeout=240).data)
    deadline=time.monotonic()+240
    while not rospy.is_shutdown():
        if first and time.monotonic()-first[0]>=a.seconds:break
        if time.monotonic()>deadline:raise RuntimeError('No replay RGB input within deadline')
        time.sleep(.05)
    with lock:recording[0]=False
    subs[-1].unregister()
    # Freeze the image observation window before draining small K/TF messages;
    # otherwise continuously arriving new images create an artificial tail miss.
    time.sleep(.2)
    publishers,subscribers,services=rospy.get_master().getSystemState()[2]
    image_publishers=next((v for topic,v in publishers if topic==ns+'/fixed_camera/image_raw'),[])
    forbidden=[topic for topic,nodes in publishers if '/mavros/' in topic or '/gimbal/command' in topic or topic.startswith('/ue_test/')]
    keys=set(images);paired=keys&set(infos)&set(poses)
    result=dict(passed=False,images=len(keys),paired_k_and_tf=len(paired),missing_pairs=[list(v) for v in sorted(keys-paired)],source=source,
        image_publishers=image_publishers,forbidden_live_or_control_topics=forbidden,
        perception_only=rospy.get_param(ns+'/track/runtime/perception_only',False),
        publish_control_reference=rospy.get_param(ns+'/track/runtime/publish_control_reference',True),
        optical_frames=sorted(set(frame for stamp,frame in keys)),camera_world_frames=sorted(set(poses[k] for k in paired)))
    result['passed']=bool(len(keys)>=100 and len(paired)==len(keys) and source['modality']=='rgb'
        and source['is_replay'] and not source['thermal_sensor_model'] and not forbidden
        and result['perception_only'] and not result['publish_control_reference']
        and len(image_publishers)==1)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));assert result['passed'],'Replay isolation/calibration audit failed'

if __name__=='__main__':main()
