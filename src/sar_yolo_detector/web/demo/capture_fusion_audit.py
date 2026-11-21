#!/usr/bin/env python3
"""Opt-in bounded ROS capture for diagnosing selection, gaps and fusion vetoes."""
import argparse
import collections
import json
import queue
import threading
import time
from pathlib import Path

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sar_yolo_detector.msg import SmartTrackerState


def stamp_of(payload):
    """Return only an explicitly supplied capture clock; absent is unknown."""
    header=payload.get('capture_header') or {}
    value=header.get('stamp') if isinstance(header,dict) else None
    if value is None:
        value=payload.get('stamp')
    if isinstance(value,dict) and 'secs' in value and 'nsecs' in value:
        return int(value['secs'])+int(value['nsecs'])/1e9
    if isinstance(value,(int,float)) and not isinstance(value,bool):
        return float(value)
    return None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,help='New output directory')
    parser.add_argument('--namespace',default='/ue_test')
    parser.add_argument('--instance',default='dfine_v3')
    parser.add_argument('--image-topic',default=None,help='Raw image topic; records header only, never image bytes')
    parser.add_argument('--seconds',type=float,default=30.)
    args=parser.parse_args()
    if args.seconds<=0:parser.error('--seconds must be positive')
    directory=Path(args.output);directory.mkdir(parents=True,exist_ok=False)
    ns='/'+args.namespace.strip('/')
    prefix=ns+'/sar_yolo_smart_tracker_'+args.instance
    image_topic=args.image_topic or ns+'/fixed_camera/image_raw'
    records=queue.Queue(maxsize=2048);counts=collections.Counter();drops=collections.Counter();lock=threading.Lock();selected=[]
    handle=(directory/'messages.jsonl').open('x',encoding='utf-8')
    def writer():
        while True:
            record=records.get()
            if record is None:break
            handle.write(json.dumps(record,ensure_ascii=False,allow_nan=False,separators=(',',':'))+'\n')
        handle.flush();handle.close()
    thread=threading.Thread(target=writer,daemon=True);thread.start()
    def emit(channel,capture,payload):
        item={'channel':channel,'received_wall':time.time(),'capture_stamp':capture,'payload':payload}
        with lock:
            counts[channel]+=1
        try:records.put_nowait(item)
        except queue.Full:
            with lock:drops[channel]+=1
    def json_callback(channel):
        def callback(message):
            try:payload=json.loads(message.data)
            except (ValueError,TypeError):payload={'parse_error':True}
            if not isinstance(payload,dict):payload={'value':payload}
            # Pipeline trace stamp is publication time, not a capture stamp.
            capture=None if channel=='pipeline_trace' else stamp_of(payload)
            emit(channel,capture,payload)
        return callback
    def image_callback(message):
        stamp=message.header.stamp.to_sec()
        emit('camera_image',stamp,{'seq':message.header.seq,'frame_id':message.header.frame_id,
                                  'width':message.width,'height':message.height})
    def state_callback(message):
        stamp=message.header.stamp.to_sec()
        box=message.selected_bbox
        payload={'seq':message.header.seq,'frame_id':message.header.frame_id,'selected_id':message.selected_track_id,
                 'has_selection':message.has_selection,'valid':message.control_measurement_ready,
                 'reason':message.freshness_reason,'source':message.association_method,
                 'box':[box.center.x,box.center.y,box.size_x,box.size_y],
                 'inference_ms':message.inference_ms,'processing_ms':message.frame_processing_ms}
        with lock:
            selected.append((stamp,int(message.selected_track_id),bool(message.has_selection),bool(message.control_measurement_ready)))
        emit('state',stamp,payload)
    rospy.init_node('fusion_audit',anonymous=True,disable_signals=True)
    subs=[rospy.Subscriber(image_topic,Image,image_callback,queue_size=1,buff_size=16*1024*1024,tcp_nodelay=True),
          rospy.Subscriber(ns+'/sar_yolo_detector/'+args.instance+'/state',SmartTrackerState,state_callback,queue_size=20)]
    for channel in ('pipeline_trace','fusion_loss_history','fusion_evidence','classification_evidence'):
        subs.append(rospy.Subscriber(prefix+'/'+channel,String,json_callback(channel),queue_size=30))
    deadline=time.monotonic()+args.seconds
    try:
        while not rospy.is_shutdown() and time.monotonic()<deadline:time.sleep(.1)
    except KeyboardInterrupt:pass
    finally:
        for sub in subs:sub.unregister()
        records.put(None);thread.join()
        with lock:ordered=sorted(selected);counts_copy=dict(counts);drops_copy=dict(drops)
        transitions=[];previous=None
        for stamp,identity,has_selection,valid in ordered:
            current=identity if has_selection else -1
            if current!=previous:
                transitions.append({'capture_stamp':stamp,'selected_id':current,'valid':valid})
                previous=current
        summary={'duration_requested_sec':args.seconds,'topics':{'camera_image':image_topic,'node_prefix':prefix},
                 'received_by_channel':counts_copy,'writer_queue_drops_by_channel':drops_copy,
                 'missing_channels':[x for x in ('camera_image','state','pipeline_trace','fusion_loss_history','fusion_evidence','classification_evidence') if not counts_copy.get(x)],
                 'selection_transitions':transitions,
                 'limitations':['No independent target truth; ID changes are not automatically false locks.',
                    'Pipeline trace is periodic and its stamp is publication time.',
                    'Fusion evidence may be rate limited; veto details between messages can be unknown.',
                    'No image pixels are stored; capture gaps can include subscriber queue loss.']}
        (directory/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(summary,ensure_ascii=False))


if __name__=='__main__':main()
