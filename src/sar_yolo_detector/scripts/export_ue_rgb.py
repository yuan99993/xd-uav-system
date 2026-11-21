#!/usr/bin/python3
"""Export validated RGB pixels plus original capture metadata for video analysis."""
import argparse
import json
from pathlib import Path
import cv2
import rosbag
from sar_yolo_detector.rgb_recording import load_manifest,calibration_index,validate_frame,image_bgr


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bag',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();value=load_manifest(a.bag);source=value['source']
    if a.output.suffix.lower()!='.mp4':p.error('Use .mp4 for RGB analysis export')
    sidecar=Path(str(a.output)+'.frames.jsonl');summary_path=Path(str(a.output)+'.json')
    if any(path.exists() for path in [a.output,sidecar,summary_path]):raise FileExistsError('Export destination exists')
    if len(value['summary']['dimensions'])!=1:raise ValueError('Split variable-resolution captures before video export')
    w,h=value['summary']['dimensions'][0];fps=value['summary']['effective_fps']
    a.output.parent.mkdir(parents=True,exist_ok=True);cv2.setNumThreads(1)
    writer=cv2.VideoWriter(str(a.output),cv2.VideoWriter_fourcc(*'mp4v'),fps,(w,h))
    if not writer.isOpened():raise RuntimeError('MP4 encoder unavailable')
    count=0
    try:
        with rosbag.Bag(str(a.bag)) as bag,sidecar.open('x') as records:
            infos,transforms=calibration_index(bag,source)
            for _,image,received in bag.read_messages(topics=[source['image_topic']]):
                stamp=image.header.stamp.to_nsec();info=infos[stamp];pose=transforms[stamp]
                validate_frame(image,info,pose,source);writer.write(image_bgr(image))
                t=pose.transform.translation;q=pose.transform.rotation
                records.write(json.dumps(dict(index=count,capture_nsec=stamp,bag_receive_nsec=received.to_nsec(),
                    frame_id=image.header.frame_id,width=image.width,height=image.height,K=list(info.K),
                    world_frame=pose.header.frame_id,camera_position=[t.x,t.y,t.z],
                    camera_quaternion_xyzw=[q.x,q.y,q.z,q.w],modality='rgb'))+'\n');count+=1
    finally:writer.release()
    if count!=value['summary']['frames']:raise RuntimeError('Export frame count mismatch')
    result=dict(passed=True,frames=count,width=w,height=h,modality='rgb',infrared_validated=False,
        video_fps=fps,original_capture_span_sec=value['summary']['duration_sec'],
        timing='MP4 uses average capture FPS for visual analysis; exact variable capture times/K/pose are in frames.jsonl and the lossless rosbag',
        bag_sha256=value['bag_sha256'])
    summary_path.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))

if __name__=='__main__':main()
