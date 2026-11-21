"""Strict RGB recording index. Does not infer thermal capability from pixels."""
import copy
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def file_sha256(path):
    digest=hashlib.sha256()
    with open(path,'rb') as stream:
        for chunk in iter(lambda:stream.read(4*1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()


def validate_source(source):
    if (source.get('modality')!='rgb' or source.get('thermal_sensor_model') is not False
            or source.get('encoding') not in ('bgr8','rgb8')):
        raise ValueError('This tool requires explicitly identified RGB source; no infrared claim or conversion')
    for key in ('image_topic','camera_info_topic','frame_id','producer'):
        if not source.get(key):raise ValueError('Missing RGB source field: '+key)
    return source


def manifest_path(bag):return Path(str(bag)+'.manifest.json')


def load_manifest(bag):
    bag=Path(bag);value=json.loads(manifest_path(bag).read_text())
    if value.get('schema_version')!=1 or not value.get('validated'):raise ValueError('Recording was not validated')
    validate_source(value['source'])
    if value['bag_sha256']!=file_sha256(bag):raise ValueError('Recording integrity mismatch')
    return value


def source_from_bag(bag,topic):
    sources=[json.loads(msg.data) for _,msg,_ in bag.read_messages(topics=[topic])]
    if not sources:raise ValueError('Missing recorded RGB source provenance')
    first=validate_source(sources[0])
    if any(value!=first for value in sources):raise ValueError('Recording crossed RGB source/session boundaries')
    return first


def calibration_index(bag,source,maximum=36000):
    """Small metadata index only; raw images are never cached as a video array."""
    validate_source(source);infos={};transforms={};frame=source['frame_id']
    for _,msg,_ in bag.read_messages(topics=[source['camera_info_topic']]):
        stamp=msg.header.stamp.to_nsec()
        k=np.asarray(msg.K,float)
        if (stamp<=0 or msg.header.frame_id!=frame or msg.width<=0 or msg.height<=0
                or not np.isfinite(k).all() or k[0]<=0 or k[4]<=0 or k[8]!=1
                or any(not math.isfinite(v) or v!=0 for v in msg.D)):
            raise ValueError('Invalid exact-capture RGB calibration')
        if stamp in infos and str(infos[stamp])!=str(msg):raise ValueError('Conflicting same-capture calibration')
        infos[stamp]=msg
        if len(infos)>maximum:raise ValueError('Split recording: metadata index capacity exceeded')
    for _,msg,_ in bag.read_messages(topics=['/tf']):
        for value in msg.transforms:
            if value.child_frame_id!=frame:continue
            stamp=value.header.stamp.to_nsec();t=value.transform.translation;q=value.transform.rotation
            if not all(math.isfinite(v) for v in [t.x,t.y,t.z,q.x,q.y,q.z,q.w]):raise ValueError('Non-finite camera pose')
            if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-3:raise ValueError('Invalid camera quaternion')
            if stamp in transforms and str(transforms[stamp])!=str(value):raise ValueError('Conflicting same-capture pose')
            transforms[stamp]=value
            if len(transforms)>maximum:raise ValueError('Split recording: pose index capacity exceeded')
    if not infos or not transforms:raise ValueError('Missing capture calibration or camera pose')
    return infos,transforms


def validate_frame(image,info,transform,source):
    if (image.encoding!=source['encoding'] or image.header.frame_id!=source['frame_id']
            or image.width!=info.width or image.height!=info.height
            or image.header.stamp!=info.header.stamp or image.header.stamp!=transform.header.stamp
            or image.step<image.width*3 or len(image.data)!=image.step*image.height):
        raise ValueError('RGB image/calibration/pose capture mismatch')


def inspect_bag(bag,source):
    infos,transforms=calibration_index(bag,source);previous=0;count=0;first=0;last=0;dimensions=set()
    for _,image,_ in bag.read_messages(topics=[source['image_topic']]):
        stamp=image.header.stamp.to_nsec()
        if stamp<=previous:raise ValueError('Non-increasing original RGB capture time')
        if stamp not in infos or stamp not in transforms:raise ValueError('Image lacks its own exact-capture calibration/pose')
        validate_frame(image,infos[stamp],transforms[stamp],source)
        if not count:first=stamp
        previous=last=stamp;count+=1;dimensions.add((image.width,image.height))
    if count<2:raise ValueError('Recording has fewer than two RGB captures')
    return dict(frames=count,paired_calibrations=count,paired_camera_poses=count,
        first_capture_nsec=first,last_capture_nsec=last,duration_sec=(last-first)/1e9,
        effective_fps=(count-1)/((last-first)/1e9),dimensions=[list(v) for v in sorted(dimensions)])


def write_manifest(path,source):
    import rosbag
    path=Path(path)
    if manifest_path(path).exists():raise FileExistsError(str(manifest_path(path)))
    with rosbag.Bag(str(path)) as bag:summary=inspect_bag(bag,source)
    result=dict(schema_version=1,validated=True,modality='rgb',source=validate_source(source),
        bag_file=path.name,bag_sha256=file_sha256(path),summary=summary,
        timing='Original capture-time Image/CameraInfo/camera TF; bag receive times retained separately',
        infrared_validated=False)
    with manifest_path(path).open('x') as stream:json.dump(result,stream,ensure_ascii=False,indent=2);stream.write('\n')
    return result


def remap_capture(image,info,transform,target_stamp,target_frame,target_world):
    """Copy a coherent tuple; preserve recorded evidence and never mutate bag messages."""
    image,info,transform=copy.deepcopy(image),copy.deepcopy(info),copy.deepcopy(transform)
    for value in (image,info):value.header.stamp=target_stamp;value.header.frame_id=target_frame
    transform.header.stamp=target_stamp;transform.header.frame_id=target_world;transform.child_frame_id=target_frame
    return image,info,transform


def image_bgr(image):
    if image.encoding not in ('bgr8','rgb8'):raise ValueError('Expected RGB recording')
    pixels=np.frombuffer(image.data,np.uint8).reshape(image.height,image.step)[:,:image.width*3].reshape(image.height,image.width,3)
    return np.ascontiguousarray(pixels[:,:,::-1] if image.encoding=='rgb8' else pixels)
