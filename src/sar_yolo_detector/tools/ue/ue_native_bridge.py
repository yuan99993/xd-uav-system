#!/usr/bin/env python3
"""UE5.5 SceneCapture2D/RenderTarget frames -> ROS perception topics.

No AirSim/UnrealCV binary is required. The editor renders either a kinematic mission or actual MAVROS pose feedback.
In PX4 mode, a separate Gazebo/PX4 process computes the flight dynamics.
"""
import argparse
import json
import math
from pathlib import Path
import time
import uuid
import shutil
import cv2
import statistics
import subprocess
import numpy as np
from common import ROOT,workload_lock,write_json
from geometry import route_pose,unreal_camera_optical_rotation
from xd_uav_track.mission import Mission


def exchange(root,request,timeout=180):
    request_started=time.perf_counter()
    write_json(root/"request.json",request)
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        path=root/"response.json"
        if path.exists():
            try:response=json.loads(path.read_text())
            except (json.JSONDecodeError,PermissionError):response={}
            if (response.get("session"),response.get("id"))==(request["session"],request["id"]):
                if not response["ok"]:
                    if response.get('retryable'):
                        time.sleep(.1);write_json(root/'request.json',request);continue
                    raise RuntimeError(response.get("error","UE capture failed"))
                response["bridge_exchange_ms"]=(time.perf_counter()-request_started)*1000.
                return response
        # The editor itself ticks at 30 Hz; a 10 ms client poll added avoidable
        # file-share wait to every response. Keep the poll short without busy-looping.
        time.sleep(.004)
    raise RuntimeError("UE editor response timeout; start editor_server.py in the normal editor")


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--project",type=Path,required=True)
    p.add_argument("--scene",type=Path)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--seconds",type=float,default=60.)
    p.add_argument("--fps",type=float,default=3.)
    p.add_argument("--width",type=int,default=960)
    p.add_argument("--height",type=int,default=540)
    p.add_argument("--image-format",choices=["png","jpg"],default="png")
    p.add_argument("--max-frames",type=int,default=0)
    p.add_argument('--capture-timeout',type=float,default=180.)
    p.add_argument('--camera',choices=('down','front','both'),default='both',
                   help='UE camera(s) to capture; down is the four-mode detector input')
    p.add_argument('--camera-world-frame',default='ue_render_ned')
    p.add_argument("--retain-frames",type=int,default=0,help="0 keeps all images; positive retains a rolling camera buffer")
    p.add_argument("--stream-only",action="store_true",help="Publish frames without duplicate RGB copies on disk")
    p.add_argument("--ros",action="store_true")
    p.add_argument("--pose-source",choices=["kinematic","px4"],default="kinematic")
    p.add_argument("--mavros-namespace",default="/uav1")
    p.add_argument("--namespace",default="ue_test")
    a=p.parse_args()
    if a.out.exists():p.error("Use a new recording directory")
    if a.seconds<=0 or a.fps<=0:p.error("seconds/fps must be positive")
    if not 320<=a.width<=1920 or not 240<=a.height<=1080:p.error("Capture dimensions outside supported bounds")
    if a.pose_source=="px4" and not a.ros:p.error("PX4 feedback requires --ros")
    root=a.project/"Saved/AirGroundExchange"
    registry=ROOT/'runtime/ue_editor_sources.json'
    sources=json.loads(registry.read_text()) if registry.exists() else []
    if str((root/'ready.json').resolve()) not in sources:
        write_json(registry,sources+[str((root/'ready.json').resolve())])
    ready=json.loads((root/"ready.json").read_text())
    if abs(time.time()-ready["heartbeat"])>10:raise RuntimeError("UE editor capture server is not running")
    # Minimized Slate windows throttle capture even with background CPU limits
    # disabled. Restore the editor without taking foreground focus.
    if shutil.which('powershell.exe') and shutil.which('wslpath'):
        native_paths=[subprocess.check_output(['wslpath','-w',str(path)],text=True).strip()
            for path in (Path(__file__).with_name('restore_ue_window.ps1'),root/'ready.json')]
        restored=subprocess.run(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass',
            '-File',native_paths[0],'-ReadyPath',native_paths[1]],capture_output=True,text=True)
        if restored.returncode:
            raise RuntimeError('UE capture window preflight failed: '+restored.stderr.strip())
        print('UE capture window:',restored.stdout.strip(),flush=True)
    spec=json.loads((a.scene or a.project/"Scripts/AirGround/scene.json").read_text())
    mission=Mission(spec['mission']) if 'mission' in spec else None
    all_cameras=[("down",-70.,"fixed_camera"),("front",-10.,"gimbal_camera")]
    cameras=[camera for camera in all_cameras if a.camera=='both' or camera[0]==a.camera]
    pubs={}
    if a.ros:
        import rospy
        from sensor_msgs.msg import Image,CameraInfo
        from std_msgs.msg import String
        from geometry_msgs.msg import TransformStamped
        import tf2_ros
        from tf.transformations import quaternion_from_matrix
        rospy.init_node("ue_native_bridge")
        if rospy.get_param('/use_sim_time',False):raise RuntimeError("Native bridge uses wall-time stamps")
        mission_pub=rospy.Publisher(f'/{a.namespace}/mission_status',String,queue_size=1,latch=True)
        camera_tf=tf2_ros.TransformBroadcaster()
        for name,pitch,topic in cameras:
            pubs[name]=(rospy.Publisher(f'/{a.namespace}/{topic}/image_raw',Image,queue_size=1),
                        rospy.Publisher(f'/{a.namespace}/{topic}/camera_info',CameraInfo,queue_size=1))
    feedback=None
    if a.pose_source=='px4':
        from xd_uav_track.px4_pose import PoseFeedback
        feedback=PoseFeedback(a.mavros_namespace,spec['mission']['home_ned_m'],rospy,status_namespace=a.namespace)
        deadline=time.monotonic()+240
        while True:
            if rospy.is_shutdown():raise RuntimeError('Capture shutdown while waiting for PX4')
            try:feedback.sample();break
            except RuntimeError:
                if time.monotonic()>deadline:raise
                time.sleep(.1)
    session=uuid.uuid4().hex;a.out.mkdir(parents=True)
    write_json(a.out/'run.json',{"scene":spec,"cameras":[camera[0] for camera in cameras],"kind":"UE5.5_PX4_Gazebo_feedback_capture" if feedback else "UE5.5_native_editor_kinematic_capture",
        "ground_truth":"3D object poses; no visibility masks or calibrated 3D detector positions",
        "recognition_validated":False,"flight_dynamics_validated":False,"session":session})
    count=0;request_id=0;latencies=[];stage_samples={};capture_ticks=[]
    def record_stage(name,value):
        if value is not None:stage_samples.setdefault(name,[]).append(float(value))
    with workload_lock("native_capture_controller",lock_path=ROOT/'runtime/ue_capture_controller.lock'), \
         workload_lock("native_ue_capture",shared=True),(a.out/'frames.jsonl').open('w') as record:
        start=time.monotonic()
        try:
            while time.monotonic()-start<a.seconds and (not a.max_frames or count<a.max_frames):
                if a.ros and rospy.is_shutdown():break
                tick=time.monotonic();t=tick-start
                if feedback:
                    position,yaw,status=feedback.wait_sample()
                elif mission:
                    position,yaw,status=mission.sample(t)
                else:
                    position,yaw=route_pose(spec['observer']['route'],t*spec['observer']['speed_mps'])
                    status={'stage':'legacy_route','complete':False}
                status.update(observer_ned_m=position,yaw_deg=math.degrees(yaw))
                write_json(a.out/'mission_status.json',status)
                if a.ros:mission_pub.publish(String(data=json.dumps(status)))
                objects={e['id']:[v+speed*20*math.sin(t/20) for v,speed in zip(e['position'],e['velocity'])]
                         for e in spec['entities']}
                if spec.get('observer_visual_actor'):objects[spec['observer_visual_actor']]=position
                frame={"frame":count,"route_time_sec":t,"observer_ned_m":position,"mission":status,"cameras":{}}
                for name,pitch,topic in cameras:
                    request_id+=1
                    # Sample real feedback separately for each camera render.
                    if feedback:
                        position,yaw,status=feedback.wait_sample()
                        objects[spec['observer_visual_actor']]=position
                    # Stabilized gimbal: fixed pitch/roll, measured body heading.
                    camera_position=[position[0]+1.2*math.cos(yaw),position[1]+1.2*math.sin(yaw),position[2]+.2]
                    response=exchange(root,{"id":request_id,"session":session,"op":"capture","objects":objects,
                        "object_yaw_deg":{spec["observer_visual_actor"]:math.degrees(yaw)} if spec.get("observer_visual_actor") else {},
                        "object_rotation_deg":{spec["observer_visual_actor"]:status["rotation_deg"]} if feedback and spec.get("observer_visual_actor") else {},
                        "camera_position_ned_m":camera_position,"camera_pitch_yaw_deg":[pitch,math.degrees(yaw)],
                        "width":a.width,"height":a.height,"image_format":a.image_format,
                        "client_request_wall":time.time()},timeout=a.capture_timeout)
                    if feedback:response['flight_feedback']=status
                    source=root/response['image'].replace('\\','/')
                    decode_started=time.perf_counter();image=cv2.imread(str(source))
                    decode_ms=(time.perf_counter()-decode_started)*1000.
                    if image is None:raise RuntimeError("Cannot decode UE screenshot")
                    h,w=image.shape[:2]
                    if (w,h)!=(response['width'],response['height']):raise RuntimeError("Capture dimension contract mismatch")
                    path=None;copy_ms=0.
                    if not a.stream_only:
                        dest=a.out/name;dest.mkdir(exist_ok=True)
                        path=dest/f'{count:06d}.{a.image_format}'
                        copy_started=time.perf_counter();shutil.copy2(source,path)
                        copy_ms=(time.perf_counter()-copy_started)*1000.
                    try:source.unlink(missing_ok=True)
                    except PermissionError:pass
                    publish_started=time.perf_counter()
                    if a.ros:
                        stamp=rospy.Time.from_sec(response['capture_wall_start'])
                        msg=Image();msg.header.stamp=stamp;msg.header.seq=count;msg.header.frame_id=f'{a.namespace}_{name}_optical'
                        transform=TransformStamped()
                        transform.header.stamp=stamp;transform.header.frame_id=a.camera_world_frame
                        transform.child_frame_id=msg.header.frame_id
                        position=response['camera_position_ned_m']
                        transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=position
                        matrix=np.eye(4)
                        matrix[:3,:3]=unreal_camera_optical_rotation(response['measured_camera_rotation_deg'])
                        q=quaternion_from_matrix(matrix)
                        (transform.transform.rotation.x,transform.transform.rotation.y,
                         transform.transform.rotation.z,transform.transform.rotation.w)=q
                        camera_tf.sendTransform(transform)
                        msg.height=h;msg.width=w;msg.encoding='bgr8';msg.step=w*3;msg.data=image.tobytes()
                        pubs[name][0].publish(msg)
                        info=CameraInfo();info.header=msg.header;info.width=w;info.height=h
                        f=w/(2*math.tan(math.radians(response['horizontal_fov_deg'])/2))
                        info.K=[f,0.,w/2,0.,f,h/2,0.,0.,1.];info.R=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
                        info.P=[f,0.,w/2,0.,0.,f,h/2,0.,0.,0.,1.,0.];info.D=[0.]*5;info.distortion_model='plumb_bob'
                        pubs[name][1].publish(info)
                    publish_ms=(time.perf_counter()-publish_started)*1000.
                    latency=time.time()-response['capture_wall_start'];latencies.append(latency)
                    frame['cameras'][name]=dict(response,
                        image=(str(path.relative_to(a.out)) if path else None),
                        bridge_decode_ms=decode_ms,bridge_duplicate_copy_ms=copy_ms,
                        bridge_publish_ms=publish_ms,delivery_age_sec=latency,
                        capture_to_ros_publish_sec=latency if a.ros else None)
                    for stage in ("ue_request_poll_ms","capture_scene_call_ms","export_call_ms",
                                  "export_complete_wait_ms","bridge_exchange_ms"):
                        record_stage(stage,response.get(stage))
                    for stage,value in (("bridge_decode_ms",decode_ms),
                                        ("bridge_duplicate_copy_ms",copy_ms),
                                        ("bridge_publish_ms",publish_ms),
                                        ("capture_to_ros_publish_ms",latency*1000. if a.ros else None)):
                        record_stage(stage,value)
                record.write(json.dumps(frame)+'\n');record.flush();count+=1
                capture_ticks.append(tick)
                if a.retain_frames>0 and count>a.retain_frames:
                    old=count-a.retain_frames-1
                    if not a.stream_only:
                        for camera,_,_ in cameras:
                            try:(a.out/camera/f'{old:06d}.{a.image_format}').unlink(missing_ok=True)
                            except PermissionError:pass
                    for old_request in range(old*len(cameras)+1,(old+1)*len(cameras)+1):
                        try:(root/session/f'capture_{old_request:06d}.{a.image_format}').unlink(missing_ok=True)
                        except PermissionError:pass
                if status.get('complete'):break
                if count%10==0:print('Captured steps:',count,'images:',count*len(cameras),
                                     'latest age:',round(latencies[-1],3),flush=True)
                time.sleep(max(0,1/a.fps-(time.monotonic()-tick)))
        finally:
            try:exchange(root,{"id":request_id+1,"session":session,"op":"reset"},timeout=5)
            except Exception as error:print('Reset:',error)
    def percentiles(values):
        ordered=sorted(values)
        if not ordered:return {"p50_ms":None,"p95_ms":None,"mean_ms":None}
        return {"p50_ms":statistics.median(ordered),
                "p95_ms":ordered[min(len(ordered)-1,int(.95*(len(ordered)-1)))],
                "mean_ms":statistics.fmean(ordered)}
    intervals=[b-a for a,b in zip(capture_ticks,capture_ticks[1:])]
    write_json(a.out/'capture_summary.json',{"paired_frames":count,
        "paired_frames_semantics":"legacy capture-step count; cameras/images specify the actual camera count",
        "capture_steps":count,"cameras":[camera[0] for camera in cameras],"images":count*len(cameras),
        "effective_fps":(count-1)/(capture_ticks[-1]-capture_ticks[0]) if len(capture_ticks)>1 else None,
        "frame_interval_p50_sec":statistics.median(intervals) if intervals else None,
        "mean_delivery_age_sec":sum(latencies)/len(latencies) if latencies else None,
        "maximum_delivery_age_sec":max(latencies) if latencies else None,
        "stream_only":a.stream_only,
        "stage_latency_ms":{name:percentiles(values) for name,values in stage_samples.items()},
        "not_an_accuracy_or_flight_dynamics_acceptance":True})
    print('Saved',count,'capture steps and',count*len(cameras),'images to',a.out)


if __name__=='__main__':main()
