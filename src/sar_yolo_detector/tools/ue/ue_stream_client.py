"""Native UE camera stream -> newest-only ROS frames and independent controls."""
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import threading
import time
import uuid
from collections import deque
from sar_yolo_detector.publication_lifecycle import PublicationGate
import cv2
import numpy as np
from common import ROOT,workload_lock,write_json
from geometry import route_pose,unreal_camera_optical_rotation
from gimbal_selection import SelectionSightline
from virtual_zoom import FovProfile
from benchmark_motion import entity_position,BenchmarkClock
from stream_protocol import SocketWire,LatestValue,ClockAlignment,StreamWatchdog,encode_packet,receive_packet


class RelayWire:
    def __init__(self,project,ready):
        native=ready['stream']['python_executable']
        executable=subprocess.check_output(['wslpath','-u',native],text=True).strip()
        relay=subprocess.check_output(['wslpath','-w',str(project/'Scripts/AirGround/tcp_relay.py')],text=True).strip()
        self.process=subprocess.Popen([executable,relay,'--port',str(ready['stream']['port'])],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        self.lock=threading.Lock()
    def send(self,value):
        with self.lock:self.process.stdin.write(encode_packet(value));self.process.stdin.flush()
    def receive(self):return receive_packet(self.process.stdout.read)
    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait(timeout=3)


def open_wire(a,ready):
    if a.stream_connection in ('auto','direct'):
        try:return SocketWire(a.stream_host or ready['stream']['host'],ready['stream']['port']),'direct_tcp'
        except OSError:
            if a.stream_connection=='direct':raise
    if not shutil.which('wslpath'):raise RuntimeError('Windows relay requires WSL; specify a reachable stream host')
    return RelayWire(a.project,ready),'windows_localhost_relay'


def alignment(wire,session,token):
    wire.send(dict(kind='hello',session=session,token=token))
    hello,_=wire.receive()
    if hello.get('kind')!='hello' or hello.get('session')!=session:raise RuntimeError('Camera stream authentication failed')
    clock=ClockAlignment()
    for sequence in range(7):
        sent=time.time();wire.send(dict(kind='ping',session=session,ping_sequence=sequence,client_send_wall=sent))
        pong,_=wire.receive();received=time.time()
        if pong.get('kind')!='pong' or pong.get('ping_sequence')!=sequence:raise RuntimeError('Clock response mismatch')
        clock.add(sent,pong['ue_receive_wall'],pong['ue_send_wall'],received)
    result=clock.result()
    result['utc_resolution_floor_sec']=.001
    result['uncertainty_sec']+=.001
    if result['uncertainty_sec']>.05:raise RuntimeError('Camera stream clock uncertainty exceeds 50 ms')
    return result


def decode_frame(meta,data):
    if meta.get('sensor_modality','rgb')!='rgb':raise ValueError('Native RGB channel cannot declare infrared capability')
    w,h=int(meta['width']),int(meta['height']);codec=meta['codec']
    if not 320<=w<=1920 or not 240<=h<=1080:raise ValueError('Invalid native image dimensions')
    if codec == 'bgr8':
        if len(data) != w*h*3:
            raise ValueError('Raw native image size mismatch')
        image = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
    elif codec in ('bgra8','rgba8'):
        if len(data)!=w*h*4:raise ValueError('Raw native image size mismatch')
        image=cv2.cvtColor(np.frombuffer(data,dtype=np.uint8).reshape(h,w,4),
                          cv2.COLOR_BGRA2BGR if codec=='bgra8' else cv2.COLOR_RGBA2BGR)
    elif codec=='jpeg':
        image=cv2.imdecode(np.frombuffer(data,dtype=np.uint8),cv2.IMREAD_COLOR)
        if image is None or image.shape[:2]!=(h,w):raise ValueError('Native JPEG decode failed')
    else:raise ValueError('Unsupported native image codec')
    return image


def jpeg_dimensions(data):
    """Read encoded geometry without decoding pixels in the transport bridge."""
    import io
    from PIL import Image as EncodedImage
    if data[:2]!=b'\xff\xd8' or data[-2:]!=b'\xff\xd9':raise ValueError('Incomplete JPEG frame')
    with EncodedImage.open(io.BytesIO(data)) as image:
        if image.format!='JPEG':raise ValueError('Native frame is not JPEG')
        return image.size


def bound_ros_send_buffers(publisher,maximum_bytes,configured):
    # ROS/package imports remain optional for the standalone native client.
    from sar_yolo_detector.ros_transport import bound_ros_send_buffers as configure
    return configure(publisher,maximum_bytes,configured)


def camera_messages(meta,image,frame_id,world_frame):
    """One capture owns Image/CameraInfo/TF; never alias mutable TF headers."""
    import copy
    import rospy
    from sensor_msgs.msg import Image,CameraInfo
    from geometry_msgs.msg import TransformStamped
    from tf.transformations import quaternion_from_matrix
    h,w=image.shape[:2] if image is not None else (int(meta['height']),int(meta['width']))
    fov=float(meta['horizontal_fov_deg'])
    if not math.isfinite(fov) or not 1.<fov<179.:raise ValueError('Invalid camera field of view')
    position=meta['camera_position_ned_m']
    if len(position)!=3 or not np.isfinite(position).all():raise ValueError('Invalid camera position')
    msg=Image();msg.header.seq=int(meta['frame_seq'])%2**32
    msg.header.stamp=rospy.Time.from_sec(meta['capture_wall_start']);msg.header.frame_id=frame_id
    msg.height=h;msg.width=w;msg.encoding='bgr8';msg.step=w*3
    msg.data=image.tobytes() if image is not None else b''
    transform=TransformStamped();transform.header=copy.copy(msg.header)
    transform.header.frame_id=world_frame;transform.child_frame_id=frame_id
    transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=position
    matrix=np.eye(4);matrix[:3,:3]=unreal_camera_optical_rotation(meta['measured_camera_rotation_deg'])
    q=quaternion_from_matrix(matrix)
    transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w=q
    info=CameraInfo();info.header=copy.copy(msg.header);info.height=h;info.width=w
    f=w/(2*math.tan(math.radians(fov)/2))
    info.K=[f,0.,w/2,0.,f,h/2,0.,0.,1.];info.P=[f,0.,w/2,0.,0.,f,h/2,0.,0.,0.,1.,0.]
    info.R=[1.,0.,0.,0.,1.,0.,0.,0.,1.];info.D=[0.]*5;info.distortion_model='plumb_bob'
    assert msg.header.frame_id==frame_id and info.header.frame_id==frame_id
    return msg,info,transform


def run_stream(a,spec,mission,feedback,pubs,camera_tf,mission_pub,gimbal_config,ready):
    cv2.setNumThreads(1)
    fov_profile=FovProfile(a.fov_profile)
    benchmark=spec.get('benchmark',{})
    benchmark_clock=BenchmarkClock(benchmark.get('manual_start',False))
    if not ready.get('stream',{}).get('available'):
        raise RuntimeError('SAR native stream plugin is unavailable; install/build it and restart UE')
    session=uuid.uuid4().hex;a.out.mkdir(parents=True)
    wire,connection=open_wire(a,ready)
    try:clock=alignment(wire,session,ready['stream']['token'])
    except Exception:
        wire.close();raise
    offset=clock['ue_minus_host_sec']
    print('UE stream:',connection,'clock:',clock,flush=True)
    slot=LatestValue();stop=threading.Event();error=[];feedback_cache=[None]
    publication_gate=PublicationGate()
    session_rejections=deque(maxlen=64)
    lifecycle_events=[];error_events=[]
    received=[0];wrong_session=[0];feedback_count=[0];last_arrival=[time.monotonic()]
    lens_override=[None];lens_counts=dict(accepted=0,rejected=0)
    control_sequence=[0];control_lock=threading.RLock();control_samples=deque(maxlen=20000)
    applications=deque(maxlen=20000);last_applied=[-1];sightline=SelectionSightline(gimbal_config.get('measurement_timeout_sec',1.2))
    frame_times=deque(maxlen=20000);ages=deque(maxlen=20000);readback=deque(maxlen=20000);wire_times=deque(maxlen=20000);decode_times=deque(maxlen=20000);records=deque(maxlen=20000);stale=0;published=0;decoded_frames=0;passthrough_frames=0;maximum_raw_subscribers=0
    started=time.monotonic();source_started=started;watchdog=StreamWatchdog(started)
    def fail(value):
        if not stop.is_set() and not publication_gate.closed:
            error_events.append(dict(wall=time.time(),message=str(value)))
            error.append(str(value));stop.set();slot.close()
    def reader():
        try:
            while not stop.is_set() and not publication_gate.closed:
                meta,data=wire.receive();arrival=time.time();last_arrival[0]=time.monotonic()
                if meta.get('session')!=session:
                    wrong_session[0]+=1
                    session_rejections.append(dict(wall=arrival,kind=meta.get('kind'),
                        phase='startup' if received[0]==0 else 'running',sequence=meta.get('frame_seq')))
                    continue
                kind=meta.get('kind');watchdog.received(kind,time.monotonic())
                if kind=='frame':
                    received[0]+=1
                    value=dict(meta);value['ue_capture_wall_start']=float(meta['capture_wall_start'])
                    value['capture_wall_start']=float(meta['capture_wall_start'])-offset
                    value['arrival_wall']=arrival
                    slot.put(int(meta['frame_seq']),(value,data))
                elif kind=='gimbal':
                    value=dict(meta);value['ue_wall_stamp']=value['wall_stamp'];value['wall_stamp']-=offset
                    value['feedback_age_sec']=arrival-value['wall_stamp'];value['feedback_fresh']=0<=value['feedback_age_sec']<=.5
                    value['selection_counts']=dict(sightline.counts);value['clock_alignment']=clock
                    feedback_cache[0]=value;feedback_count[0]+=1
                    seq=value.get('applied_sequence',-1)
                    if seq>last_applied[0] and value.get('command_capture_wall') is not None:
                        last_applied[0]=seq
                        applications.append(dict(sequence=seq,
                            capture_to_actuator_ms=(value['applied_wall']-value['command_capture_wall'])*1000.,
                            control_to_actuator_ms=(value['applied_wall']-value['command_created_wall'])*1000. if value.get('command_created_wall') else None,
                            ue_command_delay_ms=(value['applied_wall']-value['command_received_wall'])*1000. if value.get('command_received_wall') else None))
                    if a.ros:publication_gate.call(gimbal_pub.publish,String(data=json.dumps(value)))
                elif kind=='error':raise RuntimeError(meta.get('error','UE stream error'))
        except Exception as e:fail(e)

    def pose_at(elapsed):
        if feedback:position,yaw,status=feedback.wait_sample()
        elif mission:position,yaw,status=mission.sample(elapsed)
        else:
            position,yaw=route_pose(spec['observer']['route'],elapsed*spec['observer']['speed_mps']);status=dict(stage='legacy_route',complete=False)
        objects={e['id']:entity_position(e,benchmark_clock.sample(elapsed)) for e in spec['entities']}
        if spec.get('observer_visual_actor'):objects[spec['observer_visual_actor']]=position
        pose=dict(horizontal_fov_target_deg=fov_profile.target(elapsed) if lens_override[0] is None else lens_override[0],objects=objects,camera_position_ned_m=[position[0]+1.2*math.cos(yaw),position[1]+1.2*math.sin(yaw),position[2]+.2],
            camera_pitch_yaw_deg=[-70. if a.camera=='down' else -10.,math.degrees(yaw)],body_yaw_deg=math.degrees(yaw),
            object_yaw_deg={spec['observer_visual_actor']:math.degrees(yaw)} if spec.get('observer_visual_actor') else {},
            object_rotation_deg={spec['observer_visual_actor']:status['rotation_deg']} if feedback and spec.get('observer_visual_actor') else {})
        status.update(observer_ned_m=position,yaw_deg=math.degrees(yaw))
        return pose,status

    def producer():
        try:
            while not stop.is_set() and not publication_gate.closed:
                tick=time.monotonic();pose,status=pose_at(tick-source_started)
                wire.send(dict(kind='pose',session=session,pose=pose))
                if a.ros:publication_gate.call(mission_pub.publish,String(data=json.dumps(status)))
                if tick-source_started>=a.seconds or status.get('complete'):break
                stop.wait(max(0.,1./30-(time.monotonic()-tick)))
            stop.set();slot.close()
        except Exception as e:fail(e)

    subscriber=None;lens_subscriber=None
    if a.ros:
        import rospy
        def shutdown_stream():
            lifecycle_events.append(dict(wall=time.time(),event='ros_shutdown_requested'))
            # rospy invokes this hook inside its shutdown/signal path. Do not
            # wait for publishing workers here; the outer finally drains them.
            publication_gate.request_stop()
        publication_gate.shutdown_requested=lambda:rospy.is_shutdown() or rospy.core.is_shutdown_requested()
        rospy.on_shutdown(shutdown_stream)
        from std_msgs.msg import String,Float64
        from sensor_msgs.msg import Image,CameraInfo,CompressedImage
        import copy
        compressed_pubs={name:rospy.Publisher('/'+a.namespace+('/fixed_camera/image_raw/compressed' if name=='down' else '/gimbal_camera/image_raw/compressed'),CompressedImage,queue_size=1) for name in pubs}
        configured_send_sockets={name:set() for name in pubs}
        configured_raw_send_sockets={name:set() for name in pubs}
        from geometry_msgs.msg import TransformStamped
        from tf.transformations import quaternion_from_matrix
        from sar_yolo_detector.msg import SmartTrackerState
        if benchmark.get('manual_start'):
            def benchmark_command(message):
                try:benchmark_clock.command(message.data)
                except ValueError as error:rospy.logwarn(str(error))
            benchmark_sub=rospy.Subscriber('/'+a.namespace+'/benchmark/command',String,benchmark_command,queue_size=1)
        source_pub=rospy.Publisher('/'+a.namespace+'/camera/source',String,queue_size=1,latch=True)
        publication_gate.call(source_pub.publish,String(data=json.dumps(dict(schema_version=1,modality='rgb',
            producer='UE.SceneCapture2D.FinalColorLDR',thermal_sensor_model=False,is_replay=False,
            encoding='bgr8',session=session,frame_id=a.namespace+'_'+a.camera+'_optical',
            image_topic='/'+a.namespace+('/fixed_camera/image_raw' if a.camera=='down' else '/gimbal_camera/image_raw'),
            camera_info_topic='/'+a.namespace+('/fixed_camera/camera_info' if a.camera=='down' else '/gimbal_camera/camera_info'),transport='SRF1'))))
        if a.allow_fov_control:
            def lens_command(message):
                value=float(message.data)
                if not math.isfinite(value) or not fov_profile.config['minimum_deg']<=value<=fov_profile.config['maximum_deg']:
                    lens_counts['rejected']+=1;rospy.logwarn_throttle(2.,'Rejected invalid FOV target');return
                lens_override[0]=value;lens_counts['accepted']+=1
            lens_subscriber=rospy.Subscriber('/'+a.namespace+'/camera/fov_target_deg',Float64,lens_command,queue_size=1)
        geometry_pub=rospy.Publisher('/'+a.namespace+'/target_geometry',String,queue_size=1)
        gimbal_pub=rospy.Publisher('/'+a.namespace+'/gimbal/state',String,queue_size=1)
        def selected(message):
            if stop.is_set():return
            command=sightline.observe(dict(stamp=message.header.stamp.to_sec(),frame_id=message.header.frame_id,
                class_id=message.selected_class_id,class_name=message.selected_class_name,
                track_id=message.selected_track_id,tracker_type=message.tracker_type,has_selection=message.has_selection,
                measurement_current=message.measurement_current,control_ready=message.control_measurement_ready,
                association_method=message.association_method,
                prediction_only=message.prediction_only,stale=message.data_is_stale,tentative=message.tentative,
                confidence=message.confidence,center=[message.selected_bbox.center.x,message.selected_bbox.center.y]),time.time())
            if command is None:return
            evidence=command.pop('geometry',None)
            if evidence is not None and not publication_gate.call(geometry_pub.publish,String(data=json.dumps(evidence))):return
            now=time.time()
            with control_lock:
                control_sequence[0]+=1
                command.update(kind='gimbal',session=session,sequence=control_sequence[0],sent_wall=now+offset)
                if not command.get('cancel'):
                    source=command['measurement_stamp'];command['measurement_stamp']+=offset
                    control_samples.append(dict(sequence=control_sequence[0],capture_stamp=source,control_output_wall=now,
                        capture_to_control_ms=(now-source)*1000.,frame_processing_ms=float(message.frame_processing_ms)))
                try:wire.send(command)
                except OSError as e:fail(e)
        if a.virtual_gimbal:
            subscriber=rospy.Subscriber('/'+a.namespace+'/sar_yolo_detector/'+a.instance_name+'/state',SmartTrackerState,selected,queue_size=1)
    pose,status=pose_at(0.)
    config=dict(kind='configure',session=session,fps=a.fps,width=a.width,height=a.height,
        benchmark_truth=bool(benchmark.get('evaluation_truth',False)),camera_key=a.camera,codec=a.stream_codec,quality=getattr(a,'stream_quality',90),virtual_gimbal=a.virtual_gimbal,gimbal_config=gimbal_config,zoom_config=fov_profile.config,pose=pose)
    timer=None
    workers=[]
    def stats(values):
        if not values:return dict(samples=0,p50_ms=None,p95_ms=None)
        return dict(samples=len(values),p50_ms=float(np.percentile(values,50)),p95_ms=float(np.percentile(values,95)))
    with workload_lock('native_capture_controller',lock_path=ROOT/'runtime/ue_capture_controller.lock'),workload_lock('native_ue_capture',shared=True):
        write_json(a.out/'run.json',dict(session=session,transport='SRF1',connection=connection,clock_alignment=clock,
            requested_fps=a.fps,stream_quality=a.stream_quality,scene=spec,codec=a.stream_codec,fov_profile=a.fov_profile,fov_schedule=fov_profile.schedule,fov_control_enabled=a.allow_fov_control))
        try:
            wire.send(config)
            workers=[threading.Thread(target=reader,daemon=True),threading.Thread(target=producer,daemon=True)]
            for worker in workers:worker.start()
            while not stop.is_set() and not publication_gate.closed and (not a.max_frames or published<a.max_frames):
                if a.ros and rospy.is_shutdown():break
                expired=watchdog.reason(time.monotonic())
                if expired:raise RuntimeError(expired)
                item=slot.get(.1)
                if item is None:continue
                meta,data=item;age=time.time()-meta['capture_wall_start']
                if not -.05<=age<=a.max_delivery_age:
                    stale+=1;continue
                name=a.camera
                raw_wanted=bool(a.ros and pubs[name][0].get_num_connections()>0)
                if a.ros:maximum_raw_subscribers=max(maximum_raw_subscribers,pubs[name][0].get_num_connections())
                needs_pixels=(not a.ros or raw_wanted or meta['codec']!='jpeg' or (a.save_first_frame and published==0))
                before=time.perf_counter();image=None
                if needs_pixels:
                    image=decode_frame(meta,data);decoded_frames+=1
                else:
                    if jpeg_dimensions(data)!=(int(meta['width']),int(meta['height'])):raise ValueError('JPEG dimensions disagree with capture metadata')
                    passthrough_frames+=1
                decode_ms=(time.perf_counter()-before)*1000.
                age=time.time()-meta['capture_wall_start']
                if age>a.max_delivery_age:stale+=1;continue
                h,w=int(meta['height']),int(meta['width']);frame_id=a.namespace+'_'+name+'_optical'
                sightline.capture(meta,frame_id)
                if a.ros:
                    msg,info,transform=camera_messages(meta,image,frame_id,a.camera_world_frame)
                    if not publication_gate.call(camera_tf.sendTransform,transform):break
                    if not publication_gate.call(pubs[name][1].publish,info):break
                    if raw_wanted:
                        bound_ros_send_buffers(pubs[name][0], getattr(a, "ros_send_buffer_bytes", 0),
                                               configured_raw_send_sockets[name])
                        if not publication_gate.call(pubs[name][0].publish,msg):break
                    if compressed_pubs[name].get_num_connections()>0:
                        if meta.get('codec')=='jpeg':encoded=data
                        else:
                            ok,buffer=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,getattr(a,'stream_quality',90)])
                            if not ok:raise ValueError('Compressed ROS image encoding failed')
                            encoded=buffer.tobytes()
                        bound_ros_send_buffers(compressed_pubs[name],getattr(a,"ros_send_buffer_bytes",0),configured_send_sockets[name])
                        publication_gate.call(compressed_pubs[name].publish,CompressedImage(header=copy.copy(msg.header),format='jpeg',data=encoded))
                delivered=time.time();frame_times.append(delivered);published+=1
                ages.append((delivered-meta['capture_wall_start'])*1000.)
                readback.append((meta['readback_ready_wall']-meta['ue_capture_wall_start'])*1000.)
                wire_times.append((meta['arrival_wall']-(meta['ue_send_wall']-offset))*1000.)
                decode_times.append(decode_ms)
                encoded=meta.get('encode_complete_wall')
                arrived=meta['arrival_wall']
                records.append(dict(sequence=meta['frame_seq'],capture_wall=meta['capture_wall_start'],publish_wall=delivered,
                    delivery_age_ms=ages[-1],readback_ms=readback[-1],wire_ms=wire_times[-1],decode_ms=decode_ms,
                    readback_to_encoded_ms=(encoded-meta['readback_ready_wall'])*1000. if encoded is not None else None,
                    encoded_to_send_ms=(meta['ue_send_wall']-encoded)*1000. if encoded is not None else None,
                    arrival_to_delivery_ms=(delivered-arrived)*1000.,payload_bytes=len(data),
                    fov=meta['horizontal_fov_deg'],fov_target=meta.get('horizontal_fov_target_deg'),
                    zoom_rate_deg_s=meta.get('zoom_rate_deg_s'),camera_rotation=meta['measured_camera_rotation_deg']))
                if not a.stream_only:
                    folder=a.out/name;folder.mkdir(exist_ok=True);cv2.imwrite(str(folder/f'{published:06d}.jpg'),image)
                if a.save_first_frame and published==1:cv2.imwrite(str(a.out/'first_frame.png'),image)
                if published%100==0:print('Stream frames:',published,'age_ms:',round(ages[-1],1),'latest drops:',slot.replaced,flush=True)
            if error:raise RuntimeError(error[-1])
        except Exception as e:
            if not error or error[-1]!=str(e):error.append(str(e))
            raise
        finally:
            lifecycle_events.append(dict(wall=time.time(),event='stream_cleanup_started'))
            publication_gate.request_stop();stop.set();slot.close()
            with control_lock:
                control_sequence[0]+=1
                try:
                    wire.send(dict(kind='gimbal',session=session,sequence=control_sequence[0],cancel=True,sent_wall=time.time()+offset))
                    wire.send(dict(kind='stop',session=session))
                except OSError:pass
            stop.set();publication_gate.close();slot.close();wire.close()
            for worker in workers:worker.join(timeout=3.)
            if subscriber is not None:subscriber.unregister()
            if lens_subscriber is not None:lens_subscriber.unregister()
            summary=dict(transport='SRF1',connection=connection,clock_alignment=clock,requested_fps=a.fps,
                fov_profile=a.fov_profile,fov_control_enabled=a.allow_fov_control,fov_commands=lens_counts,fov_min_deg=min((v['fov'] for v in records),default=None),
                fov_max_deg=max((v['fov'] for v in records),default=None),
                ros_enabled=bool(a.ros),received_frames=received[0],delivered_frames=published,published_frames=published if a.ros else 0,
                effective_fps=(len(frame_times)-1)/(frame_times[-1]-frame_times[0]) if len(frame_times)>1 else 0.,
                dropped_latest=slot.replaced,dropped_out_of_order=slot.out_of_order,dropped_stale=stale,
                wrong_session=wrong_session[0],session_rejections=list(session_rejections),lifecycle_events=lifecycle_events,error_events=error_events,feedback_messages=feedback_count[0],errors=error,
                decoded_frames=decoded_frames,jpeg_passthrough_frames=passthrough_frames,maximum_raw_subscribers=maximum_raw_subscribers,
                ros_send_buffer_bytes=getattr(a,"ros_send_buffer_bytes",0),
                capture_to_delivery=stats(ages),capture_to_ros=stats(ages) if a.ros else None,gpu_readback=stats(readback),wire=stats(wire_times),decode=stats(decode_times),
                stream_quality=a.stream_quality,
                readback_to_encoded=stats([v['readback_to_encoded_ms'] for v in records if v['readback_to_encoded_ms'] is not None]),
                encoded_to_send=stats([v['encoded_to_send_ms'] for v in records if v['encoded_to_send_ms'] is not None]),
                arrival_to_delivery=stats([v['arrival_to_delivery_ms'] for v in records]),
                control_to_actuator=stats([v['control_to_actuator_ms'] for v in applications if v['control_to_actuator_ms'] is not None]),
                capture_to_control=stats([v['capture_to_control_ms'] for v in control_samples]),
                capture_to_actuator=stats([v['capture_to_actuator_ms'] for v in applications]),
                selection_counts=dict(sightline.counts))
            write_json(a.out/'capture_summary.json',summary)
            write_json(a.out/'frame_timing.json',list(records))
            write_json(a.out/'control_timing.json',list(control_samples));write_json(a.out/'actuator_timing.json',list(applications))
            if feedback_cache[0] is not None:write_json(a.out/'last_gimbal_feedback.json',feedback_cache[0])
            print(json.dumps(summary,indent=2),flush=True)
