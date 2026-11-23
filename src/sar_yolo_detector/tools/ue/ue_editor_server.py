"""UE 5.5 editor capture server; native stream plugin or legacy file export.

Loaded by project Content/Python/init_unreal.py in the normal editor.
SceneCapture2D writes camera render targets. JSON commands affect AG_ actors.
Renders kinematic poses or actual external PX4/Gazebo feedback.
Flight dynamics belong to the external simulator, not this renderer.
"""
import json
import math
import os
from pathlib import Path
import time
import traceback
import builtins
import sys
import unreal
sys.path.insert(0,str(Path(__file__).parent))
import importlib,frame_protocol,virtual_gimbal,ue_stream_host
importlib.reload(frame_protocol)
importlib.reload(virtual_gimbal)
from virtual_gimbal import VirtualGimbal
importlib.reload(ue_stream_host)
from ue_stream_host import StreamHost
from frame_protocol import image_complete,claim_request

ROOT=Path(unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_saved_dir()))/"AirGroundExchange"
ROOT.mkdir(parents=True,exist_ok=True)
ACTORS=unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
state=getattr(builtins,'_airground_capture_state',None) or {"pending":None,"last_id":None,"camera":None,"heartbeat":0,"initial":{},"target":None}
builtins._airground_capture_state=state
stream=StreamHost(state)
if 'startup_ignored_key' not in state:
    # A restart must not replay the last persisted command from a previous run.
    try:
        previous=json.loads((ROOT/'request.json').read_text())
        state['startup_ignored_key']=(previous.get('session'),previous.get('id'))
    except (OSError,ValueError):state['startup_ignored_key']=None
editor=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
world=editor.get_game_world() or editor.get_editor_world()
unreal.SystemLibrary.execute_console_command(world,"t.MaxFPS 30")
unreal.SystemLibrary.execute_console_command(world,"t.IdleWhenNotForeground 0")
unreal.SystemLibrary.execute_console_command(world,'Slate.bAllowThrottling 0')
try:
    performance_class=unreal.load_class(None,'/Script/UnrealEd.EditorPerformanceSettings')
    performance=unreal.get_default_object(performance_class)
    performance.set_editor_property('bThrottleCPUWhenNotForeground',False)
    unreal.log('AirGround disabled editor background CPU throttling')
except Exception as error:unreal.log_warning('AirGround performance settings: '+str(error))


def write(name,data):
    temp=ROOT/(name+".tmp")
    for attempt in range(100):
        try:
            temp.write_text(json.dumps(data));temp.replace(ROOT/name);return
        except PermissionError:
            if attempt==99:raise
            time.sleep(.005)


def fail(request,error,retryable=False):
    write("response.json",{"id":request.get("id"),"session":request.get("session"),"ok":False,"error":str(error),"retryable":retryable})
    unreal.log_error("AirGround capture: "+str(error))


def update_gimbal(now):
    gimbal=state.get('gimbal')
    if gimbal is None:return
    # Separate command/feedback channel keeps actuator updates independent of
    # screenshot export and the ROS detector's frame rate.
    try:
        command=(state.get('network_gimbal_command',{}) if state.get('stream_active') else
                 json.loads((ROOT/'gimbal_command.json').read_text()))
        if command.get('session')==state.get('gimbal_session') and now-command['sent_wall']<2.:
            sequence=int(command['sequence'])
            if command.get('cancel'):
                accepted=gimbal.command(sequence,time.monotonic())
            else:
                age=max(0.,now-float(command['measurement_stamp']))
                accepted=gimbal.command(sequence,time.monotonic(),float(command['yaw_world_deg']),
                               float(command['pitch_world_deg']),age)
            if accepted:
                history=state.setdefault('gimbal_command_history',{})
                history[sequence]=dict(command)
                while len(history)>256:history.pop(next(iter(history)))
    except (OSError,ValueError,KeyError,TypeError):pass
    actuator_stamp=time.monotonic()
    measured=gimbal.step(actuator_stamp,state.get('gimbal_body_yaw',0.))
    if measured['applied_sequence']!=state.get('gimbal_last_applied_sequence',-1):
        state['gimbal_last_applied_sequence']=measured['applied_sequence']
        history=state.get('gimbal_command_history',{}).get(measured['applied_sequence'],{})
        state['gimbal_application_event']=dict(applied_wall=time.time(),
            command_received_wall=history.get('ue_receive_wall'),
            command_capture_wall=history.get('measurement_stamp'),
            command_created_wall=history.get('sent_wall'))
    measured.update(state.get('gimbal_application_event',{}))
    camera=state.get('cameras',{}).get(state.get('gimbal_camera_key'))
    if camera:
        camera.set_actor_rotation(unreal.Rotator(pitch=measured['pitch_deg'],
            yaw=measured['yaw_world_deg'],roll=0.),False)
        actual=camera.get_actor_rotation()
        measured['horizontal_fov_deg']=float(camera.get_component_by_class(unreal.SceneCaptureComponent2D).get_editor_property('fov_angle'))
        measured['measured_rotation_deg']=dict(pitch=actual.pitch,yaw=actual.yaw,roll=actual.roll)
    measured.update(session=state.get('gimbal_session'),wall_stamp=time.time(),monotonic_stamp=actuator_stamp,
                    camera=state.get('gimbal_camera_key'))
    state['gimbal_feedback']=measured
    if now-state.get('gimbal_feedback_wall',0.)>=.04:
        if state.get('stream_active'):stream.status(measured)
        else:write('gimbal_feedback.json',measured)
        state['gimbal_feedback_wall']=now


def tick_impl(delta):
    global world
    editor=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
    game_world=editor.get_game_world()
    world=game_world or editor.get_editor_world()
    world_key=world.get_path_name()
    if state.get('world_key')!=world_key:
        if state.get('pending'):fail(state['pending']['result'],'World changed during capture',True)
        state.pop('gimbal',None)
        state.update(world_key=world_key,cameras={},targets={},initial={},camera=None,target=None,pending=None,last_id=None)
    now=time.time()
    stream_request=stream.poll()
    capturing=bool(state.get('stream_active'))
    if state.get('editor_capture_budget')!=capturing:
        # SceneCapture continues at its own full-resolution schedule. Suspend
        # redundant viewport renders and blocking editor autosaves only while
        # streaming; restore autosave's exact previous value at stream end.
        unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).editor_set_viewport_realtime(not capturing)
        settings=unreal.get_default_object(unreal.load_class(None,'/Script/UnrealEd.EditorLoadingSavingSettings'))
        if capturing:
            state['capture_saved_autosave']=settings.get_editor_property('bAutoSaveEnable')
            settings.set_editor_property('bAutoSaveEnable',False)
        elif 'capture_saved_autosave' in state:
            settings.set_editor_property('bAutoSaveEnable',state.pop('capture_saved_autosave'))
        state['editor_capture_budget']=capturing
        unreal.log('AirGround capture editor budget active: '+str(capturing))
    if stream.just_stopped:
        for actor,transform in state["initial"].values():
            if unreal.SystemLibrary.is_valid(actor):actor.set_actor_transform(transform,False,False)
        stream.just_stopped=False
    update_gimbal(now)
    if now-state["heartbeat"]>=1:
        write("ready.json",{"pid":os.getpid(),"heartbeat":now,"engine":unreal.SystemLibrary.get_engine_version(),
              "transport":"scene_capture_render_target_png","stream":stream.info(),"project_dir":unreal.Paths.project_dir(),
              "world":world_key,"play_mode":game_world is not None,"tick_delta_sec":delta})
        state["heartbeat"]=now
    pending=state["pending"]
    if pending:
        path=pending["path"]
        if path.is_file() and path.stat().st_size>20:
            complete=image_complete(path)
            if complete:
                pending["result"]["export_complete_wait_ms"] = max(
                    0., (time.perf_counter()-pending["export_submitted_perf"])*1000.)
                write("response.json",dict(pending["result"],ok=True,image=path.relative_to(ROOT).as_posix()))
                state["pending"]=None
        if state["pending"] and now-pending["started"]>180:
            fail(pending["result"],"Screenshot timeout");state["pending"]=None
        return
    request_path=ROOT/"request.json"
    if stream_request is None:
        if not request_path.is_file():return
        request=json.loads(request_path.read_text())
    else:request=stream_request
    request_detected_wall=time.time()
    key=(request.get("session"),request.get("id"))
    if not claim_request(state,request,native=stream_request is not None):return
    try:
        session=request["session"]
        if not session.isalnum() or len(session)>64:raise ValueError("Invalid session")
        if request.get("op")=="reload":
            state["startup_ignored_key"]=key
            write("response.json",{"id":request['id'],"session":session,"ok":True,"reload":True})
            state.update(cameras={},targets={},initial={},camera=None,target=None,pending=None)
            import runpy,sys
            sys.modules.pop('ue_stream_host',None)
            sys.modules.pop('ue_truth_native',None)
            sys.modules.pop('ue_benchmark_truth',None)
            sys.modules.pop('benchmark_visibility',None)
            runpy.run_path(str(Path(__file__)));return
        if request.get('op')=='create_visual_variant':
            if game_world is not None or state.get('stream_active'):raise RuntimeError('Stop capture/Play before creating visual variant')
            import ue_visual_variant,importlib
            importlib.reload(ue_visual_variant)
            result=ue_visual_variant.create_visual_variant()
            state.update(cameras={},targets={},initial={})
            write('response.json',dict(id=request['id'],session=session,ok=True,visual_variant=result));return
        if request.get('op')=='augment_benchmark':
            if game_world is not None or state.get('stream_active'):raise RuntimeError('Stop capture/Play before extending benchmark map')
            import runpy
            runpy.run_path(str(Path(__file__).with_name('ue_augment_benchmark.py')))
            state.update(cameras={},targets={},initial={})
            write('response.json',{'id':request['id'],'session':session,'ok':True,'benchmark_extended':True});return
        if request.get('op')=='rebuild':
            if game_world is not None:raise RuntimeError('Stop Play before rebuilding the generated map')
            import runpy
            runpy.run_path(str(Path(__file__).with_name('build_scene.py')))
            state.update(cameras={},targets={},initial={})
            write('response.json',{'id':request['id'],'session':session,'ok':True,'rebuilt':True});return
        if request.get('op')=='prepare_truth':
            if game_world is not None:raise RuntimeError('Stop Play before creating truth materials')
            import ue_truth_native
            palette=ue_truth_native.prepare()
            write('response.json',{'id':request['id'],'session':session,'ok':True,'palette':palette});return
        if request.get('op')=='import_detailed':
            if game_world is not None:raise RuntimeError('Stop Play before importing detailed assets')
            import runpy
            runpy.run_path(str(Path(__file__).with_name('import_detailed_assets.py')))
            write('response.json',{'id':request['id'],'session':session,'ok':True,'imported':True});return
        if request.get('op')=='play':
            unreal.EditorLevelLibrary.editor_play_simulate()
            write('response.json',{'id':request['id'],'session':session,'ok':True,'play_requested':True});return
        if request.get('op')=='stop_play':
            unreal.EditorLevelLibrary.editor_end_play()
            write('response.json',{'id':request['id'],'session':session,'ok':True,'stop_play_requested':True});return
        if request.get("op")=="reset":
            for actor,transform in state["initial"].values():actor.set_actor_transform(transform,False,False)
            write("response.json",{"session":session,"id":request["id"],"ok":True,"reset":True});return
        if request.get("op")!="capture":raise ValueError("Unsupported operation")
        by_name={str(a.get_actor_label()):a for a in unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor)}
        actual=[]
        if not state.get('daylight_ready'):
            sky=by_name.get('AG_SkyLight')
            if sky:
                light=sky.get_component_by_class(unreal.SkyLightComponent)
                light.set_mobility(unreal.ComponentMobility.MOVABLE)
                light.set_editor_property('source_type',unreal.SkyLightSourceType.SLS_SPECIFIED_CUBEMAP)
                light.set_cubemap(unreal.load_asset('/Engine/MapTemplates/Sky/DaylightAmbientCubemap.DaylightAmbientCubemap'))
                light.set_intensity(1.5)
                light.recapture_sky()
            state['daylight_ready']=True
        for object_id,position in request["objects"].items():
            if not object_id.startswith("AG_") or object_id not in by_name:raise ValueError("Unknown generated actor: "+object_id)
            actor=by_name[object_id]
            if object_id not in state["initial"]:state["initial"][object_id]=(actor,actor.get_actor_transform())
            if len(position)!=3 or not all(math.isfinite(v) for v in position):raise ValueError("Invalid position")
            if object_id in request.get('object_yaw_deg',{}):
                heading=request['object_yaw_deg'][object_id]
                if not math.isfinite(heading):raise ValueError('Invalid yaw')
                actor.set_actor_rotation(unreal.Rotator(pitch=0.,yaw=heading,roll=0.),False)
            if object_id in request.get('object_rotation_deg',{}):
                rotation=request['object_rotation_deg'][object_id]
                if not all(math.isfinite(rotation[k]) for k in ('pitch','yaw','roll')):raise ValueError('Invalid attitude')
                actor.set_actor_rotation(unreal.Rotator(pitch=rotation['pitch'],yaw=rotation['yaw'],roll=rotation['roll']),False)
            old_center,extent=actor.get_actor_bounds(False)
            old_location=actor.get_actor_location()
            offset=old_center-old_location
            actor.set_actor_location(unreal.Vector(position[0]*100-offset.x,position[1]*100-offset.y,
                                                   -position[2]*100-offset.z),False,False)
            measured,extent=actor.get_actor_bounds(False)
            actual.append({"id":object_id,"position_ned_m":[measured.x/100,measured.y/100,-measured.z/100]})
        camera_key=request.get('camera_key') or ('down' if request['camera_pitch_yaw_deg'][0]<-30 else 'front')
        if camera_key not in ('down','front'):raise ValueError('Invalid camera key')
        if not request.get('virtual_gimbal') and state.get('gimbal_camera_key')==camera_key:
            state.pop('gimbal',None)
        camera=state['cameras'].get(camera_key)
        if camera is None or camera!=by_name.get('AG_Capture_'+camera_key):
            state['targets'].pop(camera_key,None)
            camera=by_name.get('AG_Capture_'+camera_key)
            if camera is None and game_world is None:
                camera=ACTORS.spawn_actor_from_class(unreal.SceneCapture2D,unreal.Vector(0,0,3000),transient=True)
            elif camera is None:
                raise RuntimeError('Rebuild the map to include persistent capture actors before Play')
            camera.set_actor_label("AG_Capture_"+camera_key)
            component=camera.get_component_by_class(unreal.SceneCaptureComponent2D)
            component.set_editor_property("fov_angle",90.)
            component.set_editor_property("capture_every_frame",False)
            component.set_editor_property("capture_on_movement",False)
            component.set_editor_property("capture_source",unreal.SceneCaptureSource.SCS_FINAL_COLOR_LDR)
            state['cameras'][camera_key]=camera
        position=request["camera_position_ned_m"];pitch,yaw=request["camera_pitch_yaw_deg"]
        camera.set_actor_location(unreal.Vector(position[0]*100,position[1]*100,-position[2]*100),False,False)
        if request.get('virtual_gimbal'):
            if state.get('gimbal_session')!=session or state.get('gimbal') is None:
                state['gimbal']=VirtualGimbal(request.get('gimbal_config'))
                state['gimbal_command_history']={}
                state['gimbal_application_event']={}
                state['gimbal_last_applied_sequence']=-1
                state['gimbal_session']=session
            state['gimbal_camera_key']=camera_key
            state['gimbal_body_yaw']=float(request['body_yaw_deg'])
            update_gimbal(time.time())
            measured=state['gimbal_feedback']
            pitch,yaw=measured['pitch_deg'],measured['yaw_world_deg']
        camera.set_actor_rotation(unreal.Rotator(pitch=pitch,yaw=yaw,roll=0.),False)
        width,height=int(request.get("width",960)),int(request.get("height",540))
        if not 320<=width<=1920 or not 240<=height<=1080:raise ValueError("Invalid capture size")
        directory=ROOT/session
        if not request.get("native_stream"):directory.mkdir(exist_ok=True)
        image_format=request.get("image_format","png")
        if image_format not in ("png","jpg"):raise ValueError("Unsupported image format")
        path=directory/f"capture_{int(request['id']):06d}.{image_format}"
        if not request.get("native_stream") and path.exists():path.unlink()
        if game_world is None and hasattr(unreal.EditorLevelLibrary,"editor_set_game_view"):
            unreal.EditorLevelLibrary.editor_set_game_view(True)
        component=camera.get_component_by_class(unreal.SceneCaptureComponent2D)
        fov=float(request.get('horizontal_fov_deg',90.))
        if not 45.<=fov<=100.:raise ValueError('FOV outside validated virtual lens limits')
        component.set_editor_property('fov_angle',fov)
        fov=float(component.get_editor_property('fov_angle'))
        if state.setdefault('postprocess_cameras', {}).get(camera_key) != camera:
            settings=component.get_editor_property('post_process_settings')
            settings.set_editor_property('override_auto_exposure_method',True)
            settings.set_editor_property('auto_exposure_method',unreal.AutoExposureMethod.AEM_MANUAL)
            settings.set_editor_property('override_auto_exposure_apply_physical_camera_exposure',True)
            settings.set_editor_property('auto_exposure_apply_physical_camera_exposure',False)
            settings.set_editor_property('override_auto_exposure_bias',True)
            settings.set_editor_property('auto_exposure_bias',1.5)
            settings.set_editor_property('override_motion_blur_amount', True)
            settings.set_editor_property('motion_blur_amount', 0.)
            settings.set_editor_property('override_depth_of_field_fstop', True)
            settings.set_editor_property('depth_of_field_fstop', 32.)
            settings.set_editor_property('override_vignette_intensity',True)
            settings.set_editor_property('vignette_intensity',0.)
            component.set_editor_property('post_process_settings',settings)
            component.set_editor_property('post_process_blend_weight',1.)
            state['postprocess_cameras'][camera_key] = camera
        target=state['targets'].get(camera_key)
        if target is None or target.size_x!=width or target.size_y!=height:
            # Persistent map targets can be shared across PIE generations. Never
            # release their render resources or reuse them after a world restart.
            # Each active camera owns a transient target; UE GC fences its cleanup.
            target=unreal.RenderingLibrary.create_render_target2d(world,width,height,unreal.TextureRenderTargetFormat.RTF_RGBA8)
            component.set_editor_property("texture_target",target)
            state['targets'][camera_key]=target
        capture_wall_start=time.time()
        capture_started=time.perf_counter()
        component.capture_scene()
        render_submitted=time.perf_counter()
        export_started=time.perf_counter()
        if request.get('native_stream'):
            actual_rotation=camera.get_actor_rotation()
            metadata=dict(id=request['id'],session=session,capture_wall_start=capture_wall_start,
                sensor_modality='rgb',sensor_response_model='UE.SceneCapture2D.FinalColorLDR',
                width=width,height=height,camera_position_ned_m=position,horizontal_fov_deg=fov,
                horizontal_fov_target_deg=request.get("horizontal_fov_target_deg",fov),
                zoom_rate_deg_s=stream.zoom.axis.velocity,
                camera_pitch_yaw_deg=[pitch,yaw],
                measured_camera_rotation_deg=dict(pitch=actual_rotation.pitch,yaw=actual_rotation.yaw,roll=actual_rotation.roll),
                gimbal_feedback=dict(state.get('gimbal_feedback',{})),
                capture_scene_call_ms=(render_submitted-capture_started)*1000.)
            if request.get('benchmark_truth'):
                import ue_benchmark_truth
                ue_benchmark_truth.record(ROOT,camera,by_name,request,metadata)
            stream.capture(target,metadata,request)
            return
        if image_format=='jpg':
            options=unreal.ImageWriteOptions()
            options.set_editor_property('format',unreal.DesiredImageFormat.JPG)
            options.set_editor_property('compression_quality',92)
            unreal.ImageWriteBlueprintLibrary.export_to_disk(target,str(path),options)
        else:
            unreal.RenderingLibrary.export_render_target(world,target,str(path.parent),path.name)
        export_finished=time.perf_counter()
        if pitch>-30 and game_world is None:
            unreal.EditorLevelLibrary.set_level_viewport_camera_info(camera.get_actor_location(),camera.get_actor_rotation())
        elif pitch>-30:
            controller=unreal.GameplayStatics.get_player_controller(world,0)
            if controller:controller.set_view_target_with_blend(camera,0.)
        measured_rotation=camera.get_actor_rotation()
        result={"id":request["id"],"session":session,"capture_wall_start":capture_wall_start,"width":width,"height":height,
                "camera_position_ned_m":position,"camera_pitch_yaw_deg":[pitch,yaw],"horizontal_fov_deg":fov,
                "measured_camera_rotation_deg":{"pitch":measured_rotation.pitch,"yaw":measured_rotation.yaw,"roll":measured_rotation.roll},
                "objects":actual,"truth":"3D poses only; no visibility/segmentation ground truth"}
        if request.get('virtual_gimbal'):result['gimbal_feedback']=dict(state['gimbal_feedback'])
        result["capture_method"]="SceneCapture2D/RenderTarget"
        result["image_format"]=image_format
        result["ue_request_poll_ms"]=max(0.,(request_detected_wall-float(request.get("client_request_wall",request_detected_wall)))*1000.)
        result["capture_scene_call_ms"]=(render_submitted-capture_started)*1000.
        result["export_call_ms"]=(export_finished-export_started)*1000.
        if request.get('with_truth'):
            import ue_truth_native
            mask_path=path.with_name(path.stem+'_mask.png')
            result['truth_palette']=ue_truth_native.capture(world,component,target,mask_path)
            result['truth_mask']=mask_path.relative_to(ROOT).as_posix()
            result['truth']='visible instance segmentation extent; no detector predictions used'
            result['annotation_wall_end']=time.time()
        # Export normally closes a complete PNG synchronously. Acknowledge it
        # immediately instead of adding an entire editor tick of avoidable delay.
        if path.is_file() and image_complete(path):
            result["export_complete_wait_ms"]=(time.perf_counter()-export_finished)*1000.
            write('response.json',dict(result,ok=True,image=path.relative_to(ROOT).as_posix()))
        else:
            state["pending"]={"path":path,"result":result,"started":now,
                               "export_submitted_perf":export_finished}
    except Exception as error:
        retryable=str(error).startswith('Unknown generated actor:')
        if request.get("native_stream"):
            stream.error(str(error),request)
        else:fail(request,error,retryable)
        if retryable:state['last_id']=None
        else:unreal.log_error(traceback.format_exc())


def tick(delta):
    if state.get("busy"):return
    state["busy"]=True
    try:tick_impl(delta)
    except Exception:unreal.log_error(traceback.format_exc())
    finally:state["busy"]=False


if hasattr(builtins,"_airground_pre_tick_handle"):
    unreal.unregister_slate_pre_tick_callback(builtins._airground_pre_tick_handle)
    del builtins._airground_pre_tick_handle
    unreal.log("AirGround pre-tick readback polling disabled")

if hasattr(builtins,"_airground_tick_handle"):
    unreal.unregister_slate_post_tick_callback(builtins._airground_tick_handle)
handle=unreal.register_slate_post_tick_callback(tick)
builtins._airground_tick_handle=handle
unreal.log("AirGround native editor capture server ready: "+str(ROOT))
