#!/usr/bin/python3
"""Run visual stress cases through the unchanged live four-mode stack.

Default kinematic observer isolates vision; --pose-source px4 explicitly adds
flight dynamics. Given initial ROI scores are not detector mAP.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rospkg
from sar_yolo_detector.local_ros_transport import local_ros_environment
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--project',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--python-executable',required=True,help='Model environment Python with D-FINE and OpenCV contrib')
parser.add_argument('--no-record',action='store_true',help='Isolate live visual lock without raw ROS bag I/O; states/traces/frames remain recorded')
parser.add_argument('--image-width',type=int)
parser.add_argument('--image-height',type=int)
parser.add_argument('--platform-config',type=Path)
parser.add_argument('--track-config',type=Path,
 help='Optional xd_uav_track config override, used to isolate the detector/tracker test policy')
parser.add_argument('--model-manifest',type=Path)
parser.add_argument('--visual-manifest',type=Path)
parser.add_argument('--shared-memory',action='store_true')
parser.add_argument('--bridge-nice',type=int,choices=range(-10,11),default=0)
parser.add_argument('--editor-poll-fps',type=int,choices=(60,90,120,240),default=60)
parser.add_argument('--stream-codec',choices=('jpeg','bgr8','bgra8'),default='jpeg',
 help='Wire format for the native UE stream; JPEG reduces cross-system payload bytes')
parser.add_argument('--stream-quality',type=int,choices=range(1,101),default=98,metavar='1..100')
parser.add_argument('--fov-profile',type=Path)
parser.add_argument('--require-stage-pass',action='store_true')
parser.add_argument('--seconds',type=float,default=20)
parser.add_argument('--jpeg-decode-device',choices=['cpu','cuda'])
parser.add_argument('--minimum-motion-warp-px',type=float)
parser.add_argument('--feature-tracker-algorithm',choices=['kcf','csrt','nano','nano_trt'])
parser.add_argument('--moving-pixel-witness',action='store_true',help='Compare strict same-frame pixel verification in the moving-target profile')
parser.add_argument('--roi-padding',type=int,choices=range(9),default=0)
parser.add_argument('--pose-source',choices=['kinematic','px4'],default='kinematic')
parser.add_argument('--cases',nargs='+',default=['ground_2mps_h60','ground_6mps_h60','air_6mps_h60','occlusion_6mps_h60'])
parser.add_argument('--launch-file',choices=['ue_dfine_v3_live.launch','ue_civil_static_lock.launch','ue_civil_moving_targets.launch'],default='ue_dfine_v3_live.launch')
parser.add_argument('--master-uri',default=os.environ.get('ROS_MASTER_URI','http://127.0.0.1:11461'))
args=parser.parse_args()
if args.moving_pixel_witness and args.launch_file!='ue_civil_moving_targets.launch':
 parser.error('--moving-pixel-witness requires ue_civil_moving_targets.launch')
if args.fov_profile is not None and args.launch_file=='ue_civil_static_lock.launch':
 parser.error('--fov-profile is not available in the static lock profile')
if not 10<=args.seconds<=120:parser.error('--seconds must be in [10,120]')
package=Path(rospkg.RosPack().get_path('sar_yolo_detector'))
project=str(args.project.resolve());out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
lock_path=args.project/'Saved/AirGroundExchange/visual_benchmark.lock'
lock_path.parent.mkdir(parents=True,exist_ok=True)
_suite_lock=lock_path.open('a')
try:fcntl.flock(_suite_lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:parser.error('Another benchmark suite owns this UE camera; wait for it to finish')
profiles=args.cases
for name in profiles:
 if Path(name).name!=name or not (package/'config/ue/benchmark_cases'/(name+'.json')).is_file():parser.error('Unknown case: '+name)
if len(set(profiles))!=len(profiles):parser.error('Duplicate cases are not allowed')
if any(out.iterdir()):parser.error('--output must be a new or empty directory')
# Match the source wrapper's approved contrib overlay, while keeping ROS bag
# processes on their own system-Python ABI.
# A new editor can display its window before Python capture startup completes.
editor_deadline=time.monotonic()+240.
while time.monotonic()<editor_deadline:
 try:
  editor_ready=json.loads((args.project/'Saved/AirGroundExchange/ready.json').read_text())
  if 0<=time.time()-float(editor_ready['heartbeat'])<3.:break
 except (OSError,ValueError,KeyError):pass
 time.sleep(.25)
else:raise RuntimeError('UE capture server heartbeat did not become fresh before launch')
visual_manifest=None
if args.visual_manifest:
 visual_manifest=json.loads(args.visual_manifest.read_text())
 expected=visual_manifest['visual_variant']['level']
 if not editor_ready.get('world','').startswith(expected+'.'):
  raise RuntimeError('UE world differs from the requested visual variant')
preflight_env=dict(os.environ)
workspace=package.parent.parent
contrib=workspace/'.venv-fusion/lib/python3.10/site-packages'
if args.python_executable==str(workspace/'.venv-dfine/bin/python') and contrib.is_dir():
 preflight_env['PYTHONPATH']=str(contrib)+os.pathsep+preflight_env.get('PYTHONPATH','')
preflight_code="import cv2,json; k=callable(getattr(cv2,'TrackerKCF_create',None)) or callable(getattr(getattr(cv2,'legacy',None),'TrackerKCF_create',None)); c=callable(getattr(cv2,'TrackerCSRT_create',None)) or callable(getattr(getattr(cv2,'legacy',None),'TrackerCSRT_create',None)); assert k and c, 'Install pinned OpenCV contrib: KCF and CSRT are required'; print(json.dumps(dict(cv2_version=cv2.__version__,cv2_path=cv2.__file__,kcf=k,csrt=c)))"
runtime=json.loads(subprocess.check_output([args.python_executable,'-c',preflight_code],env=preflight_env,text=True))
def source_snapshot():
 roots=[package/'scripts',package/'python',package/'config',package/'launch',package/'tools/ue',package/'ue_plugin',package/'web/dist']
 xd=package.parent/'xd_uav_track'
 roots.extend(xd/name for name in ['src','include','python','config','launch','scripts'])
 suffixes={'.py','.launch','.json','.yaml','.yml','.cpp','.h','.ini','.uplugin','.html','.js','.css'}
 files={p for root in roots for p in root.rglob('*') if p.is_file() and p.suffix in suffixes and '__pycache__' not in p.parts}
 files.update(package/'scripts'/name for name in ['sar_yolo_smart_tracker','ue_camera_bridge'])
 for binary in [workspace/'devel/lib/xd_uav_track/xd_uav_track_node',workspace/'devel/lib/libxd_uav_track_core.so',workspace/'lib/xd_uav_track/xd_uav_track_node',workspace/'lib/libxd_uav_track_core.so']:
  if binary.is_file():files.add(binary)
 return {os.path.relpath(p,package):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
frozen_sources=source_snapshot()
(out/'suite_manifest.json').write_text(json.dumps(dict(pose_source=args.pose_source,cases=profiles,requested_editor_poll_fps=args.editor_poll_fps,requested_bridge_nice=args.bridge_nice,requested_shared_memory=args.shared_memory,ue_editor_world=editor_ready.get('world'),
 visual_manifest=visual_manifest,visual_manifest_sha256=hashlib.sha256(args.visual_manifest.read_bytes()).hexdigest() if args.visual_manifest else None,source_sha256=frozen_sources,requested_feature_tracker_algorithm=args.feature_tracker_algorithm,
 requested_platform_config=str(args.platform_config.resolve()) if args.platform_config else None,
 requested_platform_sha256=hashlib.sha256(args.platform_config.read_bytes()).hexdigest() if args.platform_config else None,
 requested_track_config=str(args.track_config.resolve()) if args.track_config else None,
 requested_track_sha256=hashlib.sha256(args.track_config.read_bytes()).hexdigest() if args.track_config else None,
 requested_model_manifest=str(args.model_manifest.resolve()) if args.model_manifest else None,
 requested_fov_profile=json.loads(args.fov_profile.read_text()) if args.fov_profile else None,requested_stream_quality=args.stream_quality,requested_moving_pixel_witness=args.moving_pixel_witness,requested_jpeg_decode_device=args.jpeg_decode_device,requested_minimum_motion_warp_px=args.minimum_motion_warp_px,
 ros_network={k:v for k,v in local_ros_environment(dict(os.environ,ROS_MASTER_URI=args.master_uri)).items() if k in ('ROS_MASTER_URI','ROS_IP','ROS_HOSTNAME','SAR_LOCAL_WEB_HOST')},
 evaluation='given initial ROI, evaluation-only projected world AABB and centre ray',
 simulation_truth_consumed_by_algorithms=False,initial_roi_padding_px=args.roi_padding,seconds_per_case=args.seconds,local_tracker_runtime=runtime,record_raw_rgb=not args.no_record),indent=2)+'\n')

def stop_owned(child,grace=10):
 if child is None or child.poll() is not None:return
 for signum,seconds in [(signal.SIGINT,grace),(signal.SIGTERM,5),(signal.SIGKILL,5)]:
  try:os.killpg(child.pid,signum)
  except ProcessLookupError:pass
  try:child.wait(timeout=seconds);return
  except subprocess.TimeoutExpired:continue
 raise RuntimeError('Owned benchmark process did not exit: '+str(child.pid))

results=[]
for name in profiles:
 if source_snapshot()!=frozen_sources:raise RuntimeError("Source changed during this suite; run a new frozen validation")
 capture=out/(name+'_live');evaluation=out/(name+'_eval');env=local_ros_environment(dict(os.environ,ROS_MASTER_URI=args.master_uri))
 env['NO_PROXY']=env.get('NO_PROXY','')+',127.0.0.1,localhost,::1'
 env['no_proxy']=env['NO_PROXY']
 env['PYTHONPATH']=os.pathsep.join(p for p in env.get('PYTHONPATH','').split(os.pathsep) if '/python3.10/site-packages' not in p)
 with (out/(name+'_live.log')).open('w') as live_log,(out/(name+'_eval.log')).open('w') as eval_log:
  launch=subprocess.Popen(['roslaunch',str(package/'launch'/args.launch_file),'ue_project:='+project,
   'pose_source:='+args.pose_source,'bridge_nice:='+str(args.bridge_nice),'editor_poll_fps:='+str(args.editor_poll_fps),'record_images:='+('false' if args.no_record else 'true'),'shared_images:='+('true' if args.shared_memory else 'false'),'scene:='+str(package/'config/ue/benchmark_cases'/(name+'.json')),
   'gimbal_config:='+str(package/'config/ue/benchmark_cases'/(name+'_gimbal.json')),
   'stream_codec:='+args.stream_codec,'stream_quality:='+str(args.stream_quality),'seconds:=360',
   'output:='+str(capture),'python_executable:='+args.python_executable]+
   (['platform_config:='+str(args.platform_config.resolve())] if args.platform_config else [])+
   (['track_config:='+str(args.track_config.resolve())] if args.track_config else [])+
   (['image_width:='+str(args.image_width)] if args.image_width else [])+
   (['image_height:='+str(args.image_height)] if args.image_height else [])+
   (['model_path:='+str(args.model_manifest.resolve()),'model_sha256:='+hashlib.sha256(args.model_manifest.read_bytes()).hexdigest()] if args.model_manifest else [])+(['fov_profile:='+str(args.fov_profile.resolve())] if args.fov_profile else [])+(['experimental_moving_pixel_witness:=true'] if args.moving_pixel_witness else [])+([] if args.jpeg_decode_device is None else ['jpeg_decode_device:='+args.jpeg_decode_device])+([] if args.minimum_motion_warp_px is None else ['minimum_motion_warp_px:='+str(args.minimum_motion_warp_px)])+([] if args.feature_tracker_algorithm is None else ['feature_tracker_algorithm:='+args.feature_tracker_algorithm]),stdout=live_log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  evaluate=subprocess.Popen(['/usr/bin/python3',str(Path(__file__).resolve().with_name('verify_ue_stress_case.py')),'--project',project,
   '--capture',str(capture),'--output',str(evaluation),'--seconds',str(args.seconds),'--roi-padding',str(args.roi_padding)]+([] if args.no_record else ['--record-ready',str(out/(name+'_record_ready.json'))])+(['--web-url','http://'+env['SAR_LOCAL_WEB_HOST']+':8787'] if env.get('SAR_LOCAL_WEB_HOST') else []),stdout=eval_log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
  recorder=None;record_log=None;deadline=time.monotonic()+max(320,args.seconds+270)
  try:
   while evaluate.poll() is None:
    if launch.poll() is not None:
     stop_owned(evaluate);break
    flight_file=out/(name+'_live_px4')/'status.json'
    recording_window_ready=(evaluation/'camera_barrier.json').exists()
    if args.pose_source=='px4':
     try:recording_window_ready=recording_window_ready and json.loads(flight_file.read_text()).get('stage')=='inspect_01'
     except (OSError,ValueError):recording_window_ready=False
    if not args.no_record and recorder is None and recording_window_ready:
     record_log=(out/(name+'_record.log')).open('w')
     recorder=subprocess.Popen(['/usr/bin/python3',str(Path(__file__).resolve().with_name('record_ue_rgb.py')),'--capture-ack','--seconds',str(args.seconds+240),'--ready-file',str(out/(name+'_record_ready.json')),
       '--output',str(out/(name+'.bag'))],stdout=record_log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
    if time.monotonic()>deadline:stop_owned(evaluate);raise RuntimeError('Evaluation timeout '+name)
    time.sleep(.2)
   if recorder:
    try:
     stop_owned(recorder,45)
    except subprocess.TimeoutExpired:stop_owned(recorder,15)
   result=json.loads((evaluation/'result.json').read_text()) if (evaluation/'result.json').exists() else dict(case_id=name,executed=False,reason='Evaluation process exited '+str(evaluate.returncode))
   result['record_exit_code']=None if recorder is None else recorder.returncode
   if recorder is not None:
    record_text=(out/(name+'_record.log')).read_text()
    result['record_buffer_overflow']='Dropping oldest queued message' in record_text
    validation=subprocess.run(['/usr/bin/python3',str(Path(__file__).resolve().with_name('verify_car_selection_recording.py')),
      '--bag',str(out/(name+'.bag')),'--evaluation',str(evaluation),'--output',str(out/(name+'_selection_validation'))],
      stdout=eval_log,stderr=subprocess.STDOUT,env=env)
    result['record_selection_passed']=validation.returncode==0 and recorder.returncode==0 and not result['record_buffer_overflow']
    if not result['record_selection_passed']:
     result['passed']=False
     result['recording_failure']='Initial capture/calibration/processing evidence or recording completeness failed'
   results.append(result);print(json.dumps(result),flush=True)
  finally:
   stop_owned(evaluate)
   stop_owned(launch,35)
   stop_owned(recorder,15)
   if record_log:record_log.close()
  if (capture/'run.json').exists():
   session=json.loads((capture/'run.json').read_text())['session'];source=Path(project)/'Saved/AirGroundExchange/BenchmarkTruth'/(session+'.jsonl')
   if source.exists():
    import shutil
    shutil.copy2(source,evaluation/'truth.jsonl')
 if args.require_stage_pass and result.get('executed'):
  from sar_yolo_detector.fov_acceptance import evaluate_stage
  requested = json.loads(args.fov_profile.read_text())['schedule'][0][1] if args.fov_profile else 90.
  evaluate_stage(result,capture,requested,name.startswith('occlusion_'))
 if source_snapshot()!=frozen_sources:
  result['source_changed_during_case']=True
  result['evaluation_valid']=False
  result['fov_stage_acceptance']=False
  result['tracking_acceptance']=False
  if 'static_lock_acceptance' in result:result['static_lock_acceptance']=False
 (out/'suite_results.json').write_text(json.dumps(results,indent=2)+'\n')
 if args.require_stage_pass and not result.get('fov_stage_acceptance',False):
  raise SystemExit(2)
