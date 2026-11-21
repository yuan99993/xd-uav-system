"""Game-thread coordinator for the optional native camera-stream plugin.

No target truth is used for selecting a view. Scene motion is supplied by the
same simulator pose controller as the legacy capture path.
"""
import json
import time
import uuid
from pathlib import Path
import unreal
from virtual_zoom import VirtualZoom


class StreamHost:
    def __init__(self,state):
        self.state=state;self.api=getattr(unreal,'SarCameraStreamLibrary',None)
        self.config=None;self.pose=None;self.zoom=VirtualZoom();self.frame_seq=0;self.deadline=0.;self.just_stopped=False
        self.token=uuid.uuid4().hex;self.port=-1
        if self.api is not None:self.port=self.api.start_server(self.token,0)
        if self.api is not None and self.port<0:raise RuntimeError('Native camera stream listener failed')

    def info(self):
        return dict(available=self.api is not None,host='127.0.0.1',port=self.port,
                    token=self.token,protocol='SRF1',version=1,
                    python_executable=str(Path(unreal.Paths.convert_relative_path_to_full(unreal.Paths.engine_dir()))/'Binaries/ThirdParty/Python3/Win64/python.exe'))

    def console(self,command):
        editor=unreal.get_editor_subsystem(unreal.UnrealEditorSubsystem)
        unreal.SystemLibrary.execute_console_command(editor.get_game_world() or editor.get_editor_world(),command)

    def status(self,data):
        if self.api is not None and self.api.is_connected():
            self.api.send_metadata(json.dumps(dict(data,kind='gimbal',native_stream_stats=json.loads(self.api.get_stats())),allow_nan=False))

    def error(self,error,request):
        if self.api is not None and self.api.is_connected():
            self.api.send_metadata(json.dumps(dict(kind='error',session=request['session'],error=str(error))))
        self.stop()

    def stop(self):
        if self.config is not None:
            if self.config.get('benchmark_truth'):
                import ue_benchmark_truth
                ue_benchmark_truth.close(self.config['session'])
            gimbal=self.state.get('gimbal')
            if gimbal is not None:gimbal.command(gimbal.last_sequence+1,time.monotonic())
            self.just_stopped=True
            self.console('t.MaxFPS %g' % self.state.pop('capture_saved_max_fps', 30.))
            if 'capture_saved_thread_lag' in self.state:
                self.console('r.OneFrameThreadLag %d' % self.state.pop('capture_saved_thread_lag'))
            for camera in self.state.get('cameras',{}).values():
                if unreal.SystemLibrary.is_valid(camera):
                    camera.get_component_by_class(unreal.SceneCaptureComponent2D).set_editor_property('fov_angle',90.)
        self.config=None;self.pose=None;self.state['stream_active']=False

    def poll(self):
        if self.api is None:return None
        self.api.poll_frames()
        if not self.api.is_connected():self.stop();return None
        for raw in self.api.take_commands():
            try:
                message=json.loads(raw);kind=message['kind']
                if kind=='configure':
                    fps=float(message['fps']);width,height=int(message['width']),int(message['height'])
                    if not 1<=fps<=60 or not 320<=width<=1920 or not 240<=height<=1080:
                        raise ValueError('Invalid native capture rate or dimensions')
                    if message.get('codec','bgra8') not in ('bgr8','bgra8','jpeg'):raise ValueError('Invalid native codec')
                    if message.get('camera_key','down') not in ('down','front'):raise ValueError('Invalid camera key')
                    self.zoom=VirtualZoom(message.get('zoom_config'))
                    self.config=message;self.pose=message.get('pose');self.deadline=0.;self.frame_seq=0
                    self.state['stream_active']=True
                    if 'capture_saved_thread_lag' not in self.state:
                        self.state['capture_saved_thread_lag'] = unreal.SystemLibrary.get_console_variable_int_value('r.OneFrameThreadLag')
                        self.state['capture_saved_max_fps'] = unreal.SystemLibrary.get_console_variable_float_value('t.MaxFPS')
                    self.console('r.OneFrameThreadLag 0')
                    # GPU readback readiness is polled on editor ticks. Keep
                    # that poll faster than the independently limited camera.
                    self.console('t.MaxFPS 60')
                    self.console('r.MotionBlurQuality 0')
                    self.console('r.DepthOfFieldQuality 0')
                    self.console('r.AntiAliasingMethod 1')
                    self.api.send_metadata(json.dumps(dict(kind='configured',session=message['session'],fps=fps)))
                elif kind=='pose':self.pose=message['pose']
                elif kind=='gimbal':self.state['network_gimbal_command']=message
                elif kind=='stop':self.stop()
            except Exception as error:
                self.api.send_metadata(json.dumps(dict(kind='error',session=message.get('session',''),error=str(error))))
                self.stop()
        now=time.monotonic()
        if self.config is not None and self.pose is not None:
            self.zoom.step(now,float(self.pose.get('horizontal_fov_target_deg',90.)))
        if self.config is None or self.pose is None or now<self.deadline:return None
        period=1./float(self.config['fps'])
        self.deadline=(now+period if self.deadline<=0 or now-self.deadline>=period else self.deadline+period)
        config=self.config
        # Immutable per-frame pose/metadata. Late readbacks keep this frame's
        # identity, calibration and pose rather than querying newer state.
        value=dict(self.pose,id=self.frame_seq,frame_seq=self.frame_seq,session=config['session'],
                   op='capture',native_stream=True,benchmark_truth=config.get('benchmark_truth',False),camera_key=config.get('camera_key','down'),
                   virtual_gimbal=config.get('virtual_gimbal',True),gimbal_config=config.get('gimbal_config',{}),
                   width=config['width'],height=config['height'],codec=config.get('codec','bgra8'),
                   quality=int(config.get('quality',90)),horizontal_fov_deg=self.zoom.axis.angle)
        self.frame_seq+=1
        return value

    def capture(self,target,metadata,request):
        value=dict(metadata,kind='frame',frame_seq=request['frame_seq'],
                   capture_method='SceneCapture2D/async_gpu_readback/SRF1',protocol_version=1)
        value.pop('objects',None);value['truth']='none'
        return self.api.capture(target,json.dumps(value,allow_nan=False),request['codec'],request['quality'])
