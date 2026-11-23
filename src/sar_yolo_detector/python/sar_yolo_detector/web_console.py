"""Exact-frame selection leases, ROS live tuning, and bounded telemetry."""
import json
import math
import threading
import time
import uuid
from collections import OrderedDict, deque
import numpy as np
import rospy
from std_msgs.msg import Header, String
from .live_tuning import PRESETS, validate_patch
from .latency_ledger import breakdown


def _capture_key(header):
    stamp=header.get('stamp',{})
    return (int(stamp.get('secs',0)),int(stamp.get('nsecs',0)),str(header.get('frame_id','')),int(header.get('seq',0)))


class WebConsoleMixin:
    """Methods mixed into Preview without touching the image response headers."""
    def _console_init(self):
        self._console_lock=threading.RLock()
        self._console_condition=threading.Condition(self._console_lock)
        self._console_recorder=dict(active=False,receipt_mono=0.,record_id='',caller_id='')
        self._console_record_topic_configured=False
        self._console_source={}
        self._console_acks=OrderedDict()
        self._console_leases=OrderedDict()
        self._console_overlays=OrderedDict()
        self._console_latencies=deque(maxlen=1000)
        self._console_latency_keys=set()
        self._console_quality=deque(maxlen=256)
        self._console_quality_keys=set()
        self._console_revision=None
        self._console_last_config=None
        self._console_last_config_mono=0.
        self._console_source_events=OrderedDict()
        self._console_runtime_command=rospy.Publisher(self._namespace+'/camera/runtime_command',String,queue_size=8)
        self._console_subscribers=[
            rospy.Subscriber(self._namespace+'/recording/status',String,self._console_on_recording,queue_size=2,tcp_nodelay=True),
            rospy.Subscriber(self._namespace+'/recording/capture_written',Header,self._console_on_capture_written,queue_size=256,tcp_nodelay=True),
            rospy.Subscriber(self._namespace+'/camera/source',String,self._console_on_source,queue_size=1,tcp_nodelay=True),
            rospy.Subscriber(self._namespace+'/camera/runtime_status',String,self._console_on_runtime,queue_size=4,tcp_nodelay=True),
            rospy.Subscriber(self._namespace+'/camera/control_timing',String,self._console_on_control,queue_size=128,tcp_nodelay=True),
            rospy.Subscriber(self._tracker_namespace+'/console_overlay',String,self._console_on_overlay,queue_size=2,tcp_nodelay=True),
        ]
        self._console_configure=rospy.ServiceProxy(self._tracker_namespace+'/configure_tracker',
            __import__('sar_yolo_detector.srv',fromlist=['ConfigureTracker']).ConfigureTracker)

    def _console_on_recording(self,message):
        try:
            value=json.loads(message.data)
            if value.get('protocol')!='exact_capture_bag_write_v1':return
            caller=str(value.get('caller_id',''));record_id=str(value.get('record_id',''))
            with self._console_condition:
                previous=self._console_recorder
                if (record_id!=previous.get('record_id') or caller!=previous.get('caller_id')):
                    self._console_acks.clear()
                    self._console_leases.clear()
                self._console_recorder=dict(active=bool(value.get('active')),receipt_mono=time.monotonic(),
                    record_id=record_id,caller_id=caller,session=str(value.get('session','')),
                    frame_id=str(value.get('frame_id','')))
                self._console_condition.notify_all()
        except (TypeError,ValueError):return

    def _console_on_capture_written(self,message):
        connection=getattr(message,'_connection_header',{}) or {}
        caller=str(connection.get('callerid',''))
        key=(message.stamp.secs,message.stamp.nsecs,message.frame_id)
        with self._console_condition:
            status=self._console_recorder
            if (not status.get('active') or time.monotonic()-status.get('receipt_mono',0.)>1.5
                    or not caller or caller!=status.get('caller_id')):return
            self._console_acks[(status['record_id'],)+key]=time.monotonic()
            while len(self._console_acks)>4096:self._console_acks.popitem(last=False)
            self._console_condition.notify_all()

    def _console_on_source(self,message):
        try:
            value=json.loads(message.data)
            if value.get('schema_version')!=1:return
            with self._console_lock:
                self._console_record_topic_configured=bool(value.get('record_image_topic'))
                self._console_source.update(source_session=str(value.get('session','')),
                    source_frame_id=str(value.get('frame_id','')),source_transport=str(value.get('transport','unknown')))
        except (TypeError,ValueError):return

    def _console_on_runtime(self,message):
        try:
            value=json.loads(message.data)
            with self._console_condition:
                value['_receipt_mono']=time.monotonic();self._console_source=value
                request_id=str(value.get('last_applied_request_id',''))
                if request_id:self._console_source_events[request_id]=dict(value)
                while len(self._console_source_events)>32:self._console_source_events.popitem(last=False)
                self._console_condition.notify_all()
        except (TypeError,ValueError):return

    def _console_on_control(self,message):
        try:
            value=json.loads(message.data)
            capture=str(value.get('capture_ns',''))
            total=float(value.get('total_ms'))
            if not capture.isdigit() or not math.isfinite(total) or total<0 or total>10000:return
            key=capture
            if not value.get('valid') or key in self._console_latency_keys:return
            row=dict(capture_ns=capture,total_ms=total,segments_ms=value.get('segments_ms',{}),
                segment_ledger_valid=bool(value.get('segment_ledger_valid')),
                control_sequence=str(value.get('control_sequence','')),receipt_wall=time.time())
            with self._console_lock:
                self._console_latency_keys.add(key);self._console_latencies.append(row)
                while len(self._console_latency_keys)>1500:self._console_latency_keys.discard(next(iter(self._console_latency_keys)))
        except (TypeError,ValueError):return

    def _console_on_overlay(self,message):
        try:
            value=json.loads(message.data);header=value['capture_header'];key=_capture_key(header)
            with self._console_lock:
                self._console_overlays[key]=value
                while len(self._console_overlays)>256:self._console_overlays.popitem(last=False)
                quality=value.get('quality') or {};apce=quality.get('apce');psr=quality.get('psr')
                qkey=key
                if (qkey not in self._console_quality_keys and self._finite(apce) and self._finite(psr)):
                    self._console_quality.append(dict(capture_ns=str(key[0]*1000000000+key[1]),
                        apce=float(apce),psr=float(psr)))
                    self._console_quality_keys.add(qkey)
                    while len(self._console_quality_keys)>512:self._console_quality_keys.discard(next(iter(self._console_quality_keys)))
        except (KeyError,TypeError,ValueError):return

    @staticmethod
    def _finite(value):
        return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)

    def _console_recording_state(self):
        with self._console_lock:
            source=dict(self._console_source);record=dict(self._console_recorder)
            configured=self._console_record_topic_configured
            ack_count=len(self._console_acks)
        source_fresh=time.monotonic()-source.get('_receipt_mono',0.)<=1.5
        record_fresh=time.monotonic()-record.get('receipt_mono',0.)<=1.5
        active=bool(record_fresh and record.get('active'))
        required=bool(configured or active or (source_fresh and source.get('record_images_enabled',False)))
        return dict(required=required,active=active,recorder_id=record.get('record_id',''),
            ack_samples=ack_count,source_status_fresh=source_fresh,recorder_status_fresh=record_fresh,
            protocol='exact_capture_bag_write_v1' if required else None)

    def freeze_selection(self,body):
        token=str(body.get('frame_token',''))
        with self.lock:
            original=self.history.get(token)
            epoch=self.mode_epoch
        if original is None:return 409,dict(success=False,message='Displayed frame has expired; refresh before selecting')
        header,width,height=original
        age=(rospy.Time.now().to_sec()-(header['stamp']['secs']+header['stamp']['nsecs']*1e-9))
        if age<-.02 or age>self.max_age:return 409,dict(success=False,message='Frozen capture is stale; select the latest visible frame')
        required=self._console_recording_state()['required']
        lease_id=uuid.uuid4().hex
        row=dict(token=token,header=header,width=width,height=height,epoch=epoch,
            created_mono=time.monotonic(),expires_mono=time.monotonic()+self.max_age,
            recorder_required=required)
        with self._console_lock:
            self._console_leases[lease_id]=row
            while len(self._console_leases)>16:self._console_leases.popitem(last=False)
        return 200,dict(success=True,lease_id=lease_id,frame_token=token,capture_header=header,
            lease_expires_in_ms=self.max_age*1000.,recorder_required=required,
            message='Frozen capture; awaiting exact bag-write confirmation' if required else 'Frozen capture')

    def _console_lease_status(self,lease_id):
        with self._console_lock:
            row=self._console_leases.get(str(lease_id))
            if row is None:return dict(known=False,ready=False,reason='selection_lease_missing')
            recorder=dict(self._console_recorder)
            acks=self._console_acks
        if row['epoch']!=self.mode_epoch:return dict(known=True,ready=False,reason='tracking_mode_changed')
        if time.monotonic()>row['expires_mono']:return dict(known=True,ready=False,reason='selection_lease_expired')
        if not row['recorder_required']:return dict(known=True,ready=True,required=False,reason='not_recording')
        if not recorder.get('active') or time.monotonic()-recorder.get('receipt_mono',0.)>1.5:
            return dict(known=True,ready=False,required=True,reason='waiting_for_active_recorder')
        header=row['header'];s=header['stamp'];key=(recorder.get('record_id',''),s['secs'],s['nsecs'],header['frame_id'])
        acked=key in acks
        return dict(known=True,ready=acked,required=True,reason='capture_written' if acked else 'waiting_for_capture_written',
            recorder_id=recorder.get('record_id',''))

    def _console_validate_selection(self,body,token,header):
        if not hasattr(self,'_console_lock'):
            if body.get('lease_id'):raise ValueError('Frozen-frame console is not initialized')
            return
        lease_id=str(body.get('lease_id',''))
        row=None
        if lease_id:
            with self._console_lock:row=self._console_leases.get(lease_id)
            if row is None:raise ValueError('Frozen-frame lease expired or was released; select again')
            if row['token']!=token or row['header']!=header:raise ValueError('Selection does not match the frozen capture')
            if row['epoch']!=self.mode_epoch:raise ValueError('Tracking mode changed after the frame was frozen')
            if time.monotonic()>row['expires_mono']:raise ValueError('Frozen-frame lease expired; select a fresh frame')
        status=self._console_recording_state()
        if not status['required']:return
        recorder=self._console_recorder
        if not status['active']:
            raise ValueError('Recording is configured but no live capture recorder is confirmed')
        stamp=header['stamp'];key=(recorder['record_id'],stamp['secs'],stamp['nsecs'],header['frame_id'])
        with self._console_lock:confirmed=key in self._console_acks
        if not confirmed:raise ValueError('Waiting for capture_written: this exact frame is not on disk yet')
        if row is not None and not row['recorder_required']:
            raise ValueError('Recorder state changed after the frame was frozen; select again')

    def _console_pinned_original(self,token,lease_id):
        if not lease_id or not hasattr(self,'_console_lock'):return None
        with self._console_lock:row=self._console_leases.get(str(lease_id))
        if row is None or row.get('token')!=str(token):return None
        return row['header'],row['width'],row['height']

    def console_snapshot(self,frame_token=None,lease_id=None):
        with self.lock:
            current=dict(self.current) if self.current else None
            history=self.history.get(str(frame_token)) if frame_token else None
            header=history[0] if history else (current.get('capture_header') if current else None)
            width,height=(history[1],history[2]) if history else ((current.get('width'),current.get('height')) if current else (None,None))
        with self._console_lock:
            source=dict(self._console_source);overlays=self._console_overlays
            latencies=list(self._console_latencies);quality=list(self._console_quality)
            revision=self._console_revision
        config=self._console_query_config()
        overlay=overlays.get(_capture_key(header)) if header else None
        stamp=(header or {}).get('stamp',{})
        age_ms=max(0.,(rospy.Time.now().to_sec()-(stamp.get('secs',0)+stamp.get('nsecs',0)*1e-9))*1000.) if header else None
        totals=[x['total_ms'] for x in latencies]
        percentiles=dict(samples=len(totals),p50_ms=float(np.percentile(totals,50)) if totals else None,
            p95_ms=float(np.percentile(totals,95)) if totals else None,window=1000,
            ready=len(totals)>=1000,acceptance_label='rolling live window; not suite acceptance')
        latest_segments=next((row.get('segments_ms',{}) for row in reversed(latencies)
            if row.get('segment_ledger_valid') and isinstance(row.get('segments_ms'),dict)),{})
        status=self._console_recording_state()
        lease=self._console_lease_status(lease_id) if lease_id else None
        source_age_ms=max(0.,(time.time()-float(source.get('capture_wall',time.time())))*1000.) if source else None
        slots=source.get('shared_slots')
        if not isinstance(slots,list):slots=[]
        frame_fresh=bool(current and time.monotonic()-current.get('received_wall',0.)<=2.
            and age_ms is not None and age_ms<=self.max_age*1000.)
        thresholds=(overlay or {}).get('thresholds') or dict(trip_apce=config.get('nano_minimum_apce',1.8),
            trip_psr=config.get('nano_minimum_psr',2.2),recover_apce=config.get('nano_recovery_apce',2.5),
            recover_psr=config.get('nano_recovery_psr',2.8))
        current_ns=int(stamp.get('secs',0))*1000000000+int(stamp.get('nsecs',0))
        quality_age_ms=(max(0.,(current_ns-int(quality[-1]['capture_ns']))/1e6)
            if quality and str(quality[-1].get('capture_ns','')).isdigit() else None)
        return dict(capture_header=header,width=width,height=height,overlay=overlay,
            recording=status,selection_lease=lease,latency=percentiles,latency_segments_ms=latest_segments,
            latency_samples=[x['total_ms'] for x in latencies],quality=quality[-160:],quality_age_ms=quality_age_ms,revision=revision,source=dict(
                fresh=time.monotonic()-source.get('_receipt_mono',0.)<=1.5,
                input_image_transport=config.get('input_image_transport',source.get('input_image_transport')),
                editor_poll_fps=source.get('editor_poll_fps'),shared_images_enabled=source.get('shared_images_enabled',False),
                shared_slots=slots,shared_slots_known=bool(slots),image_age_ms=source_age_ms,
                stale=source_age_ms is not None and source_age_ms>250.,
                record_images_enabled=source.get('record_images_enabled',False),
                runtime_request_id=source.get('last_applied_request_id','')),
            tracker_config=config,thresholds=thresholds,
            control_fresh=bool(frame_fresh and age_ms<=self.control_max_age_ms),
            current_frame_age_ms=age_ms)

    def _console_apply(self,patch):
        current=self._console_query_config(force=True)
        validate_patch(patch,current)
        self._console_configure.wait_for_service(timeout=1.5)
        from sar_yolo_detector.srv import ConfigureTrackerRequest
        result=self._console_configure(ConfigureTrackerRequest(config_json=json.dumps(patch,allow_nan=False)))
        if not result.success:raise ValueError(result.message or 'Tracker rejected live settings')
        applied=json.loads(result.applied_json or '{}')
        with self._console_lock:
            self._console_last_config=applied.get('options',{})
            self._console_last_config_mono=time.monotonic()
            revision=applied.get('revision')
            if revision!=self._console_revision:
                self._console_latencies.clear();self._console_latency_keys.clear();self._console_quality.clear();self._console_quality_keys.clear()
                self._console_revision=revision
        if applied.get('requires_reselect'):
            with self.lock:self.history.clear();self.mode_epoch+=1
        return dict(success=True,message=result.message,options=applied.get('options',{}),
            revision=revision,requires_reselect=bool(applied.get('requires_reselect')))

    def _console_query_config(self,force=False):
        with self._console_lock:
            cached=self._console_last_config
            fresh=time.monotonic()-self._console_last_config_mono<2.
        if cached is not None and fresh and not force:return dict(cached)
        try:
            self._console_configure.wait_for_service(timeout=.4)
            from sar_yolo_detector.srv import ConfigureTrackerRequest
            response=self._console_configure(ConfigureTrackerRequest(config_json='{}'))
            if response.success:
                applied=json.loads(response.applied_json or '{}')
                options=applied.get('options',{})
                with self._console_lock:
                    self._console_last_config=options;self._console_last_config_mono=time.monotonic()
                    self._console_revision=applied.get('revision')
                return dict(options)
        except (rospy.ROSException,rospy.ServiceException,ValueError):pass
        return dict(cached or {})

    def console_command(self,body):
        action=str(body.get('action',''))
        try:
            if action=='freeze':return self.freeze_selection(body)
            if action=='release':
                with self._console_lock:self._console_leases.pop(str(body.get('lease_id','')),None)
                return 200,dict(success=True)
            if action in ('tune','preset'):
                if action=='preset':
                    name=str(body.get('preset',''))
                    if name not in PRESETS:raise ValueError('Unknown experiment preset')
                    patch=dict(PRESETS[name])
                else:
                    patch=body.get('values')
                    if not isinstance(patch,dict):raise ValueError('Tuning values must be an object')
                return 200,self._console_apply(patch)
            if action=='source':return self._console_source_command(body)
            return 400,dict(success=False,message='Unknown console action')
        except (ValueError,KeyError,TypeError,rospy.ROSException,rospy.ServiceException) as error:
            return 409,dict(success=False,message=str(error))

    def _console_source_command(self,body):
        source=str(body.get('source_action',''))
        if source not in ('editor_poll_fps','shared_images_enabled','record_images_enabled'):
            raise ValueError('Unsupported source control')
        value=body.get('value')
        if source=='editor_poll_fps' and (type(value) is not int or value not in (60,120,240)):
            raise ValueError('UE polling rate must be 60, 120 or 240 Hz')
        if source!='editor_poll_fps' and type(value) is not bool:raise ValueError(source+' requires a boolean')
        if source=='record_images_enabled':raise ValueError('Recording channel is fixed when the bridge starts; restart the recorder with record_images enabled')
        request_id=uuid.uuid4().hex
        payload=dict(request_id=request_id,**{source:value})
        self._console_runtime_command.publish(String(data=json.dumps(payload,separators=(',',':'))))
        deadline=time.monotonic()+3.
        with self._console_condition:
            while time.monotonic()<deadline:
                result=self._console_source_events.pop(request_id,None)
                if result is not None:
                    if result.get('last_request_success') is False:
                        raise ValueError(result.get('last_request_message','UE source rejected the change'))
                    return 200,dict(success=True,message='UE source acknowledged the change',source=self._console_public_source(result))
                self._console_condition.wait(min(.1,deadline-time.monotonic()))
        raise ValueError('UE source did not acknowledge this runtime request; current value remains unconfirmed')

    @staticmethod
    def _console_public_source(source):
        return {key:source.get(key) for key in ('input_image_transport','editor_poll_fps','shared_images_enabled',
            'record_images_enabled','shared_slots','capture_wall','last_applied_request_id')}
