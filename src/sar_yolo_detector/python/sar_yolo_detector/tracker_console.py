"""Serialized ROS service for bounded, acknowledged live tracker tuning."""
import copy
import json
import time
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from .live_tuning import MANUAL, validate_patch
from .manual_box_tracker import ManualBoxTracker
from .response_quality import ResponseHysteresis


class TrackerConsoleMixin:
    def _effective_live_options(self):
        fusion=self._fusion
        feature=getattr(fusion.tracker,'feature_config',{}) or {}
        config=fusion.config
        return dict(full_frame_interval=int(config.get('full_frame_interval',8)),
            appearance_cache_enabled=bool(config.get('appearance_cache_enabled',False)),
            latency_aware_association=bool(config.get('latency_aware_association',False)),
            latency_uncertainty_gain=float(config.get('latency_uncertainty_gain',1.)),
            occluded_reid_enabled=bool(config.get('occluded_reid_enabled',False)),
            occlusion_freeze_enabled=bool(config.get('occlusion_freeze_enabled',False)),
            tracker_algorithm=str(feature.get('tracker_algorithm',getattr(fusion.tracker,'algorithm','kcf'))),
            nano_pose_search_only=bool(feature.get('nano_pose_search_only',False)),
            nano_occlusion_guard_enabled=bool(feature.get('nano_occlusion_guard_enabled',False)),
            nano_minimum_apce=float(feature.get('nano_minimum_apce',1.8)),
            nano_minimum_psr=float(feature.get('nano_minimum_psr',2.2)),
            nano_recovery_apce=float(feature.get('nano_recovery_apce',2.5)),
            nano_recovery_psr=float(feature.get('nano_recovery_psr',2.8)),
            input_image_transport=str(getattr(self,'_live_transport','raw')))

    def _configure_tracker(self,request):
        from sar_yolo_detector.srv import ConfigureTrackerResponse
        response=ConfigureTrackerResponse()
        new_sub=None
        try:
            value=json.loads(request.config_json or '{}')
            with self._backend_lock,self._lock:
                if self._model_busy:raise ValueError('Model switch is busy')
                current=self._effective_live_options();patch=validate_patch(value,current)
                merged=dict(current,**patch)
                if 'input_image_transport' in patch and merged['input_image_transport']!='raw':
                    source=getattr(self,'_console_source_status',{})
                    if time.monotonic()-source.get('receipt_mono',0.)>1.5 or not source.get('shared_images_enabled',False):
                        raise ValueError('Shared memory is not acknowledged by the UE bridge')
                rebuild=merged['tracker_algorithm']!=current['tracker_algorithm']
                fusion_config=copy.deepcopy(self._fusion.tracker.config)
                feature=fusion_config.setdefault('feature_tracker',{})
                for key,item in patch.items():
                    if key in MANUAL and key!='tracker_algorithm':feature[key]=item
                if 'tracker_algorithm' in patch:
                    feature['tracker_algorithm']=merged['tracker_algorithm']
                    feature['require_opencv_tracker']=True
                replacement=ManualBoxTracker(fusion_config) if rebuild else None
                manual_replacement=None
                manual_tracker=getattr(self,'_manual_trackers',{}).get('feature_tracker')
                if rebuild and manual_tracker is not None:
                    manual_config=copy.deepcopy(manual_tracker.config)
                    manual_feature=manual_config.setdefault('feature_tracker',{})
                    manual_feature.update(feature)
                    manual_replacement=ManualBoxTracker(manual_config)
                if rebuild and (replacement.tracker_type!='feature_tracker' or
                        (manual_replacement is not None and manual_replacement.tracker_type!='feature_tracker')):
                    raise ValueError('Local tracker replacement failed validation')
                transport=merged['input_image_transport']
                if transport!=current['input_image_transport']:
                    topic=rospy.resolve_name(self._input_topic+('/shm' if transport=='shared_memory' else ''))
                    if transport=='shared_memory' and not any(name==topic for name,_ in rospy.get_published_topics()):
                        raise ValueError('Shared image topic is unavailable')
                    new_sub=rospy.Subscriber(topic,String if transport=='shared_memory' else Image,
                        self._shared_image_callback if transport=='shared_memory' else self._image_callback,
                        queue_size=1,buff_size=16*1024*1024,tcp_nodelay=True)
                config_patch={k:v for k,v in patch.items() if k not in MANUAL and k!='input_image_transport'}
                self._fusion.config.update(config_patch)
                if 'full_frame_interval' in patch:
                    schedule=getattr(self,'_detector_schedule',None)
                    if schedule is not None:schedule.configure_interval(merged['full_frame_interval'])
                if rebuild:
                    self._invalidate_pipeline();self._fusion.reset();self._fusion.tracker=replacement
                    self._fusion_snapshots.clear();self._tracker.clear_selection();self._clear_selected_target()
                    if manual_replacement is not None:
                        self._manual_trackers['feature_tracker']=manual_replacement
                        if self._tracking_mode=='feature_tracker':self._manual_active=False
                elif patch:
                    local=self._fusion.tracker
                    local.config=fusion_config;local.feature_config=feature
                    local.nano_pose_search_only=merged['nano_pose_search_only']
                    self._apply_manual_features(local,feature,merged)
                    manual=getattr(self,'_manual_trackers',{}).get('feature_tracker')
                    if manual is not None:
                        manual.config.update(fusion_config);manual.feature_config.update(feature)
                        manual.nano_pose_search_only=merged['nano_pose_search_only']
                        self._apply_manual_features(manual,feature,merged)
                if new_sub is not None:
                    old=self._subscriber;self._subscriber=new_sub;self._live_transport=transport
                    self._image_mailbox.clear()
                    if self._decoded_mailbox is not None:self._decoded_mailbox.clear()
                    old.unregister();new_sub=None
                self._live_revision=getattr(self,'_live_revision',0)+(1 if patch else 0)
                response.success=True
                response.message='Applied; select a new ROI' if rebuild else 'Effective configuration' if not patch else 'Applied'
                response.applied_json=json.dumps(dict(options=self._effective_live_options(),revision=self._live_revision,
                    requires_reselect=rebuild),allow_nan=False)
        except Exception as error:
            response.message=str(error);response.applied_json='{}'
        finally:
            if new_sub is not None:new_sub.unregister()
        return response

    @staticmethod
    def _apply_manual_features(tracker,feature,values):
        backend=getattr(tracker,'_opencv_tracker',None)
        if backend is None or not hasattr(backend,'response_guard'):return
        backend.occlusion_guard_enabled=values['nano_occlusion_guard_enabled']
        backend.minimum_apce=values['nano_minimum_apce'];backend.minimum_psr=values['nano_minimum_psr']
        backend.response_guard=ResponseHysteresis(values['nano_minimum_apce'],values['nano_minimum_psr'],
            values['nano_recovery_apce'],values['nano_recovery_psr'])

    def _recording_console_status(self,message):
        try:
            import json,time
            value=json.loads(message.data)
            self._console_recording_status=(bool(value.get('active')),time.monotonic(),
                str(value.get('record_id','')),str(value.get('caller_id','')))
        except (TypeError,ValueError):pass

    def _console_source_status_callback(self,message):
        try:
            import json,time
            value=json.loads(message.data);value['receipt_mono']=time.monotonic()
            self._console_source_status=value
        except (TypeError,ValueError):pass

    def _fusion_console_overlay(self,stamp,result,candidates):
        fusion=self._fusion;absence=max(0.,stamp-fusion.last_observation_stamp)
        backend=getattr(fusion.tracker,'_opencv_tracker',None)
        quality=(dict(getattr(backend,'response_diagnostics',{}))
            if fusion.diagnostics.get('local_tracker_executed',False) else {})
        feature=getattr(fusion.tracker,'feature_config',{}) or {}
        thresholds=dict(trip_apce=float(feature.get('nano_minimum_apce',1.8)),
            trip_psr=float(feature.get('nano_minimum_psr',2.2)),
            recover_apce=float(feature.get('nano_recovery_apce',2.5)),
            recover_psr=float(feature.get('nano_recovery_psr',2.8)))
        reid=float(fusion.config.get('occluded_reid_max_age_sec',5.))
        source=str(result.source);reason=str(result.reason)
        status='LOCKED' if result.observed else ('COAST' if source=='prediction' and absence<=fusion.predict_hold else
            'REID_WAIT' if getattr(fusion,'_occluded_coast',False) and absence<=reid else
            'EXPIRED' if reason=='occluded_reid_expired' or absence>reid else 'LOST')
        diagnostics=fusion.diagnostics
        raw_corridor=diagnostics.get('reid_corridor')
        corridor=(dict(center=[float(v) for v in raw_corridor['center']],radius=float(raw_corridor['radius']))
            if isinstance(raw_corridor,dict) and raw_corridor.get('center') is not None else None)
        boxes=[]
        for candidate in candidates[:30]:
            uncertainty=getattr(candidate,'association_uncertainty_px',0.)
            boxes.append(dict(box=[float(v) for v in candidate.box],class_id=int(candidate.class_id),
                name=str(getattr(candidate,'class_name','')),local_id=int(candidate.local_id),
                gain_radius_px=float(uncertainty)))
        clean=dict(state=status,public_id=int(result.public_id),local_id=int(result.local_id),
            observed=bool(result.observed),reason=reason,source=source,absence_sec=absence,
            coast_limit_sec=float(fusion.predict_hold),reid_limit_sec=reid,
            template_frozen=bool(getattr(fusion,'_occluded_coast',False)),corridor=corridor,
            anchor_thumbnail=getattr(fusion,'_frozen_thumbnail',None) if getattr(fusion,'_occluded_coast',False) else None,
            quality=quality,thresholds=thresholds,guard_enabled=bool(getattr(backend,'occlusion_guard_enabled',False)),
            capture_boxes=boxes,revision=getattr(self,'_live_revision',0),
            selected_class_id=int(result.class_id))
        return clean
