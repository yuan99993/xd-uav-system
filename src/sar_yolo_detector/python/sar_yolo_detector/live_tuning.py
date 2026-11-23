"""Bounded runtime controls shared by the ROS adapter and test console."""
import math
BOOLS={'appearance_cache_enabled','latency_aware_association','nano_pose_search_only','nano_occlusion_guard_enabled','occluded_reid_enabled','occlusion_freeze_enabled'}
BOUNDS={'latency_uncertainty_gain':(0.,4.),'nano_minimum_apce':(.1,20.),'nano_minimum_psr':(.1,20.),'nano_recovery_apce':(.1,30.),'nano_recovery_psr':(.1,30.)}
MANUAL={'tracker_algorithm','nano_pose_search_only','nano_occlusion_guard_enabled','nano_minimum_apce','nano_minimum_psr','nano_recovery_apce','nano_recovery_psr'}
PRESETS={
 'v3':dict(full_frame_interval=4,appearance_cache_enabled=False,latency_aware_association=False,
          latency_uncertainty_gain=1.,occluded_reid_enabled=False,occlusion_freeze_enabled=False,
          tracker_algorithm='kcf',nano_pose_search_only=False,nano_occlusion_guard_enabled=False,
          nano_minimum_apce=1.8,nano_minimum_psr=2.2,nano_recovery_apce=2.5,nano_recovery_psr=2.8),
 'fast':dict(full_frame_interval=8,appearance_cache_enabled=True,latency_aware_association=True,occluded_reid_enabled=False,occlusion_freeze_enabled=False,nano_pose_search_only=False,nano_occlusion_guard_enabled=False),
 'reid':dict(full_frame_interval=4,latency_aware_association=True,occluded_reid_enabled=True,occlusion_freeze_enabled=True,nano_occlusion_guard_enabled=True),
 'pose_search':dict(full_frame_interval=4,latency_aware_association=True,
                    tracker_algorithm='nano_trt',nano_pose_search_only=True)}
def validate_patch(value,current):
    if not isinstance(value,dict) or len(value)>20:raise ValueError('Configuration must be a bounded object')
    patch=dict(value)
    for key,v in patch.items():
        if key in BOOLS:
            if type(v) is not bool:raise ValueError(key+' requires a boolean')
        elif key in BOUNDS:
            if type(v) not in (int,float) or not math.isfinite(v) or not BOUNDS[key][0]<=v<=BOUNDS[key][1]:raise ValueError('Out of range: '+key)
        elif key=='full_frame_interval':
            if type(v) is not int or v not in (4,8,12):raise ValueError('full_frame_interval must be 4/8/12')
        elif key=='tracker_algorithm':
            if v not in ('kcf','nano_trt'):raise ValueError('Unsupported local tracker')
        elif key=='input_image_transport':
            if v not in ('raw','shared_memory'):raise ValueError('Unsupported transport')
        else:raise ValueError('Unknown runtime parameter: '+key)
    merged=dict(current,**patch)
    for low,high in [('nano_minimum_apce','nano_recovery_apce'),('nano_minimum_psr','nano_recovery_psr')]:
        if merged.get(low,0)>=merged.get(high,float('inf')):raise ValueError('Recovery threshold must exceed trip threshold')
    return patch

GROUND_VEHICLE_CLASSES=frozenset({'car','tank','truck','bus','ar-car','m142'})
def class_family(name):
    return 'Vehicle [car/tank]' if str(name).lower() in GROUND_VEHICLE_CLASSES else 'Drone' if str(name).lower() in ('drone','uav') else str(name)
