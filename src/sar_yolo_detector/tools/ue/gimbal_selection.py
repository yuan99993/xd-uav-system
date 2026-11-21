"""Capture-time selected-pixel to world sight line, independent of ROS/UE.

The same timing rule as the Gazebo web gimbal: only a fresh accepted visual
measurement may update the sight line. Prediction boxes cannot refresh it.
"""
import math
import threading
from collections import deque, Counter
import numpy as np
from geometry import unreal_camera_optical_rotation
from target_geometry import sightline_evidence


def current_visual_measurement(state):
    """Detector-current differs from a current, accepted local visual update.

    SmartTrackerState.measurement_current describes detector evidence. The
    producer's control-ready flag and explicit association source identify
    accepted local measurements; predictions never qualify.
    """
    method=state.get('association_method','').lower()
    local=method=='fusion_visual' or method in {
        'manual_kcf','manual_csrt','manual_mil','manual_nano','manual_nanotrack','manual_template',
        'manual_color_roi_camshift','manual_color_auto_contrast','manual_color_roi_contrast',
        *('manual_color_'+name+'_contrast' for name in ('red','green','blue','white','black','custom'))}
    return bool(state['control_ready'] and (state['measurement_current'] or local)
        and not state['prediction_only'] and not state['stale'] and not state['tentative'])


class SelectionSightline:
    def __init__(self, maximum_age=1.2, minimum_confidence=.15):
        self.frames = deque(maxlen=128)
        self.maximum_age = float(maximum_age)
        self.minimum_confidence = float(minimum_confidence)
        self.lock = threading.RLock()
        self.last_stamp = 0.
        self.selection = None
        self.counts = Counter()

    def capture(self, response, frame_id):
        w,h = response['width'],response['height']
        f = w/(2*math.tan(math.radians(response['horizontal_fov_deg'])/2))
        with self.lock:
            self.frames.append((response['capture_wall_start'], frame_id,w,h,f,
                                unreal_camera_optical_rotation(response['measured_camera_rotation_deg']),
                                response.get('camera_position_ned_m')))

    def observe(self, state, now):
        with self.lock:
            return self._observe(state, now)

    def _observe(self, state, now):
        stamp = float(state['stamp'])
        if not math.isfinite(stamp) or stamp < self.last_stamp:
            self.counts['out_of_order'] += 1;return None
        selection = (state['track_id'],state['tracker_type']) if state['has_selection'] else None
        if selection is None:
            if self.selection is not None:
                self.selection = None;self.last_stamp = stamp
                self.counts['cancelled'] += 1
                return dict(cancel=True, geometry=dict(schema_version=1,measurement_stamp=stamp,
                    observation_kind='image_sightline',measurement_valid=False,reason='selection_cleared',
                    range_valid=False,target_position_valid=False,ground_projection_applied=False))
            return None
        if stamp == self.last_stamp:
            self.counts['duplicate'] += 1;return None
        if not current_visual_measurement(state):
            self.counts['not_measurement'] += 1;return None
        if not 0 <= now-stamp <= self.maximum_age:
            self.counts['stale'] += 1;return None
        if not self.frames:
            self.counts['no_capture'] += 1;return None
        frame = min(self.frames,key=lambda f:abs(f[0]-stamp))
        if abs(frame[0]-stamp) > .0001 or frame[1] != state['frame_id']:
            self.counts['capture_mismatch'] += 1;return None
        _,_,w,h,f,rotation,origin = frame
        u,v = state['center']
        if not (all(math.isfinite(x) for x in (u,v,state['confidence'])) and
                0 <= u < w and 0 <= v < h and state['confidence'] >= self.minimum_confidence):
            self.counts['invalid_box'] += 1;return None
        ray = rotation @ np.array([(u-w/2)/f,(v-h/2)/f,1.])
        yaw = math.degrees(math.atan2(ray[1],ray[0]))
        pitch = math.degrees(math.atan2(-ray[2],math.hypot(ray[0],ray[1])))
        evidence=sightline_evidence(state,ray,origin)
        evidence.update(measurement_valid=True,valid_until_wall=stamp+self.maximum_age)
        self.last_stamp,self.selection = stamp,selection
        self.counts['accepted'] += 1
        return dict(cancel=False,yaw_world_deg=yaw,pitch_world_deg=pitch,
                    measurement_stamp=stamp,track_id=selection[0],tracker_type=selection[1],
                    geometry=evidence,
                    center_px=[u,v],error_norm=[(u-w/2)/(w/2),(v-h/2)/(h/2)])
