"""Presentation contract: image observations, bounded predictions and identity-only loss."""

import numpy as np


def observation_age_ms(capture_stamp, last_observation, now):
    if not last_observation or last_observation.is_zero() or last_observation > capture_stamp:
        return -1.0
    reference = max(capture_stamp, now)
    return max(0.0, (reference - last_observation).to_sec() * 1000.0)


def metric_position_valid(track, capture_stamp, expected_frame, now, maximum_age_sec):
    if (not track.detected or track.predicted or track.lifecycle_state != "confirmed" or
            not track.control_measurement_ready or not track.has_relative_position_body or
            not track.range_valid or track.position_frame != expected_frame or
            track.position_stamp.is_zero() or track.position_stamp != capture_stamp):
        return False
    if not now.is_zero() and (capture_stamp - now).to_sec() > 0.05:
        return False
    age = observation_age_ms(capture_stamp, track.position_stamp, now)
    if age < 0.0 or age > maximum_age_sec * 1000.0:
        return False
    position = np.asarray(track.relative_position_body, dtype=float)
    covariance = np.asarray(track.position_covariance, dtype=float).reshape(3, 3)
    return bool(np.isfinite(position).all() and np.isfinite(covariance).all() and
                np.allclose(covariance, covariance.T, atol=1e-6, rtol=1e-5) and
                np.linalg.eigvalsh(covariance).min() >= -1e-6)

def target_status(target):
    if target.lifecycle_state == "lost" or target.measurement_source == "stale":
        return "LOST"
    return {"detector": "DET", "visual_tracker": "SOT",
            "prediction": "PRED"}.get(target.measurement_source, "LOST")


def identity_only(target):
    target.lifecycle_state = "lost"
    target.measurement_source = "stale"
    target.tracking_quality = 0.0
    target.position_valid = False
    target.bbox_xyxy = [0, 0, 0, 0]
    target.bbox_cxcywh_norm = [0.0, 0.0, 0.0, 0.0]
    target.center_px = [0.0, 0.0]
    return target


def bound_prediction(target, capture_stamp, last_observation, horizon_sec, camera_stale=False):
    status = target_status(target)
    if status == "LOST" or camera_stale:
        return identity_only(target)
    if status == "PRED":
        age = (capture_stamp - last_observation).to_sec() if last_observation else -1.0
        if target.lifecycle_state != "occluded" or age < 0.0 or age > horizon_sec:
            return identity_only(target)
        # Image prediction is not a fresh 3-D measurement or a detection confidence.
        target.position_valid = False
        target.detection_confidence = 0.0
    return target
