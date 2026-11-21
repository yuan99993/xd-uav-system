"""Geometry/provenance vetoes for selected raw proposals, before either arbiter path.

No clocks, images, tracker mutation or evaluation truth is consumed here.
"""
import math
from .box_fusion import center, iou


def rejection_reason(candidate, proposals, *, stamp, last_detection_stamp,
                     reference, public_id, class_groups, independent_history=True,
                     anchor_area=None, predicted_reference=None):
    measured = stamp if candidate.detector_stamp is None else candidate.detector_stamp
    if candidate.camera_projected or measured < stamp:
        return "transported_detection_not_current"
    reference_area = max(1., reference[2] * reference[3], anchor_area or 0.)
    if measured - last_detection_stamp > 1.0:
        ratio = candidate.box[2] * candidate.box[3] / reference_area
        predicted = predicted_reference if predicted_reference is not None else reference
        distance = math.hypot(*(center(candidate.box)-center(predicted)))
        reach = 3. + .5 * math.hypot(*reference[2:])
        if distance > reach or not .60 <= ratio <= 1.60:
            return "long_gap_identity_geometry_changed"
        # A close, current candidate may use the existing appearance and
        # consecutive-confirmation path, never direct strong takeover.
    area = candidate.box[2] * candidate.box[3]
    # A single expanded detector box must not rewrite independent size/history
    # and make the adjacent compact vehicle reachable on the following frame.
    # Check against the camera-compensated detector reference, not local KCF/LK.
    if (independent_history and area / reference_area > 1.60 and
            max(candidate.box[2] / max(1., reference[2]),
                candidate.box[3] / max(1., reference[3])) > 1.50):
        return "abrupt_detector_scale_expansion"
    diagonal = math.hypot(*candidate.box[2:])
    for other in proposals:
        if (other is candidate or other.selected_public_id != public_id
                or not math.isfinite(other.confidence) or other.confidence < .50
                or other.confidence <= candidate.confidence
                or (other.detector_stamp is not None and other.detector_stamp != stamp)
                or len(other.box) != 4 or not all(math.isfinite(v) for v in other.box)
                or other.box[2] <= 0 or other.box[3] <= 0):
            continue
        if other.class_id != candidate.class_id and not any(
                other.class_id in group and candidate.class_id in group for group in class_groups):
            continue
        other_area = other.box[2] * other.box[3]
        overlap = iou(candidate.box, other.box)
        intersection = overlap * (area + other_area) / (1.0 + overlap)
        separation = math.hypot(*(center(candidate.box) - center(other.box)))
        if (area >= 1.75 * other_area and intersection >= .90 * other_area
                and separation > .15 * diagonal):
            return "expanded_box_contains_competing_strong_target"
    return None
