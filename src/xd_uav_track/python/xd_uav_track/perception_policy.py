"""Shared, ROS-independent scheduling and image evidence gates."""

import math
import cv2
import numpy as np


def admit_frame(source, now, rate):
    """Phase-locked, no-catch-up admission; camera jitter cannot accumulate drift."""
    if rate <= 0:
        source.pop("rate_deadline", None)
        source.pop("rate_limit", None)
        return True
    period = 1.0 / rate
    deadline = source.get("rate_deadline")
    if source.get("rate_limit") != rate or deadline is None:
        deadline = now
    if now + 1e-9 < deadline:
        return False
    # Advance to the next future slot, never queue missed slots in a burst.
    source["rate_deadline"] = deadline + (math.floor(max(0., now-deadline)/period)+1)*period
    source["rate_limit"] = rate
    source["last_accept_wall"] = now
    return True


def detector_rate(mode, tracking, selected_source, source, normal_rate,
                  anchor_rate=1.0, standby_rate=0.5):
    if mode != "track" or not selected_source or not tracking:
        return normal_rate
    requested = anchor_rate if source == selected_source else standby_rate
    return min(normal_rate, requested) if normal_rate > 0 else requested


def image_signature(frame, bbox):
    x1, y1, x2, y2 = [int(round(v)) for v in bbox]
    height, width = frame.shape[:2]
    crop = frame[max(0, y1):min(height, y2), max(0, x1):min(width, x2)]
    if crop.size == 0 or min(crop.shape[:2]) < 8:
        return None
    crop = cv2.resize(crop, (32, 32), interpolation=cv2.INTER_AREA)
    lab = cv2.cvtColor(crop, cv2.COLOR_BGR2LAB)
    hist = cv2.calcHist([lab], [0, 1, 2], None, [8, 8, 8], [0, 256]*3)
    norm = float(np.linalg.norm(hist))
    return hist.flatten() / norm if norm > 0 else None


def evidence_gate(old_box, new_box, dt, reference, observed,
                  minimum_iou=0.15, minimum_cosine=0.55,
                  maximum_speed=800.0, same_capture=False):
    """Require appearance and bounded motion; overlapping competitors reject."""
    if not math.isfinite(dt) or dt < 0 or dt > 1.0:
        return False, "stale_evidence"
    if same_capture and dt != 0.0:
        return False, "capture_mismatch"
    ax1, ay1, ax2, ay2 = old_box
    bx1, by1, bx2, by2 = new_box
    intersection = max(0, min(ax2, bx2)-max(ax1, bx1)) * max(0, min(ay2, by2)-max(ay1, by1))
    area_a = max(1, (ax2-ax1)*(ay2-ay1))
    area_b = max(1, (bx2-bx1)*(by2-by1))
    iou = intersection / max(1, area_a+area_b-intersection)
    distance = math.hypot((bx1+bx2-ax1-ax2)*0.5, (by1+by2-ay1-ay2)*0.5)
    if not 0.4 <= area_b/area_a <= 2.5 or (not same_capture and
            distance > max(12.0, maximum_speed*dt)):
        return False, "motion_conflict"
    # Simultaneous estimators do not define a velocity. Use stricter overlap
    # rather than a spurious 12-pixel "speed" limit for differently sized ROIs.
    if iou < max(minimum_iou, .5 if same_capture else 0.):
        return False, "geometry_conflict"
    if reference is None or observed is None:
        return False, "appearance_unavailable"
    if float(np.dot(reference, observed)) < minimum_cosine:
        return False, "appearance_conflict"
    return True, "accepted"
