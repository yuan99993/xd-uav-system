"""Bounded image-space motion for each recognized target.

This is a measurement aid, not an identity source: failed texture/flow checks
must never create a new identity or claim that an occluded target was seen.
"""

import math

import cv2
import numpy as np


def box_iou(first, second):
    x1, y1 = max(first[0], second[0]), max(first[1], second[1])
    x2, y2 = min(first[2], second[2]), min(first[3], second[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    area_first = max(0, first[2] - first[0]) * max(0, first[3] - first[1])
    area_second = max(0, second[2] - second[0]) * max(0, second[3] - second[1])
    return intersection / max(1, area_first + area_second - intersection)


def compatible_box(predicted, observed, minimum_iou=0.40):
    """Reject a detector jump before it can re-anchor a locked pixel track."""
    width = max(1, predicted[2] - predicted[0])
    height = max(1, predicted[3] - predicted[1])
    observed_width = max(1, observed[2] - observed[0])
    observed_height = max(1, observed[3] - observed[1])
    scale = (observed_width * observed_height) / float(width * height)
    distance = math.hypot(
        0.5 * (predicted[0] + predicted[2] - observed[0] - observed[2]),
        0.5 * (predicted[1] + predicted[3] - observed[1] - observed[3]))
    return (0.45 <= scale <= 2.2 and
            distance <= max(8.0, 0.30 * math.hypot(width, height)) and
            box_iou(predicted, observed) >= minimum_iou)


def turn_compatible_box(predicted, observed):
    """Allow a tank to change its silhouette during a turn, not its identity.

    Callers must additionally reject this relaxed gate when another target is
    close to the candidate; otherwise two similar tanks can be conflated.
    """
    width = max(1, predicted[2] - predicted[0])
    height = max(1, predicted[3] - predicted[1])
    observed_width = max(1, observed[2] - observed[0])
    observed_height = max(1, observed[3] - observed[1])
    scale = (observed_width * observed_height) / float(width * height)
    distance = math.hypot(
        0.5 * (predicted[0] + predicted[2] - observed[0] - observed[2]),
        0.5 * (predicted[1] + predicted[3] - observed[1] - observed[3]))
    return (0.30 <= scale <= 3.0 and
            distance <= max(10.0, 0.35 * math.hypot(width, height)) and
            box_iou(predicted, observed) >= 0.18)


def blend_boxes(predicted, observed, detector_weight=0.35):
    """The detector may refine scale, but cannot abruptly move the locked box."""
    return tuple(int(round((1.0 - detector_weight) * a + detector_weight * b))
                 for a, b in zip(predicted, observed))


def coast_box(box, image_velocity, dt_sec, image_shape,
              maximum_velocity_px_sec=150.0):
    """Very short visual-only extrapolation when both LK and YOLO are late."""
    if not 0.0 < dt_sec <= 0.5:
        return None
    vx, vy = float(image_velocity[0]), float(image_velocity[1])
    if not math.isfinite(vx) or not math.isfinite(vy):
        return None
    width = max(1, box[2] - box[0])
    height = max(1, box[3] - box[1])
    dx, dy = vx * dt_sec, vy * dt_sec
    distance = math.hypot(dx, dy)
    limit = min(maximum_velocity_px_sec * dt_sec,
                0.30 * math.hypot(width, height))
    if distance > limit and distance > 0.0:
        dx, dy = dx * limit / distance, dy * limit / distance
    image_height, image_width = image_shape[:2]
    x1 = int(round(box[0] + dx))
    y1 = int(round(box[1] + dy))
    x1 = max(0, min(image_width - width, x1))
    y1 = max(0, min(image_height - height, y1))
    return x1, y1, x1 + width, y1 + height


def color_box(previous_frame, current_frame, box, dt_sec, rgb=False):
    """Local hue/saturation centroid fallback for a rotating colored target.

    The search is confined to the prior box neighborhood. Uniform or
    low-saturation backgrounds produce no measurement; this is never used to
    discover a new target or establish identity by color alone.
    """
    if (previous_frame is None or current_frame is None or
            previous_frame.shape != current_frame.shape or
            not 0.0 < dt_sec <= 0.8):
        return None
    height, width = current_frame.shape[:2]
    x1, y1, x2, y2 = [int(value) for value in box]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(width, x2), min(height, y2)
    bw, bh = x2 - x1, y2 - y1
    if bw < 20 or bh < 20:
        return None
    conversion = cv2.COLOR_RGB2HSV if rgb else cv2.COLOR_BGR2HSV
    previous_hsv = cv2.cvtColor(previous_frame[y1:y2, x1:x2], conversion)
    previous_mask = cv2.inRange(previous_hsv, (0, 45, 25), (179, 255, 245))
    colored = int(cv2.countNonZero(previous_mask))
    if colored < max(18, int(0.06 * bw * bh)):
        return None
    histogram = cv2.calcHist([previous_hsv], [0, 1], previous_mask,
                             [18, 8], [0, 180, 0, 256])
    cv2.normalize(histogram, histogram, 0, 255, cv2.NORM_MINMAX)
    previous_score = cv2.calcBackProject([previous_hsv], [0, 1], histogram,
                                         [0, 180, 0, 256], 1)
    reference = float(cv2.mean(previous_score, mask=previous_mask)[0])
    if reference < 12.0:
        return None
    padding_x = max(12, int(0.55 * bw))
    padding_y = max(12, int(0.55 * bh))
    sx1, sy1 = max(0, x1 - padding_x), max(0, y1 - padding_y)
    sx2, sy2 = min(width, x2 + padding_x), min(height, y2 + padding_y)
    current_hsv = cv2.cvtColor(current_frame[sy1:sy2, sx1:sx2], conversion)
    current_mask = cv2.inRange(current_hsv, (0, 45, 25), (179, 255, 245))
    scores = cv2.calcBackProject([current_hsv], [0, 1], histogram,
                                 [0, 180, 0, 256], 1)
    threshold = max(18, int(0.45 * reference))
    scores[current_mask == 0] = 0
    scores[scores < threshold] = 0
    if cv2.countNonZero(scores) < max(12, int(0.20 * colored)):
        return None
    moments = cv2.moments(scores)
    if moments["m00"] <= 0:
        return None
    cx = sx1 + moments["m10"] / moments["m00"]
    cy = sy1 + moments["m01"] / moments["m00"]
    old_cx, old_cy = 0.5 * (x1 + x2), 0.5 * (y1 + y2)
    dx, dy = cx - old_cx, cy - old_cy
    maximum_motion = min(max(6.0, 180.0 * dt_sec),
                         0.60 * math.hypot(bw, bh))
    if math.hypot(dx, dy) > maximum_motion:
        return None
    nx1 = max(0, min(width - bw, int(round(x1 + dx))))
    ny1 = max(0, min(height - bh, int(round(y1 + dy))))
    nx2, ny2 = nx1 + bw, ny1 + bh
    roi = scores[max(ny1, sy1) - sy1:min(ny2, sy2) - sy1,
                 max(nx1, sx1) - sx1:min(nx2, sx2) - sx1]
    # The translated box must still contain a substantial part of the color
    # evidence, not merely be close to a remote background patch.
    if roi.size == 0 or cv2.countNonZero(roi) < max(12, int(0.16 * colored)):
        return None
    return nx1, ny1, nx2, ny2


def flow_box(previous_gray, current_gray, box, dt_sec, min_features=7,
             fb_error_px=1.5, max_velocity_px_sec=300.0,
             image_velocity=None):
    """Forward/backward LK with a bounded motion-predicted initial search."""
    if previous_gray.shape != current_gray.shape or dt_sec <= 0.0:
        return None
    height, width = previous_gray.shape[:2]
    x1 = max(0, min(width - 1, int(box[0])))
    y1 = max(0, min(height - 1, int(box[1])))
    x2 = max(x1 + 1, min(width, int(box[2])))
    y2 = max(y1 + 1, min(height, int(box[3])))
    box_width, box_height = x2 - x1, y2 - y1
    mask = np.zeros((height, width), dtype=np.uint8)
    # Features along the bounding-box edge often belong to the background.
    inset_x, inset_y = int(0.08 * box_width), int(0.08 * box_height)
    mask[y1 + inset_y:y2 - inset_y, x1 + inset_x:x2 - inset_x] = 255
    points = cv2.goodFeaturesToTrack(
        previous_gray, maxCorners=48, qualityLevel=0.015,
        minDistance=4, mask=mask, blockSize=5, useHarrisDetector=False)
    if points is None or len(points) < min_features:
        return None
    options = dict(winSize=(21, 21), maxLevel=3,
                   criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                             20, 0.03))
    next_guess = None
    if image_velocity is not None and len(image_velocity) >= 2:
        vx, vy = float(image_velocity[0]), float(image_velocity[1])
        if math.isfinite(vx) and math.isfinite(vy):
            lead_time = min(dt_sec, 0.25)
            dx, dy = vx * lead_time, vy * lead_time
            distance = math.hypot(dx, dy)
            limit = min(max_velocity_px_sec * lead_time,
                        0.30 * math.hypot(box_width, box_height))
            if distance > limit and distance > 0.0:
                dx, dy = dx * limit / distance, dy * limit / distance
            if math.hypot(dx, dy) >= 1.0:
                next_guess = points + np.array([dx, dy], dtype=np.float32)
                options["flags"] = cv2.OPTFLOW_USE_INITIAL_FLOW
    next_points, forward, _ = cv2.calcOpticalFlowPyrLK(
        previous_gray, current_gray, points, next_guess, **options)
    if (next_guess is not None and
            (next_points is None or forward is None or
             np.count_nonzero(forward) < min_features)):
        options.pop("flags")
        next_points, forward, _ = cv2.calcOpticalFlowPyrLK(
            previous_gray, current_gray, points, None, **options)
    if next_points is None or forward is None:
        return None
    options.pop("flags", None)
    back_points, backward, _ = cv2.calcOpticalFlowPyrLK(
        current_gray, previous_gray, next_points, None, **options)
    if back_points is None or backward is None:
        return None
    old_xy, new_xy, back_xy = (item.reshape(-1, 2) for item in
                               (points, next_points, back_points))
    valid = (forward.reshape(-1) != 0) & (backward.reshape(-1) != 0)
    valid &= np.linalg.norm(old_xy - back_xy, axis=1) <= fb_error_px
    if np.count_nonzero(valid) < min_features:
        return None
    displacement = new_xy[valid] - old_xy[valid]
    median = np.median(displacement, axis=0)
    inliers = np.linalg.norm(displacement - median, axis=1) <= 2.5
    if (np.count_nonzero(inliers) < min_features or
            np.count_nonzero(inliers) / len(displacement) < 0.40):
        return None
    median = np.median(displacement[inliers], axis=0)
    maximum_motion = min(max(6.0, max_velocity_px_sec * dt_sec),
                         max(8.0, 0.75 * max(box_width, box_height)))
    if not np.isfinite(median).all() or np.linalg.norm(median) > maximum_motion:
        return None
    dx, dy = float(median[0]), float(median[1])
    moved = (max(0, min(width, int(round(box[0] + dx)))),
             max(0, min(height, int(round(box[1] + dy)))),
             max(0, min(width, int(round(box[2] + dx)))),
             max(0, min(height, int(round(box[3] + dy)))))
    return moved if moved[2] > moved[0] and moved[3] > moved[1] else None
