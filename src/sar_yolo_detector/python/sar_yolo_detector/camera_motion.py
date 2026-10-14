"""Validated image-plane camera compensation, never a 3-D translation claim."""
from collections import deque
import math

import cv2
import numpy as np


def transform_box(box, matrix):
    x, y, width, height = box
    corners = np.float32([[x, y], [x+width, y], [x+width, y+height], [x, y+height]])
    projected = cv2.perspectiveTransform(corners.reshape(-1, 1, 2), matrix).reshape(-1, 2)
    if not np.isfinite(projected).all():
        return None
    left, top = projected.min(axis=0)
    right, bottom = projected.max(axis=0)
    return float(left), float(top), float(right-left), float(bottom-top)


def quaternion_rotation(values):
    x, y, z, w = np.asarray(values, dtype=float)
    norm = np.linalg.norm((x, y, z, w))
    if not np.isfinite(norm) or not .9 <= norm <= 1.1:
        raise ValueError("Invalid camera quaternion")
    x, y, z, w = np.asarray((x, y, z, w))/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rotation_homography(intrinsics, previous_rotation, current_rotation):
    matrix = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    if not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("Invalid camera intrinsics")
    # Both rotations map camera optical coordinates into the same fixed frame.
    result = matrix @ current_rotation.T @ previous_rotation @ np.linalg.inv(matrix)
    return result/result[2, 2]


class CameraMotion:
    def __init__(self, config=None):
        config = dict(config or {})
        self.enabled = bool(config.get("enabled", True))
        self.max_gap = float(config.get("max_gap_sec", .75))
        self.max_features = int(config.get("max_features", 160))
        self.minimum_inliers = int(config.get("minimum_inliers", 12))
        if not math.isfinite(self.max_gap) or self.max_gap <= 0 or not 16 <= self.max_features <= 512:
            raise ValueError("Invalid camera motion bounds")
        if not 6 <= self.minimum_inliers <= self.max_features:
            raise ValueError("Invalid camera motion inlier minimum")
        self.reset()

    def reset(self):
        self.gray = None
        self.stamp = 0.
        self.source = "unavailable"
        self.inlier_ratio = 0.
        self.history = deque(maxlen=32)

    @staticmethod
    def validate(matrix, shape):
        if matrix is None or np.shape(matrix) != (3, 3) or not np.isfinite(matrix).all():
            return False
        height, width = shape[:2]
        points = np.float32([[0, 0], [width, 0], [width, height], [0, height], [width/2, height/2]])
        homogeneous = np.c_[points, np.ones(5)] @ matrix.T
        if np.any(homogeneous[:, 2] <= .2):
            return False
        projected = homogeneous[:, :2]/homogeneous[:, 2:]
        if np.max(np.linalg.norm(projected-points, axis=1)) > .3*math.hypot(width, height):
            return False
        area = abs(cv2.contourArea(projected[:4].astype(np.float32))) / max(1., width*height)
        return .65 <= area <= 1.5

    def update(self, frame, stamp, excluded_boxes=(), pose_rotation=None, estimate=True):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        matrix, self.source, self.inlier_ratio = None, "unavailable", 0.
        previous, previous_stamp = self.gray, self.stamp
        if self.enabled and estimate and previous is not None and previous.shape == gray.shape and 0 < stamp-previous_stamp <= self.max_gap:
            pose = pose_rotation if self.validate(pose_rotation, gray.shape) else None
            matrix = self._background(previous, gray, excluded_boxes, pose)
            if matrix is not None:
                self.source = "pose_seeded_background" if pose is not None else "background"
            elif pose is not None:
                matrix, self.source = pose, "pose_rotation_only"
            self.history.append((previous_stamp, stamp, matrix))
        else:
            self.history.clear()
        self.gray, self.stamp = gray, stamp
        return matrix

    def _background(self, previous, current, boxes, pose):
        # Background GMC does not need full sensor resolution. Keep the target
        # images untouched; conjugate the estimated transform back to pixels.
        scale = min(1., 320./previous.shape[1])
        if scale == 1.:
            return self._background_scaled(previous, current, boxes, pose)
        size = (round(previous.shape[1]*scale), round(previous.shape[0]*scale))
        scaling = np.diag([scale, scale, 1.])
        small_pose = scaling @ pose @ np.linalg.inv(scaling) if pose is not None else None
        estimated = self._background_scaled(
            cv2.resize(previous, size, interpolation=cv2.INTER_AREA),
            cv2.resize(current, size, interpolation=cv2.INTER_AREA),
            [tuple(value*scale for value in box) for box in boxes], small_pose)
        if estimated is None:
            return None
        matrix = np.linalg.inv(scaling) @ estimated @ scaling
        return matrix if self.validate(matrix, previous.shape) else None

    def _background_scaled(self, previous, current, boxes, pose):
        mask = np.full(previous.shape, 255, np.uint8)
        for x, y, width, height in boxes:
            if not np.all(np.isfinite((x, y, width, height))):
                continue
            margin = max(6, int(.15*max(width, height)))
            cv2.rectangle(mask, (max(0, int(x)-margin), max(0, int(y)-margin)),
                          (min(mask.shape[1]-1, int(x+width)+margin),
                           min(mask.shape[0]-1, int(y+height)+margin)), 0, -1)
        points = cv2.goodFeaturesToTrack(previous, self.max_features, .02, 8, mask=mask)
        if points is None or len(points) < self.minimum_inliers:
            return None
        try:
            initial = cv2.perspectiveTransform(points, pose) if pose is not None else None
            moved, ok, _ = cv2.calcOpticalFlowPyrLK(previous, current, points, initial,
                winSize=(21, 21), maxLevel=3,
                flags=cv2.OPTFLOW_USE_INITIAL_FLOW if initial is not None else 0)
            if moved is None:
                return None
            back, reverse, _ = cv2.calcOpticalFlowPyrLK(current, previous, moved, None,
                winSize=(21, 21), maxLevel=3)
            if back is None:
                return None
            good = (ok.ravel() > 0) & (reverse.ravel() > 0)
            good &= np.linalg.norm((back-points).reshape(-1, 2), axis=1) <= 1.5
            good &= np.isfinite(moved.reshape(-1, 2)).all(axis=1)
            before, after = points.reshape(-1, 2)[good], moved.reshape(-1, 2)[good]
            if len(before) < self.minimum_inliers:
                return None
            affine, inliers = cv2.estimateAffinePartial2D(before, after,
                method=cv2.RANSAC, ransacReprojThreshold=2., maxIters=500, confidence=.99)
            if affine is None or inliers is None:
                return None
            inliers = inliers.ravel().astype(bool)
            self.inlier_ratio = float(np.mean(inliers))
            if np.count_nonzero(inliers) < self.minimum_inliers or self.inlier_ratio < .65:
                return None
            extent = np.ptp(before[inliers], axis=0)
            if extent[0] < previous.shape[1]*.25 or extent[1] < previous.shape[0]*.25:
                return None
            matrix = np.vstack((affine, [0., 0., 1.]))
            return matrix if self.validate(matrix, previous.shape) else None
        except cv2.error:
            return None

    def between(self, earlier, later):
        if earlier == later:
            return np.eye(3)
        matrix, cursor = np.eye(3), earlier
        for start, end, transform in self.history:
            if start == cursor:
                if transform is None:
                    return None
                matrix, cursor = transform @ matrix, end
                if cursor == later:
                    return matrix
        return None
