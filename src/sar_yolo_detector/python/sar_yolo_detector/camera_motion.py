"""Validated image-plane camera compensation, never a 3-D translation claim."""
from collections import deque
import math
import threading

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


def transport_box(box, matrix):
    """Transport a tracking window without recursive enclosing-AABB growth.

    Repeatedly enclosing four rotated corners inflates an AABB even for pure
    camera roll. Transport its centre and local differential stretch instead;
    preserve the projected patch area and aspect stretch, removing in-plane
    rotation. This is a template/search-window prior, never a measurement.
    """
    x,y,w,h=map(float,box);u,v=x+w/2,y+h/2
    m=np.asarray(matrix,dtype=float)
    p=m @ np.array([u,v,1.])
    if not np.isfinite(p).all() or p[2]<=.2:return None
    c=p[:2]/p[2]
    j=(m[:2,:2]-np.outer(c,m[2,:2]))/p[2]
    sx,sy=np.linalg.norm(j[:,0]),np.linalg.norm(j[:,1])
    area=abs(np.linalg.det(j))
    if not np.isfinite([sx,sy,area]).all() or min(sx,sy,area)<=1e-9:return None
    factor=math.sqrt(area/(sx*sy));nw,nh=w*sx*factor,h*sy*factor
    return float(c[0]-nw/2),float(c[1]-nh/2),float(nw),float(nh)


def quaternion_rotation(values):
    x, y, z, w = np.asarray(values, dtype=float)
    norm = np.linalg.norm((x, y, z, w))
    if not np.isfinite(norm) or not .9 <= norm <= 1.1:
        raise ValueError("Invalid camera quaternion")
    x, y, z, w = np.asarray((x, y, z, w))/norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rotation_homography(intrinsics, previous_rotation, current_rotation, current_intrinsics=None):
    matrix = np.asarray(intrinsics, dtype=float).reshape(3, 3)
    if not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("Invalid camera intrinsics")
    current_matrix=matrix if current_intrinsics is None else np.asarray(current_intrinsics,dtype=float).reshape(3,3)
    if not np.isfinite(current_matrix).all() or current_matrix[0,0]<=0 or current_matrix[1,1]<=0:
        raise ValueError("Invalid current camera intrinsics")
    # Both rotations map camera optical coordinates into the same fixed frame.
    result = current_matrix @ current_rotation.T @ previous_rotation @ np.linalg.inv(matrix)
    return result/result[2, 2]


class AngularRateHistory:
    """Timestamped camera-optical IMU rates in radians per second."""
    def __init__(self, maximum_gap=.05):
        self.maximum_gap = float(maximum_gap)
        if not math.isfinite(self.maximum_gap) or self.maximum_gap <= 0:
            raise ValueError("Invalid angular-rate sample gap")
        self.samples = deque(maxlen=512)
        self.lock = threading.Lock()

    def add(self, stamp, rate, frame_id):
        rate = np.asarray(rate, dtype=float)
        if rate.shape != (3,) or not np.isfinite(rate).all() or not math.isfinite(stamp) or stamp <= 0 or not frame_id:
            return False
        with self.lock:
            if self.samples and stamp <= self.samples[-1][0]:
                return False
            self.samples.append((stamp, rate.copy(), frame_id))
        return True

    def homography(self, earlier, later, intrinsics, frame_id):
        if not 0 < later-earlier <= .25:
            return None
        with self.lock:
            samples = list(self.samples)
        before = [item for item in samples if item[0] <= earlier]
        if not before:
            return None
        current = before[-1]
        if earlier-current[0] > self.maximum_gap:
            return None
        rotation = np.eye(3)
        cursor = earlier
        for following in [item for item in samples if earlier < item[0] < later] + [(later, None, frame_id)]:
            end = following[0]
            if current[2] != frame_id or end-current[0] > self.maximum_gap:
                return None
            # Positive camera rotation transports stationary rays by R^-1.
            delta, _ = cv2.Rodrigues(-current[1] * (end-cursor))
            rotation = delta @ rotation
            cursor, current = end, following
        k = np.asarray(intrinsics, dtype=float).reshape(3,3)
        if not np.isfinite(k).all() or k[0,0] <= 0 or k[1,1] <= 0 or abs(np.linalg.det(k)) < 1e-9:
            return None
        matrix = k @ rotation @ np.linalg.inv(k)
        return matrix / matrix[2,2]


class CameraMotion:
    def __init__(self, config=None):
        config = dict(config or {})
        self.enabled = bool(config.get("enabled", True))
        self.max_gap = float(config.get("max_gap_sec", .75))
        self.max_features = int(config.get("max_features", 160))
        self.minimum_inliers = int(config.get("minimum_inliers", 12))
        self.processing_width=int(config.get('processing_width',320))
        self.lk_window_size=int(config.get('lk_window_size',21))
        self.lk_pyramid_levels=int(config.get('lk_pyramid_levels',3))
        if (not 80<=self.processing_width<=640 or not 5<=self.lk_window_size<=31
                or self.lk_window_size%2!=1 or not 0<=self.lk_pyramid_levels<=4):
            raise ValueError('Invalid camera-motion processing bounds')
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
        self.calibration_history = deque(maxlen=32)
        self.calibration_changed = False
        self.background_valid=False

    def advance_imu(self,stamp,rate_rotation,shape):
        """Advance the capture chain using gyro-only motion, without image work."""
        previous_stamp=self.stamp
        matrix=rate_rotation if self.enabled and self.validate(rate_rotation,shape) else None
        self.gray=None
        self.calibration_changed=False
        self.background_valid=False
        self.inlier_ratio=1. if matrix is not None else 0.
        self.source='optical_imu_rate' if matrix is not None else 'unavailable'
        if previous_stamp>0 and 0<stamp-previous_stamp<=self.max_gap:
            self.history.append((previous_stamp,stamp,matrix))
            self.calibration_history.append((previous_stamp,stamp,False))
        else:
            self.history.clear();self.calibration_history.clear()
        self.stamp=float(stamp)
        return matrix

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

    def update(self, frame, stamp, excluded_boxes=(), pose_rotation=None, estimate=True, calibration_changed=False, pure_rotation=False, estimate_background=True, rate_rotation=None):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        matrix, self.source, self.inlier_ratio = None, "unavailable", 0.
        previous, previous_stamp = self.gray, self.stamp
        self.calibration_changed=False;self.background_valid=False
        if self.enabled and estimate and previous is not None and previous.shape == gray.shape and 0 < stamp-previous_stamp <= self.max_gap:
            pose = pose_rotation if self.validate(pose_rotation, gray.shape) else None
            matrix = (self._background(previous, gray, excluded_boxes, pose)
                      if estimate_background or not (pure_rotation and pose is not None) else None)
            self.background_valid=matrix is not None
            if pure_rotation and pose is not None:
                # With equal measured optical centres, K/R exactly describe
                # every scene depth, including when background work is omitted.
                matrix,self.source=pose,"calibrated_pose"
            elif matrix is not None:
                self.source = "pose_seeded_background" if pose is not None else "background"
            elif pose is not None:
                matrix, self.source = pose, "pose_rotation_only"
            if matrix is None and self.validate(rate_rotation, gray.shape):
                matrix, self.source = rate_rotation, "optical_imu_rate"
            self.calibration_changed=bool(calibration_changed and matrix is not None)
            self.history.append((previous_stamp, stamp, matrix))
            self.calibration_history.append((previous_stamp,stamp,self.calibration_changed))
        else:
            self.history.clear()
            self.calibration_history.clear()
        self.gray, self.stamp = gray, stamp
        return matrix

    def _background(self, previous, current, boxes, pose):
        # Background GMC does not need full sensor resolution. Keep the target
        # images untouched; conjugate the estimated transform back to pixels.
        scale = min(1., self.processing_width/previous.shape[1])
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
                winSize=(self.lk_window_size, self.lk_window_size), maxLevel=self.lk_pyramid_levels,
                flags=cv2.OPTFLOW_USE_INITIAL_FLOW if initial is not None else 0)
            if moved is None:
                return None
            back, reverse, _ = cv2.calcOpticalFlowPyrLK(current, previous, moved, None,
                winSize=(self.lk_window_size, self.lk_window_size), maxLevel=self.lk_pyramid_levels)
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

    def calibration_between(self,earlier,later):
        if earlier==later:return False
        changed=False;cursor=earlier
        for start,end,value in self.calibration_history:
            if start==cursor:
                changed|=value;cursor=end
                if cursor==later:return changed
        return False


def transform_window(box,matrix,corners=None):
    """Carry persistent pixel support; enclose it only for the current window."""
    x,y,w,h=box
    source=np.float64([[x,y],[x+w,y],[x+w,y+h],[x,y+h]]) if corners is None else np.asarray(corners,np.float64)
    projected=cv2.perspectiveTransform(source.reshape(-1,1,2),np.asarray(matrix,np.float64)).reshape(-1,2)
    if not np.isfinite(projected).all():return None,None
    left,top=projected.min(axis=0);right,bottom=projected.max(axis=0)
    return (float(left),float(top),float(right-left),float(bottom-top)),projected


def correct_window_support(corners,old_box,new_box):
    if corners is None:return transform_window(new_box,np.eye(3))[1]
    x,y,w,h=old_box;nx,ny,nw,nh=new_box
    sx,sy=nw/max(1e-9,w),nh/max(1e-9,h)
    matrix=np.array([[sx,0,nx-sx*x],[0,sy,ny-sy*y],[0,0,1.]])
    return cv2.perspectiveTransform(np.asarray(corners,np.float64).reshape(-1,1,2),matrix).reshape(-1,2)
