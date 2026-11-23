"""Capture-local square crops in original image coordinates."""
import math
import cv2
import numpy as np


def crop_focused_roi(image, pred_box_xywh, scale_factor=3., min_crop_size=256,
                     motion_margin_px=0., size_bins=(), reflect_padding=False):
    x, y, width, height = pred_box_xywh
    if not np.isfinite((x, y, width, height, scale_factor)).all() or min(width, height) <= 0:
        raise ValueError("Focused ROI requires a finite positive box")
    if not 2.5 <= scale_factor <= 3.5 or min_crop_size < 1:
        raise ValueError("Invalid focused ROI configuration")
    image_height, image_width = image.shape[:2]
    if not math.isfinite(motion_margin_px) or motion_margin_px<0:
        raise ValueError('Invalid motion margin')
    side=max(min_crop_size,math.ceil(max(width,height)*scale_factor+2*motion_margin_px))
    if size_bins:
        side=next((value for value in sorted(size_bins) if value>=side),max(side,image_width))
    if reflect_padding:
        left = round(x + width / 2 - side / 2)
        top = round(y + height / 2 - side / 2)
        right, bottom = left + side, top + side
        if right <= 0 or bottom <= 0 or left >= image_width or top >= image_height:
            raise ValueError("Focused ROI does not intersect the image")
        crop = image[max(0, top):min(image_height, bottom),
                     max(0, left):min(image_width, right)]
        crop = cv2.copyMakeBorder(crop, max(0, -top), max(0, bottom-image_height),
                                 max(0, -left), max(0, right-image_width),
                                 cv2.BORDER_REFLECT_101)
        return crop, (left, top)
    side=min(side,image_width,image_height)
    left = max(0, min(round(x + width / 2 - side / 2), image_width - side))
    top = max(0, min(round(y + height / 2 - side / 2), image_height - side))
    return image[top:top + side, left:left + side], (left, top)


def reid_motion_corridor(box, velocity, age, shape, acceleration_stddev=140.):
    """Bounded search prior for suspended identity; not an observed box."""
    values=list(box)+list(velocity)+[age,acceleration_stddev]
    if not all(math.isfinite(float(v)) for v in values) or age<0 or acceleration_stddev<0:
        raise ValueError('Invalid Re-ID corridor motion')
    center=np.asarray(box[:2],dtype=float)+np.asarray(box[2:],dtype=float)/2+np.asarray(velocity,dtype=float)*age
    radius=min(max(shape[:2]),12.+.5*acceleration_stddev*age*age)
    return center,float(radius)
