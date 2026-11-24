"""Capture-local square crops in original image coordinates."""
import math
import cv2
import numpy as np


def crop_focused_roi(image, pred_box_xywh, scale_factor=3., min_crop_size=256,
                     motion_margin_px=0., size_bins=(), reflect_padding=False,
                     return_mask=False, pad_value=114):
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
    # Keep the crop centered on the selected target at the image boundary.
    # The legacy reflect_padding flag is accepted for old callers, but never
    # mirrors scene pixels: padding is a neutral 114 gray and is marked invalid.
    left = round(x + width / 2 - side / 2)
    top = round(y + height / 2 - side / 2)
    right, bottom = left + side, top + side
    if right <= 0 or bottom <= 0 or left >= image_width or top >= image_height:
        raise ValueError("Focused ROI does not intersect the image")
    if left >= 0 and top >= 0 and right <= image_width and bottom <= image_height:
        crop = image[top:bottom, left:right]
        if return_mask:
            return crop, (left, top), np.full((side, side), 255, dtype=np.uint8)
        return crop, (left, top)
    src_left, src_top = max(0, left), max(0, top)
    src_right, src_bottom = min(image_width, right), min(image_height, bottom)
    crop = np.full((side, side) + image.shape[2:], pad_value, dtype=image.dtype)
    mask = np.zeros((side, side), dtype=np.uint8)
    dst_left, dst_top = src_left - left, src_top - top
    dst_right, dst_bottom = dst_left + src_right - src_left, dst_top + src_bottom - src_top
    crop[dst_top:dst_bottom, dst_left:dst_right] = image[src_top:src_bottom, src_left:src_right]
    mask[dst_top:dst_bottom, dst_left:dst_right] = 255
    if return_mask:
        return crop, (left, top), mask
    return crop, (left, top)


def reid_motion_corridor(box, velocity, age, shape, acceleration_stddev=140.):
    """Bounded search prior for suspended identity; not an observed box."""
    values=list(box)+list(velocity)+[age,acceleration_stddev]
    if not all(math.isfinite(float(v)) for v in values) or age<0 or acceleration_stddev<0:
        raise ValueError('Invalid Re-ID corridor motion')
    center=np.asarray(box[:2],dtype=float)+np.asarray(box[2:],dtype=float)/2+np.asarray(velocity,dtype=float)*age
    radius=min(max(shape[:2]),12.+.5*acceleration_stddev*age*age)
    return center,float(radius)
