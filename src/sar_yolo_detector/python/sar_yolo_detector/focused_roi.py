"""Capture-local square crops in original image coordinates."""
import math
import numpy as np


def crop_focused_roi(image, pred_box_xywh, scale_factor=3., min_crop_size=256,
                     motion_margin_px=0., size_bins=()):
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
    side=min(side,image_width,image_height)
    left = max(0, min(round(x + width / 2 - side / 2), image_width - side))
    top = max(0, min(round(y + height / 2 - side / 2), image_height - side))
    return image[top:top + side, left:left + side], (left, top)
