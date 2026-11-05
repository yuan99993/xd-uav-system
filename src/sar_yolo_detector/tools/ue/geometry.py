"""Pure NED route/segmentation helpers, independent of Unreal and ROS."""
import math
import numpy as np


def unreal_camera_optical_rotation(rotation_degrees):
    """Map optical x-right/y-down/z-forward into the UE render NED frame.

    Use UE5 FRotationTranslationMatrix's Rotator basis, then reflect world
    Z-up into Z-down. Only the measured camera pose is used, never target truth.
    """
    angles=[float(rotation_degrees.get(name,0.)) for name in ('pitch','yaw','roll')]
    if not np.isfinite(angles).all():
        raise ValueError('Camera rotation must be finite')
    pitch,yaw,roll=map(math.radians,angles)
    sp,sy,sr=math.sin(pitch),math.sin(yaw),math.sin(roll)
    cp,cy,cr=math.cos(pitch),math.cos(yaw),math.cos(roll)
    forward=np.array([cp*cy,cp*sy,sp])
    right=np.array([sr*sp*cy-cr*sy,sr*sp*sy+cr*cy,-sr*cp])
    up=np.array([-(cr*sp*cy+sr*sy),cy*sr-cr*sp*sy,cr*cp])
    rotation=np.column_stack((right,-up,forward))
    rotation[2,:]*=-1.
    return rotation


def airsim_bgr_to_rgb(raw, width, height):
    # Colosseum RenderRequest.cpp emits item.B, item.G, item.R for raw frames.
    values=np.frombuffer(raw,dtype=np.uint8)
    if width<=0 or height<=0 or values.size!=width*height*3:
        raise ValueError("Invalid uncompressed camera capture")
    return np.ascontiguousarray(values.reshape(height,width,3)[:,:,::-1])


def route_pose(points, distance):
    lengths = [math.dist(a,b) for a,b in zip(points,points[1:])]
    total=sum(lengths)
    if total <= 0:
        raise ValueError("Route must have nonzero length")
    distance %= total
    for a,b,length in zip(points,points[1:],lengths):
        if length and distance <= length:
            f=distance/length
            return [x+(y-x)*f for x,y in zip(a,b)], math.atan2(b[1]-a[1],b[0]-a[0])
        distance-=length
    raise ValueError("Invalid route")


def segmentation_boxes(rgb, palette, entities, minimum_pixels=4):
    boxes=[]
    for index,e in enumerate(entities,1):
        mask=np.all(rgb==np.asarray(palette[index],dtype=np.uint8),axis=2)
        y,x=np.nonzero(mask)
        if len(x)<minimum_pixels:
            continue
        x1,y1,x2,y2=int(x.min()),int(y.min()),int(x.max())+1,int(y.max())+1
        boxes.append({"entity_id":e["id"],"class":e["class"],"segmentation_id":index,
                      "xyxy":[x1,y1,x2,y2],"visible_pixels":len(x),
                      "box_semantics":"visible_segmentation_extent_not_amodal"})
    return boxes
