#!/usr/bin/env python3
"""Publish repeatable obstacle surfaces for the EGO avoidance demos."""

import math
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def samples(start, stop, step):
    count = max(1, int(math.ceil((stop - start) / step)))
    spacing = (stop - start) / count
    return [start + index * spacing for index in range(count + 1)]


def vector3(entry, key):
    value = entry.get(key)
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("{} must be a three-element list".format(key))
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError("{} must contain finite values".format(key))
    return result


def box_surface(center, size, resolution):
    if any(length <= 0.0 for length in size):
        raise ValueError("box size values must be positive")
    axes = [samples(center[index] - size[index] / 2.0,
                    center[index] + size[index] / 2.0,
                    resolution)
            for index in range(3)]
    points = set()
    for x in (axes[0][0], axes[0][-1]):
        points.update((x, y, z) for y in axes[1] for z in axes[2])
    for y in (axes[1][0], axes[1][-1]):
        points.update((x, y, z) for x in axes[0] for z in axes[2])
    for z in (axes[2][0], axes[2][-1]):
        points.update((x, y, z) for x in axes[0] for y in axes[1])
    return points


def cylinder_surface(center, radius, height, resolution):
    if radius <= 0.0 or height <= 0.0:
        raise ValueError("cylinder radius and height must be positive")
    z_values = samples(center[2] - height / 2.0,
                       center[2] + height / 2.0, resolution)
    angular_count = max(12, int(math.ceil(2.0 * math.pi * radius / resolution)))
    points = set()
    for index in range(angular_count):
        angle = 2.0 * math.pi * index / angular_count
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        points.update((x, y, z) for z in z_values)
    radial_values = samples(0.0, radius, resolution)
    for z in (z_values[0], z_values[-1]):
        points.add((center[0], center[1], z))
        for radial in radial_values[1:]:
            ring_count = max(6, int(math.ceil(2.0 * math.pi * radial / resolution)))
            for index in range(ring_count):
                angle = 2.0 * math.pi * index / ring_count
                points.add((center[0] + radial * math.cos(angle),
                            center[1] + radial * math.sin(angle), z))
    return points


def configured_points(resolution):
    boxes = rospy.get_param("~boxes", [])
    cylinders = rospy.get_param("~cylinders", [])
    if not isinstance(boxes, list) or not isinstance(cylinders, list):
        raise ValueError("boxes and cylinders must be lists")
    if not boxes and not cylinders:
        wall_x = float(rospy.get_param("~wall_x", 3.0))
        half_width = float(rospy.get_param("~half_width", 1.0))
        height = float(rospy.get_param("~height", 2.2))
        thickness = float(rospy.get_param("~thickness", 0.30))
        return box_surface(
            (wall_x, 0.0, height / 2.0),
            (thickness, 2.0 * half_width, height), resolution)
    points = set()
    for entry in boxes:
        if not isinstance(entry, dict):
            raise ValueError("each box must be a mapping")
        points.update(box_surface(
            vector3(entry, "center"), vector3(entry, "size"), resolution))
    for entry in cylinders:
        if not isinstance(entry, dict):
            raise ValueError("each cylinder must be a mapping")
        points.update(cylinder_surface(
            vector3(entry, "center"), float(entry.get("radius", 0.0)),
            float(entry.get("height", 0.0)), resolution))
    return points


def main():
    rospy.init_node("deterministic_obstacle_cloud")
    frame = rospy.get_param("~frame_id", "world")
    resolution = float(rospy.get_param("~resolution", 0.10))
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("resolution must be a positive finite value")
    topic = rospy.get_param("~output_topic", "/map_generator/global_cloud")
    publisher = rospy.Publisher(topic, PointCloud2, queue_size=1, latch=True)
    points = sorted(configured_points(resolution))
    rospy.loginfo("publishing %d deterministic obstacle surface points", len(points))
    rate = rospy.Rate(float(rospy.get_param("~rate", 2.0)))
    try:
        while not rospy.is_shutdown():
            header = Header(stamp=rospy.Time.now(), frame_id=frame)
            publisher.publish(point_cloud2.create_cloud_xyz32(header, points))
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
