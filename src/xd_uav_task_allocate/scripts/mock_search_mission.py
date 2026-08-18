#!/usr/bin/env python3
"""Publish search polygons for task-layer testing without a ground station."""

import json

import rospy
from geometry_msgs.msg import Point32

from xd_uav_task_allocate.msg import SearchArea, SearchAreaArray


def main():
    rospy.init_node("mock_search_mission")
    topic = str(rospy.get_param("~topic", "/task_allocate/search_areas"))
    frame_id = str(rospy.get_param("~frame_id", "world"))
    requested = rospy.get_param(
        "~areas_json",
        '[{"id":1,"polygon":[[0,0],[100,0],[100,60],[0,60]],'
        '"altitude":20,"lane_spacing":10,"priority":1}]',
    )
    areas = json.loads(requested) if isinstance(requested, str) else requested
    if not isinstance(areas, list):
        raise ValueError("areas_json must decode to a list of search areas")
    message = SearchAreaArray()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = frame_id
    for item in areas:
        area = SearchArea()
        area.area_id = int(item["id"])
        area.altitude = float(item.get("altitude", 20.0))
        area.lane_spacing = float(item.get("lane_spacing", 10.0))
        area.priority = int(item.get("priority", 0))
        area.boundary.points = [
            Point32(x=float(point[0]), y=float(point[1]), z=0.0)
            for point in item["polygon"]
        ]
        message.areas.append(area)
    publisher = rospy.Publisher(topic, SearchAreaArray, queue_size=1, latch=True)
    deadline = rospy.Time.now() + rospy.Duration(1.0)
    while not rospy.is_shutdown() and publisher.get_num_connections() == 0:
        if rospy.Time.now() >= deadline:
            break
        rospy.sleep(0.05)
    publisher.publish(message)
    rospy.loginfo("[mock_search_mission] published %d areas on %s", len(message.areas), topic)
    rospy.sleep(0.5)


if __name__ == "__main__":
    main()
