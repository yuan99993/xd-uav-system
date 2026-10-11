#!/usr/bin/env python3
"""M4 no-flight acceptance with two independent real terrain planners."""

import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

from xd_uav_planning.msg import TerrainConflict, TerrainSwarmStatus


class TerrainFixedwingSwarmTest(unittest.TestCase):
    START = (-2012.4, -1188.0, 2340.246826)
    GOAL = (2012.4, 1188.0, 1676.894348)
    FIRST_GOAL_ID = 5201
    SECOND_GOAL_ID = 5202

    def setUp(self):
        self.first_paths = []
        self.second_paths = []
        self.swarm_statuses = []
        self.markers = []
        self.first_publisher = rospy.Publisher(
            "/uav1/planning/task_path", Path, queue_size=2)
        self.second_publisher = rospy.Publisher(
            "/uav2/planning/task_path", Path, queue_size=2)
        self.subscribers = [
            rospy.Subscriber(
                "/uav1/planning/terrain_path", Path,
                self.first_paths.append, queue_size=2),
            rospy.Subscriber(
                "/uav2/planning/terrain_path", Path,
                self.second_paths.append, queue_size=2),
            rospy.Subscriber(
                "/terrain_swarm/status", TerrainSwarmStatus,
                self.swarm_statuses.append, queue_size=10),
            rospy.Subscriber(
                "/terrain_swarm/markers", MarkerArray,
                self.markers.append, queue_size=5),
        ]

    @staticmethod
    def _wait_for(predicate, timeout=30.0, detail="condition"):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.02)
        raise AssertionError("timed out waiting for {}".format(detail))

    @classmethod
    def _request(cls, goal_id):
        output = Path()
        output.header.stamp = rospy.Time.now()
        output.header.frame_id = "world"
        output.header.seq = goal_id
        for point in (cls.START, cls.GOAL):
            pose = PoseStamped()
            pose.header.seq = goal_id
            pose.header.stamp = output.header.stamp
            pose.header.frame_id = "world"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            pose.pose.orientation.w = 1.0
            output.poses.append(pose)
        return output

    @staticmethod
    def _path_goal_id(message):
        return int(message.poses[0].header.seq) if message.poses else 0

    def test_independent_planners_and_unified_conflict_result(self):
        self._wait_for(
            lambda: self.first_publisher.get_num_connections() == 1
            and self.second_publisher.get_num_connections() == 1,
            detail="both namespaced planner subscribers")
        self._wait_for(
            lambda: any(
                not status.analysis_ready
                and status.detail.startswith("not_ready:")
                and {vehicle.uav_name for vehicle in status.vehicles}
                == {"uav1", "uav2"}
                for status in self.swarm_statuses),
            detail="complete fail-closed pre-plan swarm status")

        self.first_publisher.publish(self._request(self.FIRST_GOAL_ID))
        self.second_publisher.publish(self._request(self.SECOND_GOAL_ID))

        self._wait_for(
            lambda: any(
                self._path_goal_id(path) == self.FIRST_GOAL_ID
                and len(path.poses) > 2 for path in self.first_paths),
            detail="uav1 real terrain path")
        self._wait_for(
            lambda: any(
                self._path_goal_id(path) == self.SECOND_GOAL_ID
                and len(path.poses) > 2 for path in self.second_paths),
            detail="uav2 real terrain path")
        self.assertFalse(any(
            self._path_goal_id(path) == self.SECOND_GOAL_ID
            for path in self.first_paths))
        self.assertFalse(any(
            self._path_goal_id(path) == self.FIRST_GOAL_ID
            for path in self.second_paths))

        def accepted_status():
            for status in reversed(self.swarm_statuses):
                if (status.analysis_ready and not status.conflict_free
                        and len(status.vehicles) == 2
                        and len(status.conflicts) >= 1):
                    conflict = status.conflicts[0]
                    if (conflict.conflict_type == TerrainConflict.SPATIOTEMPORAL
                            and conflict.first_uav == "uav1"
                            and conflict.first_goal_id == self.FIRST_GOAL_ID
                            and conflict.second_uav == "uav2"
                            and conflict.second_goal_id == self.SECOND_GOAL_ID
                            and conflict.predicted_time_valid
                            and conflict.separation_m < conflict.threshold_m):
                        return status
            return None

        self._wait_for(accepted_status, detail="unified spatiotemporal conflict")
        status = accepted_status()
        by_name = {vehicle.uav_name: vehicle for vehicle in status.vehicles}
        self.assertEqual({"uav1", "uav2"}, set(by_name))
        self.assertEqual(self.FIRST_GOAL_ID,
                         by_name["uav1"].planner_status.goal_id)
        self.assertEqual(self.SECOND_GOAL_ID,
                         by_name["uav2"].planner_status.goal_id)
        self.assertTrue(by_name["uav1"].path_available)
        self.assertTrue(by_name["uav2"].path_available)
        self.assertGreater(by_name["uav1"].path_length_m, 0.0)
        self.assertGreater(by_name["uav2"].path_length_m, 0.0)
        self._wait_for(
            lambda: any(any(
                marker.ns == "terrain_swarm_conflicts"
                for marker in array.markers) for array in self.markers),
            detail="conflict visualization marker")


if __name__ == "__main__":
    rospy.init_node("terrain_fixedwing_swarm_test")
    rostest.rosrun(
        "xd_uav_planning", "terrain_fixedwing_swarm_test",
        TerrainFixedwingSwarmTest)
