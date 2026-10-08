#!/usr/bin/env python3
"""M6 no-flight gate for three real Davosdorf terrain plans."""

import math
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from xd_uav_task_allocate.msg import PlannerStatus
from xd_uav_planning.msg import TerrainSwarmStatus


ROUTES = {
    "uav1": (6201, (600.0, -2000.0, 1622.6483154296875),
             (1200.0, -2000.0, 1629.913818359375)),
    "uav2": (6202, (600.0, -1700.0, 1644.424560546875),
             (1200.0, -1700.0, 1632.3648681640625)),
    "uav3": (6203, (600.0, -1400.0, 1651.70556640625),
             (1200.0, -1400.0, 1632.1815185546875)),
}


class TerrainFixedwingSwarmThreeTest(unittest.TestCase):
    def setUp(self):
        self.paths = {name: [] for name in ROUTES}
        self.planner_statuses = {name: [] for name in ROUTES}
        self.statuses = []
        self.publishers = {
            name: rospy.Publisher(
                "/{}/planning/task_path".format(name), Path, queue_size=2)
            for name in ROUTES
        }
        self.subscribers = [rospy.Subscriber(
            "/terrain_swarm/status", TerrainSwarmStatus,
            self.statuses.append, queue_size=10)]
        for name in ROUTES:
            self.subscribers.append(rospy.Subscriber(
                "/{}/planning/terrain_path".format(name), Path,
                lambda message, key=name: self.paths[key].append(message),
                queue_size=2))
            self.subscribers.append(rospy.Subscriber(
                "/{}/planning/terrain_status".format(name), PlannerStatus,
                lambda message, key=name:
                self.planner_statuses[key].append(message), queue_size=10))

    @staticmethod
    def _wait(predicate, timeout=60.0, detail="condition"):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.02)
        raise AssertionError("timed out waiting for " + detail)

    @staticmethod
    def _request(goal_id, start, finish):
        output = Path()
        output.header.stamp = rospy.Time.now()
        output.header.frame_id = "world"
        output.header.seq = goal_id
        for point in (start, finish):
            pose = PoseStamped()
            pose.header.seq = goal_id
            pose.header.stamp = output.header.stamp
            pose.header.frame_id = "world"
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
            pose.pose.orientation.w = 1.0
            output.poses.append(pose)
        return output

    @staticmethod
    def _goal(path):
        return int(path.poses[0].header.seq) if path.poses else 0

    def test_three_paths_are_independent_and_conflict_free(self):
        self._wait(
            lambda: all(pub.get_num_connections() == 1
                        for pub in self.publishers.values()),
            detail="three terrain planner subscribers")
        for name, (goal_id, start, finish) in ROUTES.items():
            self.publishers[name].publish(self._request(goal_id, start, finish))

        def path_ready(name):
            return any(self._goal(path) == ROUTES[name][0]
                       and len(path.poses) > 2
                       for path in self.paths[name])

        terminal_states = (PlannerStatus.FAILED, PlannerStatus.BLOCKED)
        self._wait(
            lambda: all(
                path_ready(name)
                or (self.planner_statuses[name]
                    and self.planner_statuses[name][-1].state
                    in terminal_states)
                for name in ROUTES),
            timeout=20.0,
            detail="three terrain planner outcomes")
        if not all(path_ready(name) for name in ROUTES):
            latest = {
                name: (messages[-1].state, messages[-1].goal_id,
                       messages[-1].detail)
                if messages else None
                for name, messages in self.planner_statuses.items()
            }
            counts = {name: len(messages)
                      for name, messages in self.paths.items()}
            raise AssertionError(
                "terrain path failure; path_counts={}; "
                "planner_statuses={}".format(counts, latest))

        def ready():
            for status in reversed(self.statuses):
                if (status.analysis_ready and status.conflict_free
                        and len(status.vehicles) == 3
                        and not status.conflicts):
                    goals = {item.uav_name: int(item.planner_status.goal_id)
                             for item in status.vehicles}
                    if all(goals.get(name) == route[0]
                           for name, route in ROUTES.items()):
                        return status
            return None

        self._wait(ready, detail="three-vehicle conflict-free screen")
        status = ready()
        self.assertEqual(set(ROUTES),
                         {item.uav_name for item in status.vehicles})
        for name in ROUTES:
            own_goal = ROUTES[name][0]
            own_path = next(path for path in self.paths[name]
                            if self._goal(path) == own_goal)
            finish = ROUTES[name][2]
            endpoint = own_path.poses[-1].pose.position
            endpoint_error = math.sqrt(
                (endpoint.x - finish[0]) ** 2
                + (endpoint.y - finish[1]) ** 2
                + (endpoint.z - finish[2]) ** 2)
            self.assertLessEqual(endpoint_error, 1e-6)
            self.assertFalse(any(
                self._goal(path) in {route[0] for other, route in ROUTES.items()
                                     if other != name}
                for path in self.paths[name]))


if __name__ == "__main__":
    rospy.init_node("terrain_fixedwing_swarm_three_test")
    rostest.rosrun(
        "xd_uav_planning", "terrain_fixedwing_swarm_three_test",
        TerrainFixedwingSwarmThreeTest)
