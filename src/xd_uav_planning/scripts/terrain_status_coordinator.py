#!/usr/bin/env python3
"""Serialize terrain and fixed-wing backend status onto the public topic."""

import rospy

from xd_uav_task_allocate.msg import PlannerStatus


class TerrainStatusCoordinator:
    def __init__(self):
        self._active_goal_id = None
        self._publisher = rospy.Publisher(
            rospy.get_param("~planner_status_topic", "planning/status"),
            PlannerStatus, queue_size=20, latch=True)
        self._terrain_subscriber = rospy.Subscriber(
            rospy.get_param(
                "~terrain_status_topic", "planning/terrain_status"),
            PlannerStatus, self._terrain_callback, queue_size=20)
        self._backend_subscriber = rospy.Subscriber(
            rospy.get_param(
                "~backend_status_topic", "planning/backend_status"),
            PlannerStatus, self._backend_callback, queue_size=20)

    def _terrain_callback(self, message):
        goal_id = int(message.goal_id)
        if goal_id == 0:
            if self._active_goal_id is None:
                self._publisher.publish(message)
            return
        self._active_goal_id = goal_id
        self._publisher.publish(message)

    def _backend_callback(self, message):
        goal_id = int(message.goal_id)
        if goal_id == 0:
            if self._active_goal_id is None:
                self._publisher.publish(message)
            return
        if goal_id == self._active_goal_id:
            self._publisher.publish(message)


def main():
    rospy.init_node("terrain_status_coordinator")
    TerrainStatusCoordinator()
    rospy.spin()


if __name__ == "__main__":
    main()
