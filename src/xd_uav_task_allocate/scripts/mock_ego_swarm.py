#!/usr/bin/env python3
"""Task-layer-only EGO-Swarm status simulator; it publishes no flight control."""

import rospy
from geometry_msgs.msg import PoseStamped

from xd_uav_task_allocate.msg import PlannerStatus


class MockEgoSwarm:
    def __init__(self):
        self.uav_name = str(rospy.get_param("~uav_name", "uav1"))
        self.reach_delay = max(0.0, float(rospy.get_param("~reach_delay_sec", 0.5)))
        self.auto_reach = bool(rospy.get_param("~auto_reach", True))
        goal_topic = str(rospy.get_param("~goal_topic", f"/{self.uav_name}/planning/goal"))
        status_topic = str(
            rospy.get_param("~status_topic", f"/{self.uav_name}/planning/status")
        )
        self.publisher = rospy.Publisher(status_topic, PlannerStatus, queue_size=5)
        self.subscriber = rospy.Subscriber(goal_topic, PoseStamped, self.goal_callback, queue_size=5)
        self.timer = None
        self.latest_goal_id = 0
        rospy.logwarn(
            "[mock_ego_swarm] %s task-interface simulator active; NO control output",
            self.uav_name,
        )

    def publish_status(self, goal_id, state, detail):
        message = PlannerStatus()
        message.header.stamp = rospy.Time.now()
        message.goal_id = int(goal_id)
        message.state = int(state)
        message.detail = str(detail)
        self.publisher.publish(message)

    def goal_callback(self, message):
        self.latest_goal_id = int(message.header.seq)
        if self.timer is not None:
            self.timer.shutdown()
        self.publish_status(self.latest_goal_id, PlannerStatus.PLANNING, "mock planning")
        self.publish_status(self.latest_goal_id, PlannerStatus.ACTIVE, "mock trajectory active")
        if self.auto_reach:
            self.timer = rospy.Timer(
                rospy.Duration(self.reach_delay),
                lambda _event, goal_id=self.latest_goal_id: self.reached(goal_id),
                oneshot=True,
            )

    def reached(self, goal_id):
        if int(goal_id) != self.latest_goal_id:
            return
        self.publish_status(goal_id, PlannerStatus.REACHED, "mock goal reached")


def main():
    rospy.init_node("mock_ego_swarm")
    MockEgoSwarm()
    rospy.spin()


if __name__ == "__main__":
    main()
