#!/usr/bin/env python3
import rospy

from gm_control.msg import GimbalCommand


class TopicGimbalAdapterNode:
    """Placeholder adapter.

    This node deliberately only logs/republishes the command boundary. Replace this
    with a Gazebo, MAVLink, serial, UDP, or vendor-SDK adapter when the target
    gimbal interface is known.
    """

    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/gm_control/gimbal_cmd")
        self.log_every_n = rospy.get_param("~log_every_n", 30)
        self.count = 0
        self.sub = rospy.Subscriber(input_topic, GimbalCommand, self._command_callback, queue_size=10)
        rospy.loginfo("topic gimbal adapter placeholder listening on %s", input_topic)

    def _command_callback(self, msg):
        self.count += 1
        if self.log_every_n > 0 and self.count % self.log_every_n == 0:
            rospy.loginfo(
                "gimbal cmd valid=%s yaw_rate=%.2f pitch_rate=%.2f roll_rate=%.2f",
                msg.valid,
                msg.yaw_rate_deg_s,
                msg.pitch_rate_deg_s,
                msg.roll_rate_deg_s,
            )


def main():
    rospy.init_node("topic_gimbal_adapter")
    TopicGimbalAdapterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
