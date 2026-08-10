#!/usr/bin/env python3
"""
test_external_input.py — 测试外部输入接口

测试脚本：通过 rostopic 向 Tracker 发送外部输入，
验证归一化误差输出是否正常。

用法:
  rosrun tracker test_external_input.py --hold-seconds 5
"""

import argparse
import rospy
import time
from tracker.msg import ExternalInput


def test_bounding_box(hold_seconds=4.0):
    """测试边界框输入。"""
    rospy.init_node('test_external_input', anonymous=True)
    pub = rospy.Publisher(
        '/tracker_node/external_input', ExternalInput, queue_size=10,
    )
    rospy.sleep(1.0)

    rate = rospy.Rate(10)

    # 模拟目标从左上角移动到中心
    positions = [
        (0.3, 0.3, 0.1, 0.1),   # 左上
        (0.35, 0.35, 0.1, 0.1),  # 略偏左上
        (0.4, 0.4, 0.1, 0.1),    # 接近中心
        (0.45, 0.45, 0.1, 0.1),  # 更接近中心
        (0.5, 0.5, 0.1, 0.1),    # 中心
    ]

    rospy.loginfo("Starting external input test...")

    # 启动跟踪
    msg = ExternalInput()
    msg.header.stamp = rospy.Time.now()
    msg.source = "bounding_box"
    msg.command = "start_track"
    msg.normalized_bbox = positions[0]
    msg.has_normalized_bbox = True
    msg.confidence = 0.9
    msg.class_id = 0
    pub.publish(msg)
    rospy.loginfo("Sent start_track command")
    rospy.sleep(0.5)

    # 发送连续的边界框更新
    for i, pos in enumerate(positions):
        if rospy.is_shutdown():
            break

        msg = ExternalInput()
        msg.header.stamp = rospy.Time.now()
        msg.source = "bounding_box"
        msg.command = ""
        msg.normalized_bbox = pos
        msg.has_normalized_bbox = True
        msg.confidence = 0.9 - i * 0.05
        msg.class_id = 0
        pub.publish(msg)

        rospy.loginfo("Sent bbox update %d: %s", i, pos)
        rate.sleep()

    # Keep publishing the final detection long enough to inspect the
    # controller response and velocity marker in RViz.
    hold_until = time.time() + max(0.0, hold_seconds)
    while not rospy.is_shutdown() and time.time() < hold_until:
        msg = ExternalInput()
        msg.header.stamp = rospy.Time.now()
        msg.source = "bounding_box"
        msg.normalized_bbox = positions[-1]
        msg.has_normalized_bbox = True
        msg.confidence = 0.7
        msg.class_id = 0
        pub.publish(msg)
        rate.sleep()
    if hold_seconds > 0.0:
        rospy.loginfo("Held final bbox for %.1f seconds", hold_seconds)

    # 停止跟踪
    msg = ExternalInput()
    msg.header.stamp = rospy.Time.now()
    msg.source = "bounding_box"
    msg.command = "stop_track"
    pub.publish(msg)
    rospy.loginfo("Sent stop_track command")

    rospy.loginfo("External input test completed.")


def test_feature_point():
    """测试特征点输入。"""
    rospy.init_node('test_feature_point_input', anonymous=True)
    pub = rospy.Publisher(
        '/tracker_node/external_input', ExternalInput, queue_size=10,
    )
    rospy.sleep(1.0)

    rate = rospy.Rate(15)

    msg = ExternalInput()
    msg.header.stamp = rospy.Time.now()
    msg.source = "feature_point"
    msg.command = "start_track"
    msg.feature_point = [0.5, 0.5]
    msg.has_feature_point = True
    msg.confidence = 0.95
    pub.publish(msg)
    rospy.loginfo("Sent feature point start_track")
    rospy.sleep(0.5)

    for _ in range(30):
        if rospy.is_shutdown():
            break
        msg = ExternalInput()
        msg.header.stamp = rospy.Time.now()
        msg.source = "feature_point"
        msg.command = ""
        msg.feature_point = [0.5, 0.5]
        msg.has_feature_point = True
        msg.confidence = 0.95
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("Feature point test completed.")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Send reproducible external tracker input.')
    parser.add_argument('mode', nargs='?', choices=('bbox', 'feature'), default='bbox')
    parser.add_argument(
        '--hold-seconds', type=float, default=4.0,
        help='How long to keep publishing the final bounding box before stop_track.',
    )
    args = parser.parse_args(rospy.myargv()[1:])
    if args.mode == 'feature':
        test_feature_point()
    else:
        test_bounding_box(args.hold_seconds)
