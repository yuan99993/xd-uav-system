#!/usr/bin/env python3
"""
sead_onboard 启动验证 — 用 mock mavros topic 满足 Drone.__init__ 依赖
无需 PX4 SITL。

用法:
  1. roscore &
  2. python3 test_sead_onboard_startup.py
  3. 检查输出确认 rosbridge 话题就绪
"""
import rospy
from mavros_msgs.msg import State, PositionTarget, HomePosition
from sensor_msgs.msg import Imu, BatteryState, NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Quaternion
import time, sys


def publish_mock_mavros(uav_name="uav1"):
    """发布模拟 mavros 数据满足 Drone.__init__ 订阅需求."""
    ns = f"/{uav_name}/mavros"
    rospy.init_node("mock_mavros", anonymous=True)

    # 1. State (必须: armed + mode)
    state_pub = rospy.Publisher(f"{ns}/state", State, queue_size=5)
    state = State()
    state.armed = False
    state.mode = "AUTO.LOITER"
    state.connected = True

    # 2. Imu
    imu_pub = rospy.Publisher(f"{ns}/imu/data", Imu, queue_size=5)
    imu = Imu()
    imu.header = Header(frame_id=f"{uav_name}/imu_link")
    imu.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

    # 3. Odometry (local_position)
    odom_pub = rospy.Publisher(f"{ns}/local_position/odom", Odometry, queue_size=5)
    odom = Odometry()
    odom.header = Header(frame_id=f"{uav_name}/odom")
    odom.pose.pose.position.x = 0.0
    odom.pose.pose.position.y = 0.0
    odom.pose.pose.position.z = 0.0

    # 4. GPS
    gps_pub = rospy.Publisher(f"{ns}/global_position/global", NavSatFix, queue_size=5)
    gps = NavSatFix()
    gps.latitude = 47.397742
    gps.longitude = 8.545594
    gps.altitude = 500.0

    # 5. Battery
    bat_pub = rospy.Publisher(f"{ns}/battery", BatteryState, queue_size=5)
    bat = BatteryState()
    bat.voltage = 25.2
    bat.percentage = 0.95

    # 6. Home
    home_pub = rospy.Publisher(f"{ns}/home_position/home", HomePosition, queue_size=5)
    home = HomePosition()
    home.geo.latitude = 47.397742
    home.geo.longitude = 8.545594
    home.geo.altitude = 500.0

    rate = rospy.Rate(10)
    rospy.loginfo(f"[mock_mavros] Publishing mock states on /{uav_name}/mavros/* ...")
    while not rospy.is_shutdown():
        imu.header.stamp = rospy.Time.now()
        odom.header.stamp = rospy.Time.now()
        state.header.stamp = rospy.Time.now()
        state_pub.publish(state)
        imu_pub.publish(imu)
        odom_pub.publish(odom)
        gps_pub.publish(gps)
        bat_pub.publish(bat)
        home_pub.publish(home)
        rate.sleep()


if __name__ == "__main__":
    uav_name = sys.argv[1] if len(sys.argv) > 1 else "uav1"
    publish_mock_mavros(uav_name)
