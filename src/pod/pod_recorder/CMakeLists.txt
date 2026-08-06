cmake_minimum_required(VERSION 3.0.2)
project(pod_recorder)
find_package(catkin REQUIRED COMPONENTS diagnostic_msgs rosbag roscpp sensor_msgs pod_msgs)
catkin_package(CATKIN_DEPENDS diagnostic_msgs rosbag roscpp sensor_msgs pod_msgs)
