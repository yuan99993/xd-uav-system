cmake_minimum_required(VERSION 3.0.2)
project(pod_gateway)
find_package(catkin REQUIRED COMPONENTS diagnostic_msgs roscpp pod_msgs)
catkin_package(CATKIN_DEPENDS diagnostic_msgs roscpp pod_msgs)
