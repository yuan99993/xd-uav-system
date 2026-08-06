cmake_minimum_required(VERSION 3.0.2)
project(pod_video_pipeline)
find_package(catkin REQUIRED COMPONENTS diagnostic_msgs image_transport roscpp sensor_msgs pod_msgs)
catkin_package(CATKIN_DEPENDS diagnostic_msgs image_transport roscpp sensor_msgs pod_msgs)
