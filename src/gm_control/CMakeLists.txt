cmake_minimum_required(VERSION 3.0.2)
project(gm_control)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
  xd_uav_track
)

add_message_files(
  FILES
  BoundingBox2D.msg
  GimbalCommand.msg
  GimbalState.msg
)

add_service_files(
  FILES
  StartGimbalTracking.srv
  StartGimbalSearch.srv
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/GimbalPid.cfg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs message_runtime rospy sensor_msgs std_msgs xd_uav_track
)

catkin_install_python(
  PROGRAMS
    scripts/bbox_tracker_node.py
    scripts/gimbal_image_controller_node.py
    scripts/topic_gimbal_adapter_node.py
    scripts/xd_track_gimbal_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
