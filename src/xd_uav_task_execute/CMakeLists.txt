cmake_minimum_required(VERSION 3.5)
project(xd_uav_task_execute)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  rospy
  std_msgs
  xd_uav_track
)

add_action_files(
  DIRECTORY action
  FILES ExecuteTask.action
)

add_message_files(
  FILES TaskExecutionStatus.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    message_runtime
    rospy
    std_msgs
    xd_uav_track
)

catkin_install_python(PROGRAMS
  scripts/task_execute_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_track_monitor.py)
endif()
