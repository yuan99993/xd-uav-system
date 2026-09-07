cmake_minimum_required(VERSION 3.5)
project(xd_uav_task_allocate)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  rospy
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
  xd_uav_controller
  xd_uav_task_execute
  xd_uav_track
)

add_message_files(
  FILES
  GlobalTarget.msg
  GlobalTargetArray.msg
  PlannerStatus.msg
  RescueTask.msg
  RescueTaskArray.msg
  SearchArea.msg
  SearchAreaArray.msg
)

add_service_files(
  FILES
  GetMissionState.srv
  LoadSearchAreas.srv
  SetVehicleEnabled.srv
  TargetCommand.srv
  TaskCommand.srv
  VehicleCommand.srv
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    mavros_msgs
    message_runtime
    nav_msgs
    rospy
    std_msgs
    std_srvs
    tf2_ros
    visualization_msgs
    xd_uav_controller
    xd_uav_task_execute
    xd_uav_track
)

catkin_install_python(PROGRAMS
  scripts/task_allocate_node.py
  scripts/mock_ego_swarm.py
  scripts/mock_search_mission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_task_allocate_core.py)
endif()
