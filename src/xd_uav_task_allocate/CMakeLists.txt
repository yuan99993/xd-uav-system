cmake_minimum_required(VERSION 3.5)
project(xd_uav_task_allocate)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  target_geolocation
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
  CancelPlanning.srv
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
  INCLUDE_DIRS include
  LIBRARIES xd_uav_task_allocate_core
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    mavros_msgs
    message_runtime
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    target_geolocation
    tf2_ros
    visualization_msgs
    xd_uav_controller
    xd_uav_task_execute
    xd_uav_track
)

add_library(xd_uav_task_allocate_core src/core.cpp)
target_compile_features(xd_uav_task_allocate_core PUBLIC cxx_std_17)
target_include_directories(xd_uav_task_allocate_core PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(xd_uav_task_allocate_core ${catkin_LIBRARIES})
add_dependencies(xd_uav_task_allocate_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(task_allocate_node_cpp src/task_allocate_node.cpp)
target_compile_features(task_allocate_node_cpp PUBLIC cxx_std_17)
target_include_directories(task_allocate_node_cpp PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(task_allocate_node_cpp xd_uav_task_allocate_core ${catkin_LIBRARIES})
add_dependencies(task_allocate_node_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS xd_uav_task_allocate_core task_allocate_node_cpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
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
