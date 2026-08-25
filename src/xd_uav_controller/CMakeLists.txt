cmake_minimum_required(VERSION 3.0.2)
project(xd_uav_controller)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  ControlCommand.msg
  ControlState.msg
  PathStatus.msg
)

add_service_files(
  FILES
  InternalCommand.srv
  Takeoff.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    mavros_msgs
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(controller_node src/controller_node.cpp)
add_dependencies(controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(controller_node PRIVATE cxx_std_17)
target_link_libraries(controller_node Eigen3::Eigen ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/publish_fixedwing_trajectory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/controller.test)
  add_rostest(test/odom_landing.test)
  add_rostest(test/multirotor_anti_windup.test)
  add_rostest(test/fixedwing_controller.test)
endif()

install(TARGETS controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
