cmake_minimum_required(VERSION 3.0.2)
project(xd_uav_control_manager)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  xd_uav_controller
  xd_uav_state_estimators
)

catkin_package(
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    mavros_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    xd_uav_controller
    xd_uav_state_estimators
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(control_manager_node src/control_manager_node.cpp)
add_dependencies(control_manager_node ${catkin_EXPORTED_TARGETS})
target_compile_features(control_manager_node PRIVATE cxx_std_17)
target_link_libraries(control_manager_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/control_manager.test)
  add_rostest(test/fixedwing_control_manager.test)
endif()

install(TARGETS control_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
