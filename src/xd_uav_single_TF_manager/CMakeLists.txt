cmake_minimum_required(VERSION 3.0.2)
project(xd_uav_single_tf_manager)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geographic_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(GeographicLib REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    diagnostic_msgs
    geographic_msgs
    geometry_msgs
    mavros_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_executable(single_tf_manager_node
  src/single_tf_manager_node.cpp
)
add_dependencies(single_tf_manager_node ${catkin_EXPORTED_TARGETS})
target_compile_features(single_tf_manager_node PRIVATE cxx_std_17)
target_link_libraries(single_tf_manager_node ${catkin_LIBRARIES})

add_executable(gps_global_alignment_node
  src/gps_global_alignment_node.cpp
)
add_dependencies(gps_global_alignment_node ${catkin_EXPORTED_TARGETS})
target_compile_features(gps_global_alignment_node PRIVATE cxx_std_17)
target_link_libraries(gps_global_alignment_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

install(TARGETS
  single_tf_manager_node
  gps_global_alignment_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
