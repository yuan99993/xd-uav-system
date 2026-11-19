cmake_minimum_required(VERSION 3.0.2)
project(xd_uav_state_estimators)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  xmlrpcpp
)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  EstimatorStatus.msg
  Heading.msg
  PositionXY.msg
  PositionZ.msg
  VelocityXY.msg
  VelocityZ.msg
  YawRate.msg
)

add_service_files(
  FILES
  SwitchLocalizationSource.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES odometry_adapter_runtime
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    xmlrpcpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(odometry_adapter_runtime STATIC
  src/odometry_adapter.cpp
)
add_dependencies(odometry_adapter_runtime
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(odometry_adapter_runtime PRIVATE cxx_std_17)
target_link_libraries(odometry_adapter_runtime
  Eigen3::Eigen
  ${catkin_LIBRARIES}
)

add_executable(multi_source_estimator_node
  src/multi_source_estimator_node.cpp
)
add_dependencies(multi_source_estimator_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(multi_source_estimator_node PRIVATE cxx_std_17)
target_link_libraries(multi_source_estimator_node
  odometry_adapter_runtime
  Eigen3::Eigen
  ${catkin_LIBRARIES}
)

add_executable(odometry_adapter_manager_node
  src/odometry_adapter_manager_node.cpp
)
add_dependencies(odometry_adapter_manager_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(odometry_adapter_manager_node PRIVATE cxx_std_17)
target_link_libraries(odometry_adapter_manager_node
  odometry_adapter_runtime
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/three_package_integration.test)
endif()

install(TARGETS
  odometry_adapter_runtime
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS
  multi_source_estimator_node
  odometry_adapter_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
