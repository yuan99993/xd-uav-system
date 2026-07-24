cmake_minimum_required(VERSION 3.0.2)
project(xd_uav_state_estimator)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geographic_msgs
  geometry_msgs
  message_generation
  mrs_lib
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_message_files(
  FILES
  EstimatorStatus.msg
)

add_service_files(
  FILES
  SwitchLocalizationSource.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(Eigen3 REQUIRED)
get_target_property(Eigen3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xd_uav_state_filters xd_uav_localization_sources
  CATKIN_DEPENDS
    diagnostic_msgs
    geographic_msgs
    geometry_msgs
    message_runtime
    mrs_lib
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(xd_uav_state_filters
  src/heading_filter.cpp
  src/imu_acceleration_processor.cpp
  src/linear_filters.cpp
)
add_dependencies(xd_uav_state_filters ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(xd_uav_state_filters PUBLIC cxx_std_17)
target_link_libraries(xd_uav_state_filters
  Eigen3::Eigen
  ${catkin_LIBRARIES}
)

add_library(xd_uav_localization_sources
  src/localization_source_manager.cpp
)
add_dependencies(xd_uav_localization_sources ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(xd_uav_localization_sources PUBLIC cxx_std_17)
target_link_libraries(xd_uav_localization_sources
  Eigen3::Eigen
  ${catkin_LIBRARIES}
)

add_executable(state_estimator_node
  src/state_estimator_node.cpp
  src/main.cpp
)
add_dependencies(state_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(state_estimator_node PRIVATE cxx_std_17)
target_link_libraries(state_estimator_node
  xd_uav_state_filters
  xd_uav_localization_sources
  ${catkin_LIBRARIES}
)

add_executable(gps_world_alignment_node
  src/gps_world_alignment_node.cpp
)
add_dependencies(gps_world_alignment_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_features(gps_world_alignment_node PRIVATE cxx_std_17)
target_link_libraries(gps_world_alignment_node
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_gtest(test_linear_filters
    test/test_linear_filters.cpp
  )
  if(TARGET test_linear_filters)
    target_compile_features(test_linear_filters PRIVATE cxx_std_17)
    target_link_libraries(test_linear_filters
      xd_uav_state_filters
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_imu_acceleration_processor
    test/test_imu_acceleration_processor.cpp
  )
  if(TARGET test_imu_acceleration_processor)
    target_compile_features(test_imu_acceleration_processor PRIVATE cxx_std_17)
    target_link_libraries(test_imu_acceleration_processor
      xd_uav_state_filters
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_heading_filter
    test/test_heading_filter.cpp
  )
  if(TARGET test_heading_filter)
    target_compile_features(test_heading_filter PRIVATE cxx_std_17)
    target_link_libraries(test_heading_filter
      xd_uav_state_filters
      ${catkin_LIBRARIES}
    )
  endif()

  add_rostest(test/test_multi_source.test)
endif()

install(TARGETS xd_uav_state_filters xd_uav_localization_sources state_estimator_node gps_world_alignment_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
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
