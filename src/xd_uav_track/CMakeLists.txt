cmake_minimum_required(VERSION 3.5)
project(xd_uav_track)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  DetectionArray.msg
  DetectionCandidate.msg
  FollowerCommand.msg
  GimbalState.msg
  TrackState.msg
  TrackStateArray.msg
  TrackStatus.msg
  GimbalStatus.msg
)

add_service_files(
  FILES
  SelectTrack.srv
  SetProfile.srv
  StartTracker.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xd_uav_track_core
  CATKIN_DEPENDS
    geometry_msgs
    mavros_msgs
    message_runtime
    nav_msgs
    roscpp
    rospy
    std_msgs
    std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(xd_uav_track_core
  src/multi_track_manager.cpp
  src/target_guidance.cpp
  src/track_controller.cpp
)
target_compile_features(xd_uav_track_core PUBLIC cxx_std_14)
add_dependencies(xd_uav_track_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(xd_uav_track_core ${catkin_LIBRARIES})

add_executable(xd_uav_track_node
  src/xd_uav_track_node.cpp
)
target_compile_features(xd_uav_track_node PRIVATE cxx_std_14)
add_dependencies(xd_uav_track_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(xd_uav_track_node
  xd_uav_track_core
  ${catkin_LIBRARIES}
)

install(TARGETS xd_uav_track_core xd_uav_track_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch models docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/configure_px4_sitl.py
  scripts/record_tracking_decisions.py
  scripts/render_plane_catapult_sdf.py
  scripts/replay_tracking_faults.py
  scripts/run_px4_sitl.py
  scripts/run_with_config_preflight.py
  scripts/sitl_metric_input.py
  scripts/validate_tracking_config.py
  scripts/verify_fixedwing_sitl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_track_controller
    test/test_track_controller.cpp
  )
  if(TARGET test_track_controller)
    target_compile_features(test_track_controller PRIVATE cxx_std_14)
    target_link_libraries(test_track_controller
      xd_uav_track_core
      ${catkin_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_multi_track_manager
    test/test_multi_track_manager.cpp
  )
  if(TARGET test_multi_track_manager)
    target_compile_features(test_multi_track_manager PRIVATE cxx_std_14)
    add_dependencies(test_multi_track_manager
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(test_multi_track_manager
      xd_uav_track_core
      ${catkin_LIBRARIES}
    )
  endif()
endif()
