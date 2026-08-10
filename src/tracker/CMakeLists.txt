cmake_minimum_required(VERSION 3.5)
project(tracker)

set(CATKIN_DEPENDENCIES
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  diagnostic_updater
  tf2_ros
  tf2_geometry_msgs
  pod_msgs
  message_generation
  message_runtime
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d)

# ── Python Setup (MUST be before generate_messages) ────────────────────
catkin_python_setup()

# ── Custom Messages ──────────────────────────────────────────────────────
add_message_files(
  FILES
  TrackingOutput.msg
  NormalizedError.msg
  ExternalInput.msg
  DetectionCandidate.msg
  DetectionArray.msg
  TrackState.msg
  TrackStateArray.msg
  )

add_service_files(
  FILES
  SelectTrack.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tracker_multi_track_manager
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

# ── C++ tracking path ────────────────────────────────────────────────────

add_library(tracker_multi_track_manager
  src/multi_track_manager.cpp
  )
add_dependencies(tracker_multi_track_manager
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_compile_features(tracker_multi_track_manager PRIVATE cxx_std_14)
target_link_libraries(tracker_multi_track_manager
  ${catkin_LIBRARIES}
  )

add_executable(tracker_node
  src/tracker_node.cpp
  )
add_dependencies(tracker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_compile_features(tracker_node PRIVATE cxx_std_14)
target_link_libraries(tracker_node
  tracker_multi_track_manager
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

# ── Install ──────────────────────────────────────────────────────────────

install(TARGETS tracker_node tracker_multi_track_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )

install(FILES IMPLEMENTATION.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# The C++ tracker_node is the production implementation. The Python wrapper is
# retained as a backwards-compatible reference implementation only.
catkin_install_python(PROGRAMS
  scripts/tracker_node.py
  scripts/test_external_input.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  test/test_tracker_interfaces.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(PROGRAMS
  scripts/deploy_mrs_adapter.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_multi_track_manager
    test/test_multi_track_manager.cpp
    )
  if(TARGET test_multi_track_manager)
    add_dependencies(test_multi_track_manager
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
      )
    target_compile_features(test_multi_track_manager PRIVATE cxx_std_14)
    target_link_libraries(test_multi_track_manager
      tracker_multi_track_manager
      ${catkin_LIBRARIES}
      )
  endif()
  add_rostest(test/tracker_interfaces.test)
endif()
