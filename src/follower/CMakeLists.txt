cmake_minimum_required(VERSION 3.5)
project(follower)

set(CATKIN_DEPENDENCIES
  rospy
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  diagnostic_updater
  tf
  mavros_msgs
  mrs_msgs
  tracker
  xd_uav_controller
  pod_msgs
  message_generation
  message_runtime
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

# ── Python Setup (MUST be before generate_messages) ────────────────────
catkin_python_setup()

# ── Custom Messages ──────────────────────────────────────────────────────
add_message_files(
  FILES
  FollowerCommand.msg
  FollowerStatus.msg
  ControllerFeedback.msg
  ControllerCommand.msg
  )

add_service_files(
  FILES
  SetMode.srv
  ManageControlLease.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  tracker
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES follower_control_guard
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

# ── C++ real-time path ──────────────────────────────────────────────────

add_library(follower_control_guard
  src/control_guard.cpp
  )
target_compile_features(follower_control_guard PRIVATE cxx_std_14)
target_link_libraries(follower_control_guard ${catkin_LIBRARIES})

add_executable(xd_controller_reference_adapter_node
  src/xd_controller_reference_adapter_node.cpp
  )
add_executable(follower_node
  src/follower_node.cpp
  )
add_executable(px4_attitude_rate_adapter_node
  src/px4_attitude_rate_adapter_node.cpp
  )
add_dependencies(xd_controller_reference_adapter_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
add_dependencies(follower_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
add_dependencies(px4_attitude_rate_adapter_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )
target_compile_features(xd_controller_reference_adapter_node PRIVATE cxx_std_14)
target_compile_features(follower_node PRIVATE cxx_std_14)
target_compile_features(px4_attitude_rate_adapter_node PRIVATE cxx_std_14)
target_link_libraries(xd_controller_reference_adapter_node
  ${catkin_LIBRARIES}
  )
target_link_libraries(follower_node
  follower_control_guard
  ${catkin_LIBRARIES}
  )
target_link_libraries(px4_attitude_rate_adapter_node
  ${catkin_LIBRARIES}
  )

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_control_guard test/test_control_guard.cpp)
  if(TARGET test_control_guard)
    target_compile_features(test_control_guard PRIVATE cxx_std_14)
    target_link_libraries(test_control_guard follower_control_guard)
  endif()
  add_rostest(test/follower_profiles.test)
  add_rostest(test/attitude_adapter.test)
  add_rostest(test/profile_switch_guard.test)
  add_rostest(test/control_guard_interfaces.test)
endif()

# ── Install ──────────────────────────────────────────────────────────────

install(TARGETS
  follower_control_guard
  follower_node
  xd_controller_reference_adapter_node
  px4_attitude_rate_adapter_node
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

install(FILES IMPLEMENTATION.md PROFILE_MODES.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# Install Python nodes to devel AND install space
catkin_install_python(PROGRAMS
  scripts/follower_node.py
  scripts/xd_controller_reference_adapter.py
  scripts/test_tracker_follower_offboard.py
  test/test_control_guard_interfaces.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(PROGRAMS
  scripts/start_px4_sitl_udp
  scripts/start_xd_tracking_stack
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
