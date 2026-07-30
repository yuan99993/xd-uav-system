cmake_minimum_required(VERSION 3.5)
project(tracker)

set(CATKIN_DEPENDENCIES
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
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
  TrackingOutput.msg
  NormalizedError.msg
  ExternalInput.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

# ── Install ──────────────────────────────────────────────────────────────

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )

# Install Python nodes to devel AND install space
catkin_install_python(PROGRAMS
  scripts/tracker_node.py
  scripts/test_external_input.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(PROGRAMS
  scripts/deploy_mrs_adapter.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
