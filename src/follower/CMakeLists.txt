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
  tf
  mavros_msgs
  mrs_msgs
  tracker
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
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  tracker
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
  scripts/follower_node.py
  scripts/test_tracker_follower_offboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
