cmake_minimum_required(VERSION 3.0.2)
project(pod_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geographic_msgs
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  ControlLease.msg
  GeoTarget.msg
  GimbalCommand.msg
  GimbalState.msg
  MissionState.msg
  SelectedTarget.msg
  TargetTrack.msg
  TargetTrackArray.msg
  TaskEvent.msg
  UavControlExecution.msg
  UavState.msg
)

add_action_files(
  FILES
  PointToGeo.action
  SearchArea.action
  StowGimbal.action
  TrackTarget.action
)

add_service_files(
  FILES
  ManageGimbalLease.srv
  SelectTarget.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geographic_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geographic_msgs
    geometry_msgs
    message_runtime
    std_msgs
)
