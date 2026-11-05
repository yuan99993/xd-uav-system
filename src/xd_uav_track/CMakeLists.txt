cmake_minimum_required(VERSION 3.5)
project(xd_uav_track)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)
find_package(Eigen3 REQUIRED)

# Install the lightweight pixel tracker and optional ReID helpers alongside
# generated ROS messages; the devel-space source path alone is not deployable.
catkin_python_setup()

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
  MetricTarget.msg
  TargetPixel.msg
  TargetPixelArray.msg
  SetPerceptionModeCommand.msg
  SelectPerceptionTargetCommand.msg
  PerceptionCommandResult.msg
  PerceptionState.msg
  PerceptionTarget.msg
  PerceptionTargetArray.msg
  TargetTrajectoryPoint.msg
  TargetTrajectoryArray.msg
  SetDetectorModelCommand.msg
  DetectorModelCommit.msg
  PerceptionVideoFrame.msg
)

add_service_files(
  FILES
  SelectTrack.srv
  SetTargetMode.srv
  SetProfile.srv
  StartTracker.srv
  ManageVisualIdentity.srv
  GetPerceptionState.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xd_uav_track_core
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    mavros_msgs
    message_runtime
    nav_msgs
    roscpp
    rospy
    sensor_msgs
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
  src/tracklet_stitcher.cpp
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

install(DIRECTORY config launch models docs worlds web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES requirements-reid-onnx-cuda12.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/px4_mission.py
  scripts/configure_px4_sitl.py
  scripts/record_tracking_decisions.py
  scripts/render_plane_catapult_sdf.py
  scripts/replay_tracking_faults.py
  scripts/run_px4_sitl.py
  scripts/run_with_config_preflight.py
  scripts/sitl_metric_input.py
  scripts/validate_tracking_config.py
  scripts/verify_fixedwing_sitl.py
  scripts/tracking_benchmark_driver.py
  scripts/check_tracking_benchmark_baseline.py
  scripts/tracking_overlay_node.py
  scripts/combined_tracking_view_node.py
  scripts/video_replay_node.py
  scripts/video_tracking_record_node.py
  scripts/evaluate_video_tracking.py
  scripts/moving_uav_search_driver.py
  scripts/gimbal_tracking_metrics.py
  scripts/perception_mode_manager.py
  scripts/perception_overlay_node.py
  scripts/serve_perception_web.py
  scripts/bound_rosbridge_websocket.py
  scripts/validate_perception_web_runtime.py
  scripts/reid_encoder_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/benchmark_trajectories.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/xd_uav_reid_encoder
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/perception_visual_identity.test)
  add_rostest(test/perception_storage.test)
  catkin_add_nosetests(test/test_pixel_lock_overlay.py)
  catkin_add_nosetests(test/test_airground_mission.py)
  catkin_add_nosetests(test/test_combined_tracking_view.py)
  catkin_add_nosetests(test/test_sot_tracker.py)
  catkin_add_nosetests(test/test_perception_modes.py)
  catkin_add_nosetests(test/test_pixel_worker.py)
  catkin_add_nosetests(test/test_frame_admission.py)
  catkin_add_nosetests(test/test_fusion_revision_probe.py)
  catkin_add_nosetests(test/test_perception_frame_selection.py)
  catkin_add_nosetests(test/test_perception_loss_display.py)
  catkin_add_nosetests(test/test_perception_video.py)
  catkin_add_nosetests(test/test_perception_web_http.py)
  catkin_add_nosetests(test/test_perception_position_metadata.py)
  catkin_add_nosetests(test/test_perception_trajectory.py)
  catkin_add_nosetests(test/test_perception_model_transaction.py)
  catkin_add_nosetests(test/test_perception_command_queue.py)
  catkin_add_nosetests(test/test_video_tracking_eval.py)
  catkin_add_nosetests(test/test_benchmark_trajectories.py)
  catkin_add_nosetests(test/test_moving_uav_search_dynamics.py)
  catkin_add_nosetests(test/test_vehicle_perception_launch.py)
  catkin_add_nosetests(test/test_perception_deployment.py)
  catkin_add_nosetests(test/test_benchmark_baseline.py)
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
  catkin_add_gtest(test_tracklet_stitcher
    test/test_tracklet_stitcher.cpp
  )
  if(TARGET test_tracklet_stitcher)
    target_compile_features(test_tracklet_stitcher PRIVATE cxx_std_14)
    target_link_libraries(test_tracklet_stitcher
      xd_uav_track_core
      ${catkin_LIBRARIES}
    )
  endif()
endif()
