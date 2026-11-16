cmake_minimum_required(VERSION 3.5)
project(sar_yolo_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  diagnostic_updater
  geometry_msgs
  image_geometry
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  vision_msgs
  xd_uav_track
)

catkin_python_setup()

add_message_files(
  FILES
  FloodRegion.msg
  FloodRegionArray.msg
  AreaTaskAssignment.msg
  EvidenceReference.msg
  ExecutorCapabilities.msg
  ExecutorHeartbeat.msg
  GeoReference.msg
  MissionIdentity.msg
  PerceptionCandidate.msg
  PerceptionCandidateArray.msg
  FusionDetectionProvenance.msg
  FusionProvenanceArray.msg
  PerceptionIdentity.msg
  SmartTrackerState.msg
  TaskCandidate.msg
  TaskCandidateArray.msg
  TerrainGrid.msg
  ThermalImageInfo.msg
  TaskAssignment.msg
  TaskExecutionStatus.msg
  TrackedDetection2D.msg
  TrackedDetection2DArray.msg
  FusionDetection2D.msg
  FusionDetection2DArray.msg
  UavDecisionHeartbeat.msg
  DetectorModelInfo.msg
  NormalizedDetection.msg
)

add_service_files(
  FILES
  GetFloodRegions.srv
  GetTaskCandidates.srv
  GetEvidenceCrop.srv
  GetPerceptionSnapshot.srv
  GetTaskStatuses.srv
  SelectSmartTrack.srv
  SubmitAreaTaskAssignment.srv
  SubmitTaskAssignment.srv
  SwitchSmartTrackerModel.srv
  SwitchTrackingMode.srv
  SelectFusionROI.srv
  SelectFusionPoint.srv
  ListDetectorModels.srv
  GetSharedModelInfo.srv
  RunSharedInference.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
    vision_msgs
    xd_uav_track
)

# Keep the existing ROS message/service API stable for sibling packages. This
# package builds only the bridge node used by the selected SmartTracker launch.
catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    diagnostic_updater
    geometry_msgs
    image_geometry
    image_transport
    message_runtime
    roscpp
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_ros
    vision_msgs
    xd_uav_track
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(sar_vision_to_xd_bridge
  src/vision_to_xd_bridge_node.cpp
)
add_dependencies(sar_vision_to_xd_bridge
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(sar_vision_to_xd_bridge PRIVATE cxx_std_14)
target_link_libraries(sar_vision_to_xd_bridge ${catkin_LIBRARIES})

install(TARGETS sar_vision_to_xd_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch models docs web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md LICENSE THIRD_PARTY_NOTICES.md
  requirements-smart-tracker.txt requirements-tensorrt-cu12.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES LICENSES/PixEagle-Apache-2.0.txt LICENSES/Ultralytics-AGPL-3.0.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/LICENSES
)
install(PROGRAMS
  scripts/sar_yolo_smart_tracker
  scripts/sar_yolo_shared_model_server
  scripts/sar_yolo_multi_source_detection
  scripts/smart_tracker_node.py
  scripts/shared_inference_server.py
  scripts/multi_source_detection_node.py
  scripts/export_xd_vehicle_tensorrt.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/fusion_web_preview.py
  scripts/fusion_interactive_viewer.py
  scripts/validate_fusion_runtime.py
  scripts/validate_formal_gazebo_fusion.py
  scripts/verify_registered_engines.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_pixeagle_portable.py)
  catkin_add_nosetests(test/test_smart_tracker_image_codec.py)
  catkin_add_nosetests(test/test_manual_box_tracker.py)
  catkin_add_nosetests(test/test_box_fusion.py)
  catkin_add_nosetests(test/test_loss_journal.py)
  catkin_add_nosetests(test/test_fusion_metrics.py)
  catkin_add_nosetests(test/test_async_pipeline.py)
  catkin_add_nosetests(test/test_camera_motion.py)
  catkin_add_nosetests(test/test_capture_clock.py)
  catkin_add_nosetests(test/test_interactive_view.py)
  catkin_add_nosetests(test/test_fusion_web_preview.py)
  catkin_add_nosetests(test/test_fusion_adapter.py)
  catkin_add_nosetests(test/test_shared_detector_outputs.py)
  catkin_add_nosetests(test/test_model_runtime_status.py)
  catkin_add_nosetests(test/test_model_registry.py)
  catkin_add_nosetests(test/test_model_switch.py)
  catkin_add_nosetests(test/test_interface_v1_md5.py)
  catkin_add_nosetests(test/test_multi_source_roi_class_bridge.py)
  catkin_add_nosetests(test/test_model_switch_gpu.py)
  add_rostest(test/fusion_bridge.test)
  add_rostest(test/vision_to_xd_bridge.test)
endif()
