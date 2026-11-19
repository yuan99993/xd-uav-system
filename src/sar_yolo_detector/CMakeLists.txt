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
  roslib
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

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d video tracking)
find_package(CUDA REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include/aarch64-linux-gnu /usr/include /usr/local/include
)
find_library(TENSORRT_NVINFER_LIBRARY nvinfer
  PATHS /usr/lib/aarch64-linux-gnu /usr/lib /usr/local/lib
)
find_library(TENSORRT_PLUGIN_LIBRARY nvinfer_plugin
  PATHS /usr/lib/aarch64-linux-gnu /usr/lib /usr/local/lib
)
if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_NVINFER_LIBRARY OR NOT TENSORRT_PLUGIN_LIBRARY)
  message(FATAL_ERROR "TensorRT development headers and libraries (nvinfer, nvinfer_plugin) are required")
endif()

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

# Keep the existing ROS message/service API stable for sibling packages.
catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    diagnostic_updater
    geometry_msgs
    image_geometry
    image_transport
    message_runtime
    roscpp
    roslib
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

add_executable(sar_yolo_shared_model_server_cpp
  src/shared_inference_server_trt_node.cpp
)
add_dependencies(sar_yolo_shared_model_server_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(sar_yolo_shared_model_server_cpp PRIVATE cxx_std_14)
target_include_directories(sar_yolo_shared_model_server_cpp PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIRS}
)
target_link_libraries(sar_yolo_shared_model_server_cpp
  ${roscpp_LIBRARIES}
  ${roslib_LIBRARIES}
  ${OpenCV_LIBS}
  ${CUDA_CUDART_LIBRARY}
  ${TENSORRT_NVINFER_LIBRARY}
  ${TENSORRT_PLUGIN_LIBRARY}
  yaml-cpp
  OpenSSL::Crypto
)

add_executable(sar_yolo_smart_tracker_cpp
  src/smart_tracker_fusion_cpp_node.cpp
)
add_dependencies(sar_yolo_smart_tracker_cpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(sar_yolo_smart_tracker_cpp PRIVATE cxx_std_14)
target_include_directories(sar_yolo_smart_tracker_cpp PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(sar_yolo_smart_tracker_cpp
  ${roscpp_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)

install(TARGETS sar_vision_to_xd_bridge sar_yolo_shared_model_server_cpp
                sar_yolo_smart_tracker_cpp
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
