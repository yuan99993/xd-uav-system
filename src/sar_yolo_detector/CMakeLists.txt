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
  tf2
  tf2_geometry_msgs
  tf2_ros
  vision_msgs
)
find_package(OpenCV REQUIRED COMPONENTS core dnn imgcodecs imgproc)
find_package(OpenSSL REQUIRED)
find_package(SQLite3 REQUIRED)

catkin_python_setup()

option(SAR_YOLO_ENABLE_TENSORRT
  "Build the TensorRT engine backend when TensorRT and CUDA runtime are installed"
  ON)
set(SAR_YOLO_TENSORRT_SOURCE src/tensorrt_backend_unavailable.cpp)
set(SAR_YOLO_TENSORRT_LIBRARIES)
if(SAR_YOLO_ENABLE_TENSORRT)
  # TensorRT headers include <cuda_runtime_api.h> themselves.  CUDA's Debian
  # toolkit deliberately installs that header under the versioned target
  # directory, which is not a default compiler include path on ROS Noetic.
  find_path(CUDA_RUNTIME_INCLUDE_DIR cuda_runtime_api.h
    HINTS
      /usr/local/cuda/include
      /usr/local/cuda/targets/x86_64-linux/include
      /usr/local/cuda-12.4/include
      /usr/local/cuda-12.4/targets/x86_64-linux/include)
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    HINTS /usr/include /usr/local/include /usr/include/x86_64-linux-gnu)
  find_library(TENSORRT_LIBRARY NAMES nvinfer
    HINTS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib)
  find_library(CUDART_LIBRARY NAMES cudart
    HINTS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/cuda/lib64
          /usr/local/cuda/targets/x86_64-linux/lib)
  if(CUDA_RUNTIME_INCLUDE_DIR AND TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY AND CUDART_LIBRARY)
    message(STATUS "sar_yolo_detector: TensorRT engine backend enabled")
    set(SAR_YOLO_TENSORRT_SOURCE src/tensorrt_backend.cpp)
    set(SAR_YOLO_TENSORRT_LIBRARIES ${TENSORRT_LIBRARY} ${CUDART_LIBRARY})
    add_compile_definitions(SAR_YOLO_HAS_TENSORRT=1)
  else()
    message(STATUS "sar_yolo_detector: TensorRT unavailable; building explicit unavailable backend")
  endif()
endif()

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
  UavDecisionHeartbeat.msg
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
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
    vision_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sar_yolo_postprocessor sar_yolo_inference_backend sar_model_integrity sar_mission_state_store
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
    tf2
    tf2_geometry_msgs
    tf2_ros
    vision_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SQLite3_INCLUDE_DIRS}
)
if(CUDA_RUNTIME_INCLUDE_DIR AND TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY AND CUDART_LIBRARY)
  include_directories(${CUDA_RUNTIME_INCLUDE_DIR} ${TENSORRT_INCLUDE_DIR})
endif()

add_library(sar_yolo_postprocessor
  src/yolo_postprocessor.cpp
)
target_compile_features(sar_yolo_postprocessor PUBLIC cxx_std_14)
target_link_libraries(sar_yolo_postprocessor ${OpenCV_LIBRARIES})

add_library(sar_yolo_inference_backend
  src/inference_backend.cpp
  ${SAR_YOLO_TENSORRT_SOURCE}
)
target_compile_features(sar_yolo_inference_backend PUBLIC cxx_std_14)
target_link_libraries(sar_yolo_inference_backend
  ${OpenCV_LIBRARIES}
  ${SAR_YOLO_TENSORRT_LIBRARIES}
)

add_library(sar_model_integrity src/model_integrity.cpp)
target_compile_features(sar_model_integrity PUBLIC cxx_std_14)
target_link_libraries(sar_model_integrity OpenSSL::Crypto)

add_library(sar_mission_state_store src/mission_state_store.cpp)
add_dependencies(sar_mission_state_store
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(sar_mission_state_store PUBLIC cxx_std_14)
target_link_libraries(sar_mission_state_store
  OpenSSL::Crypto
  ${SQLite3_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(sar_yolo_detector_node
  src/sar_yolo_detector_node.cpp
)
add_dependencies(sar_yolo_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(sar_yolo_detector_node PRIVATE cxx_std_14)
target_link_libraries(sar_yolo_detector_node
  sar_yolo_postprocessor
  sar_yolo_inference_backend
  sar_model_integrity
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Converts temporally stable visual detections into mission-decision candidates.
# It is deliberately separate from the detector so model/tracker interfaces stay
# compatible and it can never command an aircraft directly.
add_executable(sar_task_point_generator
  src/task_point_generator_node.cpp
)
add_dependencies(sar_task_point_generator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(sar_task_point_generator PRIVATE cxx_std_14)
target_link_libraries(sar_task_point_generator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# A small C++ image source for repeatable end-to-end detector/tracker tests.
# It deliberately remains useful outside tests for replaying a still rescue
# frame without depending on a collaborator-local package or Python script.
add_executable(sar_yolo_image_replay
  src/image_replay_node.cpp
)
add_dependencies(sar_yolo_image_replay ${catkin_EXPORTED_TARGETS})
target_compile_features(sar_yolo_image_replay PRIVATE cxx_std_14)
target_link_libraries(sar_yolo_image_replay
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(sar_detection_late_fusion
  src/detection_late_fusion_node.cpp
)
add_dependencies(sar_detection_late_fusion
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(sar_detection_late_fusion PRIVATE cxx_std_14)
target_link_libraries(sar_detection_late_fusion
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# FloodNet semantic profile. This is part of the unified SAR recognition
# frontend, while remaining a separate node internally because it consumes a
# class-index mask and publishes FloodRegionArray rather than YOLO boxes.
add_executable(sar_flood_region_extractor
  src/flood_region_extractor_node.cpp
)
add_dependencies(sar_flood_region_extractor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(sar_flood_region_extractor PRIVATE cxx_std_14)
target_link_libraries(sar_flood_region_extractor
  OpenSSL::Crypto
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Complete FloodNet RGB -> SegFormer -> MONO8 class-mask inference path. The
# shared backend supports both FP32 and FP16 TensorRT I/O.
add_executable(sar_segformer_inference
  src/segformer_inference_node.cpp
)
add_dependencies(sar_segformer_inference
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(sar_segformer_inference PRIVATE cxx_std_14)
target_link_libraries(sar_segformer_inference
  sar_yolo_inference_backend
  sar_model_integrity
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Transport-neutral boundary between this perception package, a remote mission
# decision service, and the vehicle-specific onboard task executor.
add_executable(sar_decision_bridge
  src/decision_bridge_node.cpp
)
add_dependencies(sar_decision_bridge
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(sar_decision_bridge PRIVATE cxx_std_14)
target_link_libraries(sar_decision_bridge
  sar_mission_state_store
  OpenSSL::Crypto
  ${catkin_LIBRARIES}
)

install(TARGETS sar_yolo_postprocessor sar_yolo_inference_backend sar_model_integrity sar_mission_state_store sar_yolo_detector_node sar_task_point_generator sar_yolo_image_replay sar_detection_late_fusion sar_flood_region_extractor sar_segformer_inference sar_decision_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch training models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md LICENSE THIRD_PARTY_NOTICES.md requirements-smart-tracker.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES LICENSES/PixEagle-Apache-2.0.txt LICENSES/Ultralytics-AGPL-3.0.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/LICENSES
)
install(PROGRAMS
  scripts/sar_yolo_smart_tracker
  scripts/smart_tracker_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS
  examples/sign_and_submit_task.py
  test/test_detector_interfaces.py
  test/test_decision_bridge.py
  test/test_decision_bridge_scout.py
  test/test_flood_region_extractor.py
  test/test_late_fusion.py
  test/test_interface_v1_md5.py
  test/test_scout_aircraft_task_chain.py
  test/test_task_point_bearing_only.py
  test/test_task_generation_gate.py
  test/test_task_point_generator.py
  test/test_thermal_input.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_pixeagle_portable.py)
  catkin_add_nosetests(test/test_smart_tracker_image_codec.py)
  catkin_add_nosetests(test/test_interface_v1_md5.py)
  catkin_add_gtest(test_yolo_postprocessor
    test/test_yolo_postprocessor.cpp
  )
  if(TARGET test_yolo_postprocessor)
    target_compile_features(test_yolo_postprocessor PRIVATE cxx_std_14)
    target_link_libraries(test_yolo_postprocessor
      sar_yolo_postprocessor
      ${OpenCV_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_inference_backend
    test/test_inference_backend.cpp
  )
  if(TARGET test_inference_backend)
    target_compile_features(test_inference_backend PRIVATE cxx_std_14)
    target_link_libraries(test_inference_backend
      sar_yolo_inference_backend
      ${OpenCV_LIBRARIES}
    )
  endif()
  catkin_add_gtest(test_linear_assignment
    test/test_linear_assignment.cpp
  )
  if(TARGET test_linear_assignment)
    target_compile_features(test_linear_assignment PRIVATE cxx_std_14)
  endif()
  add_rostest(test/detector_interfaces.test)
  add_rostest(test/decision_bridge.test)
  add_rostest(test/decision_bridge_scout.test)
  add_rostest(test/flood_region_extractor.test)
  add_rostest(test/late_fusion.test)
  add_rostest(test/scout_aircraft_task_chain.test)
  add_rostest(test/task_point_bearing_only.test)
  add_rostest(test/task_generation_gate.test)
  add_rostest(test/task_point_generator.test)
  add_rostest(test/thermal_input.test)
endif()
