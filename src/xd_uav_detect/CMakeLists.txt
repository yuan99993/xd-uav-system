cmake_minimum_required(VERSION 3.5)
project(xd_uav_detect)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  trajectory_msgs
  xd_uav_track
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)

add_message_files(
  FILES
  WorldDetection.msg
  WorldDetectionArray.msg
  GimbalCommand.msg
  GimbalState.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xd_uav_detect_core
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_geometry
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
    trajectory_msgs
    xd_uav_track
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(xd_uav_detect_core
  src/gimbal/gimbal_command_controller.cpp
  src/localization/gimbal_range_localization.cpp
  src/localization/ground_plane_projection.cpp
  src/localization/lidar_camera_fusion.cpp
  src/localization/localization_backend.cpp
)
target_compile_features(xd_uav_detect_core PUBLIC cxx_std_14)
add_dependencies(xd_uav_detect_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xd_uav_detect_core ${catkin_LIBRARIES})

add_executable(xd_uav_detect_node
  src/xd_uav_detect_node.cpp
)
target_compile_features(xd_uav_detect_node PRIVATE cxx_std_14)
add_dependencies(xd_uav_detect_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xd_uav_detect_node
  xd_uav_detect_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(gimbal_control_node
  src/gimbal/gimbal_control_node.cpp
)
target_compile_features(gimbal_control_node PRIVATE cxx_std_14)
add_dependencies(gimbal_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gimbal_control_node
  xd_uav_detect_core
  ${catkin_LIBRARIES}
)

add_library(xd_uav_detect_gimbal_joint_controller SHARED
  src/gazebo/gimbal_joint_controller_plugin.cpp
)
target_compile_features(xd_uav_detect_gimbal_joint_controller PRIVATE cxx_std_14)
add_dependencies(xd_uav_detect_gimbal_joint_controller
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xd_uav_detect_gimbal_joint_controller
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS xd_uav_detect_core xd_uav_detect_node gimbal_control_node
  xd_uav_detect_gimbal_joint_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config docs launch models worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/demo/gimbal_flight_start.py
  scripts/demo/gimbal_range_demo.py
  scripts/demo/red_box_detector.py
  scripts/demo/spawn_random_vehicles.py
  scripts/demo/spawn_red_boxes.py
  scripts/demo/spawn_yolo_vehicle_targets.py
  scripts/model/render_payload_models.py
  test/ros/gimbal_range_node_test.py
  test/gazebo/gimbal_range_acceptance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_lidar_camera_fusion
    test/unit/test_lidar_camera_fusion.cpp
  )
  if(TARGET test_lidar_camera_fusion)
    target_compile_features(test_lidar_camera_fusion PRIVATE cxx_std_14)
    target_link_libraries(test_lidar_camera_fusion
      xd_uav_detect_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_ground_plane_projection
    test/unit/test_ground_plane_projection.cpp
  )
  if(TARGET test_ground_plane_projection)
    target_compile_features(test_ground_plane_projection PRIVATE cxx_std_14)
    target_link_libraries(test_ground_plane_projection
      xd_uav_detect_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_gimbal_range_localization
    test/unit/test_gimbal_range_localization.cpp
  )
  if(TARGET test_gimbal_range_localization)
    target_compile_features(test_gimbal_range_localization PRIVATE cxx_std_14)
    target_link_libraries(test_gimbal_range_localization
      xd_uav_detect_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_gimbal_command_controller
    test/unit/test_gimbal_command_controller.cpp
  )
  if(TARGET test_gimbal_command_controller)
    target_compile_features(test_gimbal_command_controller PRIVATE cxx_std_14)
    target_link_libraries(test_gimbal_command_controller
      xd_uav_detect_core
      ${catkin_LIBRARIES}
    )
  endif()

  add_rostest(test/gazebo/gimbal_range.test)
  catkin_add_gtest(test_localization_backend
    test/unit/test_localization_backend.cpp
  )
  if(TARGET test_localization_backend)
    target_compile_features(test_localization_backend PRIVATE cxx_std_14)
    target_link_libraries(test_localization_backend
      xd_uav_detect_core
      ${catkin_LIBRARIES}
    )
  endif()

  add_rostest(test/ros/gimbal_range_node.test)
  # Both rostests create their own ROS master. Keep the lightweight node test
  # behind Gazebo so roscpp/Gazebo shutdown threads do not race in make -j2.
  add_dependencies(
    run_tests_xd_uav_detect_rostest_test_ros_gimbal_range_node.test
    run_tests_xd_uav_detect_rostest_test_gazebo_gimbal_range.test
  )
  catkin_add_nosetests(test/model/test_payload_model.py)
  catkin_add_nosetests(test/demo/test_demo_tools.py)
endif()
