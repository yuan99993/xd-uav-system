cmake_minimum_required(VERSION 3.5)
project(xd_uav_detect)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  xd_uav_track
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xd_uav_detect_core
  CATKIN_DEPENDS
    cv_bridge
    image_geometry
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
    xd_uav_track
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(xd_uav_detect_core
  src/lidar_camera_fusion.cpp
)
target_compile_features(xd_uav_detect_core PUBLIC cxx_std_14)
target_link_libraries(xd_uav_detect_core ${catkin_LIBRARIES})

add_executable(xd_uav_detect_node
  src/xd_uav_detect_node.cpp
)
target_compile_features(xd_uav_detect_node PRIVATE cxx_std_14)
add_dependencies(xd_uav_detect_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(xd_uav_detect_node
  xd_uav_detect_core
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/red_box_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS xd_uav_detect_core xd_uav_detect_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_lidar_camera_fusion
    test/test_lidar_camera_fusion.cpp
  )
  if(TARGET test_lidar_camera_fusion)
    target_compile_features(test_lidar_camera_fusion PRIVATE cxx_std_14)
    target_link_libraries(test_lidar_camera_fusion
      xd_uav_detect_core
      ${catkin_LIBRARIES}
    )
  endif()
endif()
