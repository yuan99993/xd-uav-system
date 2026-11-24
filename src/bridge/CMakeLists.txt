cmake_minimum_required(VERSION 3.5)
project(xd_uav_video_bridge)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs)

add_executable(video_stream_receiver video_stream_receiver.cpp)
target_compile_features(video_stream_receiver PRIVATE cxx_std_14)
target_include_directories(video_stream_receiver PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(video_stream_receiver
  ${catkin_LIBRARIES}
  PkgConfig::GSTREAMER
  ${OpenCV_LIBRARIES}
)
set_target_properties(video_stream_receiver PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
)

add_executable(h264_nal_multicast_sender h264_nal_multicast_sender.cpp)
target_compile_features(h264_nal_multicast_sender PRIVATE cxx_std_14)
target_include_directories(h264_nal_multicast_sender PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(h264_nal_multicast_sender
  ${catkin_LIBRARIES}
  PkgConfig::GSTREAMER
)
set_target_properties(h264_nal_multicast_sender PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin"
)

install(TARGETS video_stream_receiver h264_nal_multicast_sender
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
