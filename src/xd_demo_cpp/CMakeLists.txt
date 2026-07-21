cmake_minimum_required(VERSION 3.0.2)
project(xd_demo_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(heartbeat_node
  src/heartbeat_node.cpp
)

target_compile_features(heartbeat_node PRIVATE cxx_std_14)

target_link_libraries(heartbeat_node
  ${catkin_LIBRARIES}
)

install(TARGETS heartbeat_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
