cmake_minimum_required(VERSION 3.0.2)
project(pod_tracker_adapter)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  roscpp
  tracker
  pod_msgs
  xmlrpcpp
)

catkin_package(CATKIN_DEPENDS diagnostic_msgs roscpp tracker pod_msgs xmlrpcpp)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(pod_tracker_adapter_node src/pod_tracker_adapter_node.cpp)
add_dependencies(pod_tracker_adapter_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_compile_features(pod_tracker_adapter_node PRIVATE cxx_std_14)
target_link_libraries(pod_tracker_adapter_node ${catkin_LIBRARIES})

install(TARGETS pod_tracker_adapter_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config/ launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  test/test_tracker_adapter.py
  test/test_tracker_product_integration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/tracker_adapter.test)
  add_rostest(test/tracker_product_integration.test)
  add_rostest(test/body_los_compensation.test)
endif()
