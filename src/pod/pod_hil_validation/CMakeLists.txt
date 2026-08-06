cmake_minimum_required(VERSION 3.0.2)
project(pod_hil_validation)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  diagnostic_updater
  roscpp
  std_srvs
  tracker
  pod_msgs
)

catkin_package(CATKIN_DEPENDS diagnostic_msgs diagnostic_updater roscpp std_srvs tracker pod_msgs)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(pod_hil_validation_node src/pod_hil_validation_node.cpp)
add_dependencies(pod_hil_validation_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_compile_features(pod_hil_validation_node PRIVATE cxx_std_14)
target_link_libraries(pod_hil_validation_node ${catkin_LIBRARIES})
install(TARGETS pod_hil_validation_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config/ launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} OPTIONAL)
