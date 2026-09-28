cmake_minimum_required(VERSION 3.0.2)
project(xd_uav_control_manager)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  xd_uav_controller
  xd_uav_state_estimators
)

add_service_files(
  FILES
  SetFlightRegime.srv
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    mavros_msgs
    message_runtime
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    xd_uav_controller
    xd_uav_state_estimators
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(control_manager_node
  src/control_manager_node.cpp
  src/vtol_vehicle_adapter.cpp
)
add_dependencies(control_manager_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(control_manager_node PRIVATE cxx_std_17)
target_link_libraries(control_manager_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_control_interfaces test/test_control_interfaces.cpp)
  if(TARGET test_control_interfaces)
    add_dependencies(test_control_interfaces
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_compile_features(test_control_interfaces PRIVATE cxx_std_17)
    target_link_libraries(test_control_interfaces gtest_main ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_vtol_vehicle_adapter
    test/test_vtol_vehicle_adapter.cpp
    src/vtol_vehicle_adapter.cpp
  )
  if(TARGET test_vtol_vehicle_adapter)
    target_compile_features(test_vtol_vehicle_adapter PRIVATE cxx_std_17)
    target_link_libraries(test_vtol_vehicle_adapter gtest_main ${catkin_LIBRARIES})
  endif()
  add_rostest(test/control_manager.test)
  add_rostest(test/fixedwing_control_manager.test)
endif()

install(TARGETS control_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
