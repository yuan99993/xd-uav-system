cmake_minimum_required(VERSION 3.5)
project(xd_uav_planning)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geographic_msgs
  geometry_msgs
  grid_map_core
  grid_map_geo
  message_generation
  mavros_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  terrain_navigation
  terrain_planner
  xd_uav_controller
  xd_uav_task_allocate
)

find_package(ompl REQUIRED)

catkin_python_setup()

add_message_files(
  FILES
  NoFlyZone.msg
  NoFlyZoneArray.msg
  TerrainConflict.msg
  TerrainFleetStatus.msg
  TerrainFleetVehicle.msg
  TerrainSwarmStatus.msg
  TerrainTask.msg
  TerrainTaskBatch.msg
  TerrainVehicleStatus.msg
)

add_service_files(
  FILES
  RemoveTerrainVehicle.srv
  SpawnTerrainVehicle.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  xd_uav_task_allocate
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xd_uav_terrain_path_validator
  CATKIN_DEPENDS
  diagnostic_msgs
  geometry_msgs
  grid_map_core
  grid_map_geo
  message_runtime
  mavros_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  terrain_navigation
  terrain_planner
  xd_uav_controller
  xd_uav_task_allocate
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(xd_uav_terrain_path_validator
  src/terrain/terrain_path_validator.cpp
)
add_dependencies(xd_uav_terrain_path_validator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(xd_uav_terrain_path_validator ${catkin_LIBRARIES})

add_executable(terrain_planner_node
  src/terrain/terrain_planner_node.cpp
)
add_dependencies(terrain_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(terrain_planner_node
  xd_uav_terrain_path_validator
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/ego_bridge_node.py
  scripts/ego_pointcloud_adapter_node.py
  scripts/ego_status_node.py
  scripts/fixedwing_path_backend_node.py
  scripts/demo/fixedwing_path_acceptance.py
  scripts/demo/fixedwing_nofly_visualizer.py
  scripts/demo/terrain_fixedwing_swarm_acceptance.py
  scripts/runtime/deterministic_obstacle_cloud.py
  scripts/runtime/imu_frame_adapter.py
  scripts/runtime/mavros_odom_recovery_gate.py
  scripts/runtime/multi_odom_registration.py
  scripts/runtime/navsat_origin_provider.py
  scripts/runtime/terrain_swarm_release_gate.py
  scripts/runtime/terrain_fleet_manager.py
  scripts/health_gate_node.py
  scripts/reference_mux_node.py
  scripts/select_owner_when_ready.py
  scripts/state_hold_baseline_node.py
  scripts/swarm_handoff_relay.py
  scripts/terrain_swarm_monitor.py
  scripts/terrain_status_coordinator.py
  scripts/demo/wait_ego_swarm_ready.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/demo/ego_demo.sh
  scripts/demo/ego_obstacle_demo.sh
  scripts/demo/terrain_swarm_sitl_demo.sh
  scripts/runtime/fixedwing_sitl_runtime.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/tools/prepare_gazebo_dem.py
  scripts/tools/verify_gazebo_dem.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts/tools
)

install(TARGETS xd_uav_terrain_path_validator terrain_planner_node
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

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_terrain_path_validator
    test/test_terrain_path_validator.cpp)
  if(TARGET test_terrain_path_validator)
    target_link_libraries(test_terrain_path_validator
      xd_uav_terrain_path_validator
      ${catkin_LIBRARIES})
  endif()
  catkin_add_nosetests(test/test_bridge_core.py)
  catkin_add_nosetests(test/test_pointcloud_core.py)
  catkin_add_nosetests(test/test_planning_core.py)
  catkin_add_nosetests(test/test_nofly.py)
  catkin_add_nosetests(test/test_portable_runtime_configuration.py)
  catkin_add_nosetests(test/test_terrain_swarm.py)
  catkin_add_nosetests(test/test_terrain_swarm_gate.py)
  catkin_add_nosetests(test/test_terrain_fleet.py)
  add_rostest(test/ego_bridge_node.test)
  add_rostest(test/ego_pointcloud_adapter.test)
  add_rostest(test/ego_status_node.test)
  add_rostest(test/single_chain.test)
  add_rostest(test/ego_route_obstacle.test)
  add_rostest(test/reference_integration.test)
  add_rostest(test/fixedwing_path_backend.test)
  add_rostest(test/terrain_fixedwing_chain.test)
  add_rostest(test/terrain_fixedwing_swarm.test)
  add_rostest(test/terrain_fixedwing_swarm_three.test)
  add_rostest(test/swarm_handoff_relay.test)
  add_rostest(test/mavros_odom_recovery_gate.test)
endif()
