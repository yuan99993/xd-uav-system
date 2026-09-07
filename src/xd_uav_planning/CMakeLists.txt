cmake_minimum_required(VERSION 3.5)
project(xd_uav_planning)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  xd_uav_controller
  xd_uav_task_allocate
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  diagnostic_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  xd_uav_controller
  xd_uav_task_allocate
)

catkin_install_python(PROGRAMS
  scripts/ego_bridge_node.py
  scripts/ego_pointcloud_adapter_node.py
  scripts/ego_status_node.py
  scripts/fixedwing_path_backend_node.py
  scripts/demo/fixedwing_path_acceptance.py
  scripts/health_gate_node.py
  scripts/reference_mux_node.py
  scripts/select_owner_when_ready.py
  scripts/state_hold_baseline_node.py
  scripts/demo/wait_ego_swarm_ready.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/demo/ego_demo.sh
  scripts/demo/ego_obstacle_demo.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config docs launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_bridge_core.py)
  catkin_add_nosetests(test/test_pointcloud_core.py)
  catkin_add_nosetests(test/test_planning_core.py)
  add_rostest(test/ego_bridge_node.test)
  add_rostest(test/ego_pointcloud_adapter.test)
  add_rostest(test/ego_status_node.test)
  add_rostest(test/single_chain.test)
  add_rostest(test/reference_integration.test)
  add_rostest(test/fixedwing_path_backend.test)
endif()
