cmake_minimum_required(VERSION 3.5)
project(xd_uav_ego_bridge)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  xd_uav_controller
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
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  xd_uav_controller
)

catkin_install_python(PROGRAMS
  scripts/ego_bridge_node.py
  scripts/ego_pointcloud_adapter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_bridge_core.py)
  catkin_add_nosetests(test/test_pointcloud_core.py)
  add_rostest(test/ego_bridge_node.test)
  add_rostest(test/ego_pointcloud_adapter.test)
  add_rostest(test/single_chain.test)
endif()
