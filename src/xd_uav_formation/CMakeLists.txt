cmake_minimum_required(VERSION 3.0.2)
project(xd_uav_formation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs message_generation mavros_msgs nav_msgs rospy std_msgs std_srvs
  tf tf2_geometry_msgs tf2_ros trajectory_msgs xd_uav_controller
  xd_uav_task_execute
)

catkin_python_setup()
add_service_files(FILES FormationControl.srv)
generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)

catkin_install_python(PROGRAMS
  scripts/formation_follower.py
  scripts/formation_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_core.py)
  add_rostest(test/formation.test)
  add_rostest(test/formation_fixedwing.test)
  add_rostest(test/formation_vtol.test)
  add_rostest(test/consensus.test)
  add_rostest(test/consensus_fixedwing.test)
endif()
