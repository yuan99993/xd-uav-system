cmake_minimum_required(VERSION 3.0.2)
project(xd_uav_formation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs message_generation mavros_msgs nav_msgs roscpp rospy std_msgs std_srvs
  tf tf2 tf2_geometry_msgs tf2_ros trajectory_msgs xmlrpcpp xd_uav_controller
  xd_uav_task_execute
)
find_package(yaml-cpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

catkin_python_setup()
add_service_files(FILES FormationControl.srv)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xd_uav_formation_core xd_uav_formation_runtime
  CATKIN_DEPENDS geometry_msgs mavros_msgs message_runtime nav_msgs roscpp
                 std_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros trajectory_msgs
                 xmlrpcpp xd_uav_controller xd_uav_task_execute
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(xd_uav_formation_core src/core.cpp)
target_link_libraries(xd_uav_formation_core ${catkin_LIBRARIES})
add_dependencies(xd_uav_formation_core ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_library(xd_uav_formation_runtime src/runtime.cpp)
target_link_libraries(xd_uav_formation_runtime xd_uav_formation_core
                      ${catkin_LIBRARIES})
add_dependencies(xd_uav_formation_runtime ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

add_executable(formation_manager_cpp src/manager.cpp)
target_link_libraries(formation_manager_cpp xd_uav_formation_runtime
                      xd_uav_formation_core ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(formation_manager_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

install(TARGETS xd_uav_formation_core xd_uav_formation_runtime
                formation_manager_cpp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/formation_follower.py
  scripts/formation_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_core_cpp test/test_core_cpp.cpp)
  if(TARGET test_core_cpp)
    target_link_libraries(test_core_cpp xd_uav_formation_core ${catkin_LIBRARIES})
  endif()
  catkin_add_nosetests(test/test_core.py)
  add_rostest(test/formation_cpp.test)
  add_rostest(test/formation_cpp_fixedwing.test)
  add_rostest(test/formation_cpp_vtol.test)
  add_rostest(test/consensus_cpp.test)
  add_rostest(test/consensus_cpp_fixedwing.test)
  add_rostest(test/formation.test)
  add_rostest(test/formation_fixedwing.test)
  add_rostest(test/formation_vtol.test)
  add_rostest(test/consensus.test)
  add_rostest(test/consensus_fixedwing.test)
endif()
