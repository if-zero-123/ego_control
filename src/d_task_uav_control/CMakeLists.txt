cmake_minimum_required(VERSION 3.0.2)
project(d_task_uav_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs rospy std_msgs
)

catkin_install_python(PROGRAMS
  scripts/fastlio_supervisor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_positioning_readiness.py)
endif()
