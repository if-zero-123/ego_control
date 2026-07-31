cmake_minimum_required(VERSION 3.0.2)
project(uav_protocol_gateway)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs rospy sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/uav_protocol_gateway_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_gateway_core.py)
endif()
