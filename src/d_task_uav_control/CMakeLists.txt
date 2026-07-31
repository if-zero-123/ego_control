cmake_minimum_required(VERSION 3.0.2)
project(d_task_uav_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  ego_api
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS cv_bridge ego_api image_transport nav_msgs roscpp rospy sensor_msgs std_msgs
  DEPENDS OpenCV
)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# The competition path intentionally contains one flight-control executable:
# raw AprilTag ID 0 centre -> horizontal PID -> mode-specific release or
# visual landing state machine -> return home.
add_executable(d_task_mission_node src/d_task_mission_node.cpp)
target_link_libraries(d_task_mission_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  jsoncpp_lib
)

catkin_install_python(PROGRAMS
  scripts/fastlio_supervisor_node.py
  scripts/payload_gpio_node.py
  scripts/payload_gpio_test_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/start_d_task_uav.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS d_task_mission_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
