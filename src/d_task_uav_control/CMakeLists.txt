cmake_minimum_required(VERSION 3.0.2)
project(d_task_uav_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  ego_api
  image_transport
  mavros_msgs
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
  INCLUDE_DIRS include
  LIBRARIES d_task_fixed_height_drop_flow d_task_payload_pulse
  CATKIN_DEPENDS cv_bridge ego_api image_transport mavros_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
  DEPENDS OpenCV
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(d_task_fixed_height_drop_flow
  src/fixed_height_drop_flow.cpp
)
target_link_libraries(d_task_fixed_height_drop_flow ${catkin_LIBRARIES})

add_library(d_task_payload_pulse
  src/payload_pulse.cpp
)
target_link_libraries(d_task_payload_pulse ${catkin_LIBRARIES})

# Competition DROP path: raw AprilTag ID 0 centre -> horizontal PID ->
# fixed-height release -> return-home landing.
add_executable(d_task_mission_node src/d_task_mission_node.cpp)
target_link_libraries(d_task_mission_node
  d_task_fixed_height_drop_flow
  d_task_payload_pulse
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  jsoncpp_lib
)

catkin_install_python(PROGRAMS
  scripts/calibrate_tag0_offset.py
  scripts/fastlio_supervisor_node.py
  scripts/payload_gpio_test_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/start_d_task_uav.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS d_task_fixed_height_drop_flow d_task_payload_pulse
                d_task_mission_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_fixed_height_drop_flow
    test/test_fixed_height_drop_flow.cpp)
  if(TARGET test_fixed_height_drop_flow)
    target_link_libraries(test_fixed_height_drop_flow
      d_task_fixed_height_drop_flow ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_payload_pulse test/test_payload_pulse.cpp)
  if(TARGET test_payload_pulse)
    target_link_libraries(test_payload_pulse
      d_task_payload_pulse ${catkin_LIBRARIES})
  endif()

  catkin_add_nosetests(test/test_tag0_offset_calibration.py)
  catkin_add_nosetests(test/test_tag0_offset_calibration_launch.py)
  catkin_add_nosetests(test/test_mission_diagnostics_contract.py)
  catkin_add_nosetests(test/test_mission_config_sender_contract.py)
endif()
