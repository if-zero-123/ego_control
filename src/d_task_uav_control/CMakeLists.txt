cmake_minimum_required(VERSION 3.0.2)
project(d_task_uav_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ego_api
  mavros_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)

catkin_python_setup()

add_message_files(FILES
  PlatformDetection.msg
  PlatformEstimate.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ego_api geometry_msgs mavros_msgs message_runtime nav_msgs roscpp rospy sensor_msgs std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(d_task_platform_tracking
  src/pixel_projector.cpp
  src/platform_tracker.cpp
)
add_dependencies(d_task_platform_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(d_task_platform_tracking ${catkin_LIBRARIES})

add_library(d_task_mission_controller
  src/mission_controller.cpp
)
target_link_libraries(d_task_mission_controller ${catkin_LIBRARIES})

add_executable(platform_tracking_fusion_node
  src/platform_tracking_fusion_node.cpp
)

add_executable(d_task_mission_node
  src/d_task_mission_node.cpp
)
add_dependencies(d_task_mission_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(d_task_mission_node
  d_task_mission_controller
  ${catkin_LIBRARIES}
  jsoncpp_lib
)
add_dependencies(platform_tracking_fusion_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(platform_tracking_fusion_node
  d_task_platform_tracking
  ${catkin_LIBRARIES}
  jsoncpp_lib
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
  catkin_add_gtest(test_platform_tracker test/test_platform_tracker.cpp)
  if(TARGET test_platform_tracker)
    target_link_libraries(test_platform_tracker d_task_platform_tracking ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_mission_controller test/test_mission_controller.cpp)
  if(TARGET test_mission_controller)
    target_link_libraries(test_mission_controller
      d_task_mission_controller
      ${catkin_LIBRARIES})
  endif()
endif()

install(TARGETS d_task_platform_tracking d_task_mission_controller platform_tracking_fusion_node d_task_mission_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
