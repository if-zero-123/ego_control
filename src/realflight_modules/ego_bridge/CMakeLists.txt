cmake_minimum_required(VERSION 2.8.3)
project(ego_bridge)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  quadrotor_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ---- ego_bridge_node ----
add_executable(ego_bridge_node src/ego_bridge_node.cpp)
add_dependencies(ego_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ego_bridge_node ${catkin_LIBRARIES})

# ---- rc_commander_node ----
add_executable(rc_commander_node src/rc_commander_node.cpp)
add_dependencies(rc_commander_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rc_commander_node ${catkin_LIBRARIES})
