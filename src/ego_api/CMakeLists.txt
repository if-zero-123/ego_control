cmake_minimum_required(VERSION 3.0.2)
project(ego_api)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ego_api
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs quadrotor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ── Library ──
add_library(ego_api
  src/ego_api.cpp
)
target_link_libraries(ego_api
  ${catkin_LIBRARIES}
)
add_dependencies(ego_api
  ${catkin_EXPORTED_TARGETS}
)

# ── Main flight demo ──
add_executable(ego_api_demo
  src/ego_api_demo.cpp
)
target_link_libraries(ego_api_demo
  ego_api
  ${catkin_LIBRARIES}
)
add_dependencies(ego_api_demo
  ${catkin_EXPORTED_TARGETS}
)

# ── Override task demo ──
add_executable(override_task_demo
  src/override_task_demo.cpp
)
target_link_libraries(override_task_demo
  ego_api
  ${catkin_LIBRARIES}
)
add_dependencies(override_task_demo
  ${catkin_EXPORTED_TARGETS}
)
