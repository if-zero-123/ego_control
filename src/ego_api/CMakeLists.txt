cmake_minimum_required(VERSION 3.0.2)
project(ego_api)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  image_transport
  cv_bridge
  pcl_conversions
  pcl_ros
  quadrotor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree segmentation)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ego_api
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs mavros_msgs nav_msgs sensor_msgs visualization_msgs image_transport cv_bridge pcl_conversions pcl_ros quadrotor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
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
  ${OpenCV_LIBRARIES}
)
add_dependencies(override_task_demo
  ${catkin_EXPORTED_TARGETS}
)

# ── CRAIC competition mission demo ──
add_executable(craic_competition_demo
  src/craic_competition_demo.cpp
)
target_link_libraries(craic_competition_demo
  ego_api
  ${catkin_LIBRARIES}
)
add_dependencies(craic_competition_demo
  ${catkin_EXPORTED_TARGETS}
)

# ── CRAIC MID360 pillar detector ──
add_executable(craic_pillar_detector
  src/pillar_detector_node.cpp
)
target_link_libraries(craic_pillar_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(craic_pillar_detector
  ${catkin_EXPORTED_TARGETS}
)

# ── CRAIC MID360 frame detector ──
add_executable(craic_frame_detector
  src/frame_detector_node.cpp
)
target_link_libraries(craic_frame_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(craic_frame_detector
  ${catkin_EXPORTED_TARGETS}
)

# ── CRAIC point cloud self filter ──
add_executable(craic_cloud_self_filter
  src/cloud_self_filter_node.cpp
)
target_link_libraries(craic_cloud_self_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
add_dependencies(craic_cloud_self_filter
  ${catkin_EXPORTED_TARGETS}
)

catkin_install_python(PROGRAMS
  scripts/craic_web_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
