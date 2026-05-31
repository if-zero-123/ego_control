cmake_minimum_required(VERSION 3.0.2)
project(craic_pillar_debug)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
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
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs image_transport cv_bridge pcl_conversions pcl_ros quadrotor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pillar_debug_node
  src/pillar_debug_node.cpp
)

target_link_libraries(pillar_debug_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(pillar_debug_node
  ${catkin_EXPORTED_TARGETS}
)

add_executable(takeoff_align_land_demo
  src/takeoff_align_land_demo.cpp
)

target_link_libraries(takeoff_align_land_demo
  ${catkin_LIBRARIES}
)

add_dependencies(takeoff_align_land_demo
  ${catkin_EXPORTED_TARGETS}
)

add_executable(frame_debug_node
  src/frame_debug_node.cpp
)

target_link_libraries(frame_debug_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(frame_debug_node
  ${catkin_EXPORTED_TARGETS}
)

add_executable(frame_cloud_debug_node
  src/frame_cloud_debug_node.cpp
)

target_link_libraries(frame_cloud_debug_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(frame_cloud_debug_node
  ${catkin_EXPORTED_TARGETS}
)

add_executable(frame_cross_demo_node
  src/frame_cross_demo_node.cpp
)

target_link_libraries(frame_cross_demo_node
  ${catkin_LIBRARIES}
)

add_dependencies(frame_cross_demo_node
  ${catkin_EXPORTED_TARGETS}
)
