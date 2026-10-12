#include <plan_env/grid_map.h>
#include <ros/ros.h>
// Map only: no flight bridge, MAVROS, payload or command publishers.
int main(int argc,char**argv) {
  ros::init(argc,argv,"cloud_fusion_probe");
  ros::NodeHandle nh("~");GridMap map;map.initMap(nh);ros::spin();return 0;
}
