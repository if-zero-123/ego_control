#!/bin/bash
# land.sh — 发送降落命令到 ego_bridge
rostopic pub -1 /ego_bridge/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
