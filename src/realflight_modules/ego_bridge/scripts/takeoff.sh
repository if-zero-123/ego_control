#!/bin/bash
# takeoff.sh — 发送起飞命令到 ego_bridge
rostopic pub -1 /ego_bridge/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
