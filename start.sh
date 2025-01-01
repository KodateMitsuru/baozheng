#!/bin/bash
# 将workspace设置为脚本当前所在目录
workspace=$(dirname "$0")
log_folder="$workspace/ros_log"
log_path="$log_folder/log.txt"
mkdir -p $log_folder
# 切换到workspace
cd "$workspace"
export ROS_DOMAIN_ID=7
source install/setup.bash

ros2 launch startup.py | tee $log_path 2>&1
