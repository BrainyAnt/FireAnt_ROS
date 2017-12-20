#!/bin/bash
catkin_make -C ~/catkin_ws
source ~/catkin_ws/devel/setup.bash

export ROS_IP=$(hostname -I | awk -F " " '{print$1}')
export ROS_MASTER_URI=http://`hostname -I | awk -F " " '{print$1}'`:11311
export ANT_PKG_PATH=~/catkin_ws/src/ant

roslaunch ant ant_brain.launch