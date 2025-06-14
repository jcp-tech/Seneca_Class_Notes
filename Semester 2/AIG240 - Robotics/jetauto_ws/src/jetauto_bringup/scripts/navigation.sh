#!/bin/bash
source $HOME/.jetautorc # 加载环境
sudo systemctl stop start_app_node
killall -9 rosmaster
roslaunch jetauto_navigation navigation.launch map:=explore robot_name:=/ master_name:=/ & 
sleep 10 
roslaunch jetauto_navigation publish_point.launch robot_name:=/ master_name:=/ enable_navigation:=false &
rviz rviz -d  $HOME/jetauto_ws/src/jetauto_navigation/rviz/navigation_desktop.rviz
