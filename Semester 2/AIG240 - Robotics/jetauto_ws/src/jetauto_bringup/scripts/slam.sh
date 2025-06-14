#!/bin/bash
# 启动gmapping建图
gnome-terminal \
--tab -e "bash -c 'source $HOME/.jetautorc;sudo systemctl stop start_app_node;killall -9 rosmaster;roslaunch jetauto_slam slam.launch robot_name:=/ master_name:=/'" \
--tab -e "bash -c 'source $HOME/.jetautorc;sleep 25;roscd jetauto_slam/rviz;rviz rviz -d gmapping_desktop.rviz'" \
--tab -e "bash -c 'source $HOME/.jetautorc;sleep 25;roslaunch jetauto_peripherals teleop_key_control.launch robot_name:=/'" \
--tab -e "bash -c 'source $HOME/.jetautorc;sleep 25;rosrun jetauto_slam map_save.py'"
