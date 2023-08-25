#!/bin/bash

gnome-terminal -x roslaunch interbotix_xslocobot_gazebo xslocobot_gazebo.launch robot_model:=locobot_wx250s show_lidar:=true use_position_controllers:=true dof:=6 
sleep 10
rosservice call /gazebo/unpause_physics
##### gnome-terminal -x roslaunch interbotix_xslocobot_gazebo xslocobot_gazebo.launch robot_model:=locobot_wx250s show_lidar:=true use_trajectory_controllers:=true 


