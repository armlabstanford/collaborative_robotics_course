#!/bin/bash

# gnome-terminal -x roslaunch interbotix_xslocobot_gazebo xslocobot_gazebo.launch robot_model:=locobot_wx250s show_lidar:=true use_trajectory_controllers:=true 

gnome-terminal -x roslaunch me326_locobot_example locobot_gazebo_example.launch 
sleep 10
rosservice call /gazebo/unpause_physics


