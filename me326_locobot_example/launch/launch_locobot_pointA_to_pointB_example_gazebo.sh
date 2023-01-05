#!/bin/bash
gnome-terminal -x roslaunch me326_locobot_example locobot_gazebo_example.launch 
sleep 10
rosservice call /gazebo/unpause_physics


