gnome-terminal -x roslaunch interbotix_xslocobot_moveit_interface xslocobot_moveit_interface.launch robot_model:=locobot_wx250s show_lidar:=true use_actual:=true dof:=6
sleep 6
gnome-terminal -x roslaunch me326_locobot_example gazebo_moveit_example.launch 


