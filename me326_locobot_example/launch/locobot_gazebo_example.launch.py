#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    show_lidar = LaunchConfiguration('show_lidar')
    use_trajectory_controllers = LaunchConfiguration('use_trajectory_controllers')
    robot_model = LaunchConfiguration('robot_model')
    robot_name = 'locobot_1'

    return LaunchDescription(
        [
            DeclareLaunchArgument("show_lidar", default_value="true"),
            DeclareLaunchArgument("use_trajectory_controllers", default_value="true"),
            DeclareLaunchArgument("robot_model", default_value="locobot_wx250s"),
            
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare("interbotix_xslocobot_sim"), "launch", "xslocobot_gz_classic.launch.py"]
                ),
                launch_arguments={
                    "show_lidar": show_lidar,
                    "use_trajectory_controllers": use_trajectory_controllers,
                    "robot_model": robot_model,
                }.items(),
            ),

            Node(
                executable="locobot1_example_motion.py",
                package="me326_locobot_example",
                parameters=[
                    {"respawn": "true"},
                ],
            ),

            # Node(
            #     package='rviz2',
            #     executable='rviz2',
            #     name='rviz2',
            #     arguments=[
            #         '-d', 
            #         PathJoinSubstitution([
            #             FindPackageShare("me326_locobot_example"), "rviz", "rviz_example_env.rviz"]
            #         )
            #     ],
            # ),
        ]
    )