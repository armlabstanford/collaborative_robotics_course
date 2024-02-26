#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution



def generate_launch_description():

    # Define model file paths using substitution
    blue_cube_model_path_1 = PathJoinSubstitution([get_package_share_directory('locobot_autonomy'),'model','blue_cube.urdf'])
    blue_cube_model_path_2 = PathJoinSubstitution([get_package_share_directory('locobot_autonomy'),'model','blue_cube.urdf'])
    blue_cube_model_path_3 = PathJoinSubstitution([get_package_share_directory('locobot_autonomy'),'model','blue_cube.urdf'])

    red_cube_model_path_1 = PathJoinSubstitution([get_package_share_directory('locobot_autonomy'),'model','red_cube.urdf'])
    red_cube_model_path_2 = PathJoinSubstitution([get_package_share_directory('locobot_autonomy'),'model','red_cube.urdf'])
   
    green_cube_model_path_1 = PathJoinSubstitution([get_package_share_directory('locobot_autonomy'),'model','green_cube.urdf'])
    green_cube_model_path_2 = PathJoinSubstitution([get_package_share_directory('locobot_autonomy'),'model','green_cube.urdf'])
   
    yellow_cube_model_path_1 = PathJoinSubstitution([get_package_share_directory('locobot_autonomy'),'model','yellow_cube.urdf'])
    yellow_cube_model_path_2 = PathJoinSubstitution([get_package_share_directory('locobot_autonomy'),'model','yellow_cube.urdf'])

    return LaunchDescription([
        # Declare launch arguments - positions of blocks
        #blue cube 1:
        DeclareLaunchArgument('blue_1_cube_x', default_value='1.8'),
        DeclareLaunchArgument('blue_1_cube_y', default_value='0.3'),
        DeclareLaunchArgument('blue_1_cube_z', default_value='0.1'),

        #blue cube 2:
        DeclareLaunchArgument('blue_2_cube_x', default_value='1.8'),
        DeclareLaunchArgument('blue_2_cube_y', default_value='0.5'),
        DeclareLaunchArgument('blue_2_cube_z', default_value='0.1'),

        #blue cube 3:
        DeclareLaunchArgument('blue_3_cube_x', default_value='1.9'),
        DeclareLaunchArgument('blue_3_cube_y', default_value='0.4'),
        DeclareLaunchArgument('blue_3_cube_z', default_value='0.1'),

        #red cube 1:
        DeclareLaunchArgument('red_1_cube_x', default_value='1.5'),
        DeclareLaunchArgument('red_1_cube_y', default_value='0.3'),
        DeclareLaunchArgument('red_1_cube_z', default_value='0.1'),

        #red cube 2:
        DeclareLaunchArgument('red_2_cube_x', default_value='1.5'),
        DeclareLaunchArgument('red_2_cube_y', default_value='0.5'),
        DeclareLaunchArgument('red_2_cube_z', default_value='0.1'),

        #green cube 1:
        DeclareLaunchArgument('green_1_cube_x', default_value='1.5'),
        DeclareLaunchArgument('green_1_cube_y', default_value='-0.3'),
        DeclareLaunchArgument('green_1_cube_z', default_value='0.1'),

        #green cube 2:
        DeclareLaunchArgument('green_2_cube_x', default_value='1.5'),
        DeclareLaunchArgument('green_2_cube_y', default_value='-0.5'),
        DeclareLaunchArgument('green_2_cube_z', default_value='0.1'),

        #yellow cube 1:
        DeclareLaunchArgument('yellow_1_cube_x', default_value='1.8'),
        DeclareLaunchArgument('yellow_1_cube_y', default_value='-0.3'),
        DeclareLaunchArgument('yellow_1_cube_z', default_value='0.1'),

        #yellow cube 2:
        DeclareLaunchArgument('yellow_2_cube_x', default_value='1.8'),
        DeclareLaunchArgument('yellow_2_cube_y', default_value='-0.5'),
        DeclareLaunchArgument('yellow_2_cube_z', default_value='0.1'),

        # Spawn blue cube 1
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'blue_cube_1',
                '-file', blue_cube_model_path_1, 
                '-x', LaunchConfiguration('blue_1_cube_x'),
                '-y', LaunchConfiguration('blue_1_cube_y'),
                '-z', LaunchConfiguration('blue_1_cube_z'),
                '-R', '0.0', '-P', '0.0', '-Y', '0.0'
            ],
            output='screen'
        ),

        # Spawn blue cube 2
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'blue_cube_2',
                '-file', blue_cube_model_path_2, 
                '-x', LaunchConfiguration('blue_2_cube_x'),
                '-y', LaunchConfiguration('blue_2_cube_y'),
                '-z', LaunchConfiguration('blue_2_cube_z'),
                '-R', '0.0', '-P', '0.0', '-Y', '0.0'
            ],
            output='screen'
        ),

        # Spawn blue cube 3
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'blue_cube_3',
                '-file', blue_cube_model_path_3, 
                '-x', LaunchConfiguration('blue_3_cube_x'),
                '-y', LaunchConfiguration('blue_3_cube_y'),
                '-z', LaunchConfiguration('blue_3_cube_z'),
                '-R', '0.0', '-P', '0.0', '-Y', '0.0'
            ],
            output='screen'
        ),


        # Spawn red cube 1
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'red_cube_1',
                '-file', red_cube_model_path_1, 
                '-x', LaunchConfiguration('red_1_cube_x'),
                '-y', LaunchConfiguration('red_1_cube_y'),
                '-z', LaunchConfiguration('red_1_cube_z'),
                '-R', '0.0', '-P', '0.0', '-Y', '0.0'
            ],
            output='screen'
        ),

        # Spawn red cube 2
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'red_cube_2',
                '-file', red_cube_model_path_2, 
                '-x', LaunchConfiguration('red_2_cube_x'),
                '-y', LaunchConfiguration('red_2_cube_y'),
                '-z', LaunchConfiguration('red_2_cube_z'),
                '-R', '0.0', '-P', '0.0', '-Y', '0.0'
            ],
            output='screen'
        ),


        # Spawn green cube 1
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'green_cube_1',
                '-file', green_cube_model_path_1, 
                '-x', LaunchConfiguration('green_1_cube_x'),
                '-y', LaunchConfiguration('green_1_cube_y'),
                '-z', LaunchConfiguration('green_1_cube_z'),
                '-R', '0.0', '-P', '0.0', '-Y', '0.0'
            ],
            output='screen'
        ),

        # Spawn green cube 2
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'green_cube_2',
                '-file', green_cube_model_path_2, 
                '-x', LaunchConfiguration('green_2_cube_x'),
                '-y', LaunchConfiguration('green_2_cube_y'),
                '-z', LaunchConfiguration('green_2_cube_z'),
                '-R', '0.0', '-P', '0.0', '-Y', '0.0'
            ],
            output='screen'
        ),

        # Spawn yellow cube 1
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'yellow_cube_1',
                '-file', yellow_cube_model_path_1, 
                '-x', LaunchConfiguration('yellow_1_cube_x'),
                '-y', LaunchConfiguration('yellow_1_cube_y'),
                '-z', LaunchConfiguration('yellow_1_cube_z'),
                '-R', '0.0', '-P', '0.0', '-Y', '0.0'
            ],
            output='screen'
        ),

        # Spawn yellow cube 2
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'yellow_cube_2',
                '-file', yellow_cube_model_path_2, 
                '-x', LaunchConfiguration('yellow_2_cube_x'),
                '-y', LaunchConfiguration('yellow_2_cube_y'),
                '-z', LaunchConfiguration('yellow_2_cube_z'),
                '-R', '0.0', '-P', '0.0', '-Y', '0.0'
            ],
            output='screen'
        ),


    ])
