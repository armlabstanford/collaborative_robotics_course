#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'),
        '/navigation_launch.py'])
    )
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch'),
        '/online_async_launch.py'])
    )

    return LaunchDescription([
        # Node(
        #     package='locobot_autonomy',
        #     # namespace='turtlesim1',
        #     executable='move_A_to_B_py.py',
        #     name='simple_move_A_to_B_py'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['0', '0', '0', '0', '0', '0', '1', 'locobot/base_footprint', 'locobot/base_link']
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments = ['0', '0', '0', '0', '0', '0', '1', '/odom', 'locobot/base_link']
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', '1', '/odom', 'base_footprint']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', '1', 'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0', '0', '0', '0', '0', '0', '1', 'base_link', 'locobot/base_link']
        ),
        nav_node,
        slam_node
    ])
