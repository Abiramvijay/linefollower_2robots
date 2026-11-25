#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    description_share = get_package_share_directory('multi_robot_description')
    controller_share = get_package_share_directory('multi_robot_controller')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'multi_robot_world.launch.py')
        ),
        launch_arguments={
            'headless': LaunchConfiguration('headless'),
        }.items(),
    )
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'line_follower.launch.py')
        ),
        launch_arguments={
            'show_controls': LaunchConfiguration('show_controls'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without its GUI.',
        ),
        DeclareLaunchArgument(
            'show_controls',
            default_value='true',
            description='Show the clickable start, stop, and return-home controls.',
        ),
        simulation,
        controllers,
    ])
