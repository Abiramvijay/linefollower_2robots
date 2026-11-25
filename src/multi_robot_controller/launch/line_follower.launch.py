#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    show_controls = LaunchConfiguration('show_controls')

    robot1 = Node(
        package='multi_robot_controller',
        executable='line_follower_node',
        name='robot1_line_follower',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'robot_name': 'robot1',
            'line_color': 'red',
            'camera_topic': '/robot1/camera/image',
            'cmd_vel_topic': '/robot1/cmd_vel',
            'odom_topic': '/robot1/odom',
            'status_topic': '/robot1/status',
            'start_topic': '/start_race',
            'base_linear_velocity': 0.18,
            'max_angular_velocity': 1.5,
            'kp': 1.0,
            'ki': 0.01,
            'kd': 0.08,
            'controller_dt': 0.05,
            'line_missing_timeout': 1.2,
            'path_length': 24.9,
            'track_min_area': 80,
            'max_linear_speed': 0.22,
        }],
    )

    robot2 = Node(
        package='multi_robot_controller',
        executable='line_follower_node',
        name='robot2_line_follower',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'robot_name': 'robot2',
            'line_color': 'blue',
            'camera_topic': '/robot2/camera/image',
            'cmd_vel_topic': '/robot2/cmd_vel',
            'odom_topic': '/robot2/odom',
            'status_topic': '/robot2/status',
            'start_topic': '/start_race',
            'base_linear_velocity': 0.18,
            'max_angular_velocity': 1.5,
            'kp': 1.0,
            'ki': 0.01,
            'kd': 0.08,
            'controller_dt': 0.05,
            'line_missing_timeout': 1.2,
            'path_length': 24.9,
            'track_min_area': 80,
            'max_linear_speed': 0.22,
        }],
    )

    controls = Node(
        package='multi_robot_controller',
        executable='race_controls',
        name='race_controls',
        output='screen',
        condition=IfCondition(show_controls),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'show_controls',
            default_value='true',
            description='Show the clickable start and stop window.',
        ),
        robot1,
        robot2,
        controls,
    ])
