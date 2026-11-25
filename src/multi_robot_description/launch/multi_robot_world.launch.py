#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    description_dir = get_package_share_directory('multi_robot_description')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(description_dir, 'worlds', 'multi_robot_world.sdf')
    robot_model_file = os.path.join(description_dir, 'models', 'line_follower_robot', 'model.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                PythonExpression([
                    "'-s -r' if '",
                    LaunchConfiguration('headless'),
                    "' == 'true' else '-r'",
                ]),
                ' ',
                world_file,
            ],
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/robot1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/robot1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/multi_robot_world/model/robot1/link/base_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/robot2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/robot2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/multi_robot_world/model/robot2/link/base_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        remappings=[
            ('/model/robot1/cmd_vel', '/robot1/cmd_vel'),
            ('/model/robot1/odometry', '/robot1/odom'),
            ('/world/multi_robot_world/model/robot1/link/base_link/sensor/camera/image',
             '/robot1/camera/image'),
            ('/model/robot2/cmd_vel', '/robot2/cmd_vel'),
            ('/model/robot2/odometry', '/robot2/odom'),
            ('/world/multi_robot_world/model/robot2/link/base_link/sensor/camera/image',
             '/robot2/camera/image'),
        ],
        output='screen',
    )

    spawn_robot1 = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-world', 'multi_robot_world',
             '-file', robot_model_file,
             '-name', 'robot1',
             '-x', '-5.3', '-y', '-3.3', '-z', '0.0', '-Y', '1.47'],
        output='screen',
    )

    spawn_robot2 = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-world', 'multi_robot_world',
             '-file', robot_model_file,
             '-name', 'robot2',
             '-x', '-5.3', '-y', '-2.55', '-z', '0.0', '-Y', '1.47'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without its GUI.',
        ),
        gz_sim,
        bridge,
        TimerAction(
            period=5.0,
            actions=[
                spawn_robot1,
                spawn_robot2,
            ],
        ),
    ])
