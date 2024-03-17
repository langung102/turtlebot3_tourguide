#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    test2 = LaunchConfiguration(
        'turtlebot3_bringup',
        default=os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch'))
    LAUNCH_FILE2 = "/robot.launch.py"

    test3 = LaunchConfiguration(
        'turtlebot3_navigation2',
        default=os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'launch'))
    LAUNCH_FILE3 = "/navigation2.launch.py"
    
    delay_action = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='turtlebot3_tourguide',
                executable='turtlebot3_tourguide',
                output='screen'),
        ]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([test3, LAUNCH_FILE3]),
            launch_arguments={'use_sim_time': 'True', 'map': 'map.yaml'}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([test2, LAUNCH_FILE2])
        ),
        delay_action,
    ])


