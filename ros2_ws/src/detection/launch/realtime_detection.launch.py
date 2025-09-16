#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Default parameter file path
    default_param_path = os.path.join(
        get_package_share_directory('detection'),
        'config',
        'realtime_detection.yaml'
    )

    param_file = LaunchConfiguration('param_file')
    device_arg = LaunchConfiguration('device') 
    task_arg = LaunchConfiguration('task')

    return LaunchDescription([
        # Allow overriding the YAML via param_file:=...
        DeclareLaunchArgument(
            'param_file',
            default_value=TextSubstitution(text=default_param_path),
            description='Path to the YAML parameter file for realtime_detection_node'
        ),

        DeclareLaunchArgument(
            'device',
            default_value=TextSubstitution(text='auto'),
            description='cpu | cuda | auto'
        ),

        DeclareLaunchArgument(
            'task',
            default_value=TextSubstitution(text='instance'),
            description='bbox | instance | keypoint | panoptic'
        ),

        Node(
            package='detection',
            executable='realtime_detection_node',
            name='realtime_detection_node',
            output='screen',
            parameters=[param_file, {'device': device_arg, 'task': task_arg}]
        )
    ])
