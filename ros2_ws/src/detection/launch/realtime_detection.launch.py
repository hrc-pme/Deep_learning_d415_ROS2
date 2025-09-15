#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 預設參數檔路徑（假設放在 detection/config/realtime_detection.yaml）
    default_param_path = os.path.join(
        get_package_share_directory('detection'),
        'config',
        'realtime_detection.yaml'
    )

    param_file = LaunchConfiguration('param_file')
    device_arg = LaunchConfiguration('device') 

    return LaunchDescription([
        # 允許用戶以 param_file:=... 指定自訂 YAML
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

        Node(
            package='detection',
            executable='realtime_detection_node',
            name='realtime_detection_node',
            output='screen',
            # 直接把 YAML 路徑丟進 parameters
            parameters=[param_file, {'device': device_arg}] 
        )
    ])
