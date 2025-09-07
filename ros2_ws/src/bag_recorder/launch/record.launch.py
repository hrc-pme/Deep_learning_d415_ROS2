#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    return [
        LogInfo(msg=f'[bag_recorder] Using config: {config_file}'),
        ExecuteProcess(
            cmd=['ros2', 'run', 'bag_recorder', 'recorder', '-c', config_file,],
            output='screen',
            shell=False
        )
    ]

def generate_launch_description():
    default_cfg = PathJoinSubstitution([
        FindPackageShare('bag_recorder'),
        'config',
        'bag_recorder.yaml'   # ← 你的檔名
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_cfg,
            description='Path to the YAML config file for rosbag recording.'
        ),
        OpaqueFunction(function=launch_setup),
    ])
