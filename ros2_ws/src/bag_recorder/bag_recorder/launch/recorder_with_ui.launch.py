from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    cfg_arg = DeclareLaunchArgument(
        'config_yaml',
        default_value=os.path.join(
            os.path.dirname(__file__), '..', 'config', 'bag_recorder.yaml'
        ),
        description='Path to YAML config'
    )
    cfg = LaunchConfiguration('config_yaml')

    return LaunchDescription([
        cfg_arg,
        Node(
            package='bag_recorder',
            executable='recorder_node',
            name='bag_recorder',
            output='screen',
            parameters=[{'config_yaml': cfg}]
        ),
        Node(
            package='bag_recorder',
            executable='recorder_ui',
            name='bag_recorder_ui',
            output='screen'
        ),
    ])
