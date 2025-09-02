from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'rgb_camera.color_profile': '640,480,30',
                'depth_module.depth_profile': '640,480,30',
                'publish_tf': True,
                'pointcloud.enable': False,   # 如果你要點雲可以改 True
            }]
        )
    ])
