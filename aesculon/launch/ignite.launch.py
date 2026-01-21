from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='aesculon', executable='aesculon_core', name='aesculon_core', output='screen'),
        Node(package='aesculon', executable='aesculon_console', name='aesculon_console', output='screen'),
    ])
