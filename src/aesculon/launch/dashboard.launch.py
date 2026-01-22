import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aesculon',
            executable='aesculon_core',
            name='aesculon_core',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'aesculon', 'aesculon_console'],
            output='screen'
        )
    ])
