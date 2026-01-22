from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_project2',
            executable='mission_server',
            name='mission_server',
            output='screen',
            emulate_tty=True
        ),
    ])
