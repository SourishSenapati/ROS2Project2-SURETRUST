import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'aesculon'
    
    config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'reactor_params.yaml'
    )

    return LaunchDescription([
        # 1. Physics Core (The Digital Twin Backend)
        Node(
            package=pkg_name,
            executable='aesculon_core',
            name='aesculon_core',
            output='screen',
            parameters=[config]
        ),
        
        # 2. Vision System (The Distributed HMI)
        Node(
            package=pkg_name,
            executable='aesculon_vision',
            name='aesculon_vision',
            output='screen'
        )
    ])
