import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    """
    AESCULON DASHBOARD LAUNCHER
    Objective: One-command bring-up of the Physics Core and Real-Time Telemetry Plot.
    """
    
    # 1. THE PHYSICS CORE (Server)
    # This runs in the background solving the differential equations.
    core_node = Node(
        package='aesculon',
        executable='aesculon_core',
        name='aesculon_core',
        output='screen'
    )

    # 2. THE VISUALIZATION (rqt_plot)
    # We pre-load the thermodynamic topics so you don't have to type them manually.
    # Topic structure: /action_name/_action/feedback/feedback/field_name
    telemetry_plot = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='thermodynamic_monitor',
        arguments=[
            '/judge_process/_action/feedback/feedback/current_temp_k',
            '/judge_process/_action/feedback/feedback/current_pressure_pa'
        ]
    )

    # 3. THE OPERATOR CONSOLE (Client)
    # We launch this in a separate terminal so you can type inputs.
    # NOTE: 'xterm' is used for broad Linux compatibility. 
    # If using standard Ubuntu, 'gnome-terminal' is also valid.
    operator_console = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2', 'run', 'aesculon', 'aesculon_console'],
        output='screen'
    )

    return LaunchDescription([
        core_node,
        telemetry_plot,
        # operator_console  <-- UNCOMMENT THIS if you have xterm installed.
        # Otherwise, run the console manually in a second tab (Safest for WSL).
    ])
