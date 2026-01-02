from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='assignment2_rt',
            executable='safety_detector',
            name='safety_detector'
        ),
        Node(
            package='assignment2_rt',
            executable='rollback_controller',
            name='rollback_controller'
        ),
        Node(
            package='assignment2_rt',
            executable='cmd_mux',
            name='cmd_mux'
        )
    ])

