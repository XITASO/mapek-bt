from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bt_mape_k',
            namespace='managed_system',
            executable='bt_executor'
        ),
        Node(
            package='managed_subsystem',
            namespace='managed_subsystem',
            executable='blackboard_setter'
        )
    ])