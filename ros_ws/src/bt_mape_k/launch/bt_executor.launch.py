from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bt_mape_k',
            executable='bt_executor',
            namespace='managed_subsystem',
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])