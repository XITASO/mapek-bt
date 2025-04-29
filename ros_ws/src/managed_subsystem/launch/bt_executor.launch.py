from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="bt_mape_k",
                namespace="managed_subsystem",
                executable="bt_executor",
                parameters=[
                    {
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
