from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='catvehicle_demo',
            executable='catvehicle_demo_node',
            name='catvehicle_demo',
            output='screen',
            parameters=[
                {'linearvel': 9.0}
            ]
        )
    ])