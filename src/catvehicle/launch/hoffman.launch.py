from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot = LaunchConfiguration('robot')

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot',
            default_value='catvehicle'
        ),

        Node(
            package='hoffmansubsystem',
            executable='hoffmansubsystem_node',
            name=['hoffmannsubsystem_', robot],
            output='screen',
            remappings=[
                ('/cmd_control_vel', ['/', robot, '/cmd_control_vel']),
                ('/timer_companion/do_publish', ['/', robot, '/timer_companion/do_publish'])
            ]
        )
    ])