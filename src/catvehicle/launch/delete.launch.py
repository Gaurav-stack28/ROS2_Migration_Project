from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='catvehicle'),

        Node(
            package='gazebo_ros',
            executable='delete_entity.py',
            name=['delete_spawner', robot_name],
            output='screen',
            arguments=['-entity', robot_name]
        )
    ])