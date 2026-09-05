from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot = LaunchConfiguration('robot')
    dist_topic = LaunchConfiguration('dist_topic')
    rvel_topic = LaunchConfiguration('rvel_topic')

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot',
            default_value='catvehicle'
        ),

        DeclareLaunchArgument(
            'dist_topic',
            default_value='/catvehicle/distance'
        ),

        DeclareLaunchArgument(
            'rvel_topic',
            default_value='/catvehicle/relative_vel'
        ),

        Node(
            package='catvehicle',
            executable='velocityEstimator',
            name='velocityEstimator',
            output='screen',
            parameters=[
                {
                    'dist_topic': dist_topic,
                    'vel_topic': rvel_topic
                }
            ]
        )
    ])