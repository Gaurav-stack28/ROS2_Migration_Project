from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    carbot_gazebo_share = FindPackageShare('carbot_gazebo')

    carbot_world_launch = PathJoinSubstitution([
        carbot_gazebo_share,
        'launch',
        'carbot_world.launch.py'
    ])

    return LaunchDescription([

        # Launch Gazebo world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(carbot_world_launch)
        ),

        # Run the sequence script
        Node(
            package='catvehicle',
            executable='nav.bash',
            name='nav',
            output='screen'
        )
    ])