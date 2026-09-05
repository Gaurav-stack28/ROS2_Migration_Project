from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    gazebo = get_package_share_directory("gazebo_ros")
    pkg = get_package_share_directory("carbot_gazebo")

    world = os.path.join(pkg, "worlds", "sixth.world")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    gazebo,
                    "launch",
                    "gazebo.launch.py"
                )
            ),
            launch_arguments={
                "world": world,
                "extra_gazebo_args":
                    "-s libgazebo_ros_state.so"
            }.items()
        )
    ])