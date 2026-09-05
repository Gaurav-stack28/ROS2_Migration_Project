"""

Author: Rahul Kumar Bhadani
Copyright (c) 2020 Arizona Board of Regents
All rights reserved.

Permission is hereby granted, without written agreement and without 
license or royalty fees, to use, copy, modify, and distribute this
software and its documentation for any purpose, provided that the 
above copyright notice and the following two paragraphs appear in 
all copies of this software.

IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY 
FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES 
ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN 
IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.

THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, 
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    catvehicle_share = FindPackageShare('catvehicle')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='catvehicle'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    catvehicle_share,
                    'launch',
                    'catvehicle_empty.launch.py'
                ])
            )
        ),

        Node(
            package='catvehicle',
            executable='distanceEstimator',
            name='distanceEstimator',
            namespace=robot,
            output='screen',
            parameters=[{
                'scan_topic': [
                    '/',
                    robot,
                    '/front_laser_points'
                ],
                'angle_min': -3.0,
                'angle_max': 3.0
            }]
        ),

        Node(
            package='fs',
            executable='fs_node',
            name=['fs_', robot],
            namespace=robot,
            output='screen',
            remappings=[
                (
                    'd_relative',
                    [
                        '/',
                        robot,
                        '/distanceEstimator/dist'
                    ]
                )
            ]
        )
    ])