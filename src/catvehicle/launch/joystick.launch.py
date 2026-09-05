"""

Author: Jonathan Sprinkle
Copyright (c) 2015 Arizona Board of Regents
All rights reserved.

Permission is hereby granted, without written agreement and without 
license or royalty fees, to use, copy, modify, and distribute this
software and its documentation for any purpose, provided that the 
above copyright notice and the following two paragraphs appear in 
all copies of this software.

IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY 
FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES 
ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN 
IF THE ARIZONA BOARD OF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.

THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, 
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
IS ON AN "AS IS" BASIS, AND THE ARIZONA BOARD OF REGENTS HAS NO OBLIGATION
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Summary: 
=======
This launch file loads the worlds and models for the catvehicle

How to execute it:
=================
roslaunch catvehicle joystick.launch

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_name = LaunchConfiguration('robot_name')
    velmax = LaunchConfiguration('velmax')

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_name',
            default_value='catvehicle'
        ),

        DeclareLaunchArgument(
            'velmax',
            default_value='15.0'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            required=True
        ),

        Node(
            package='catvehicle',
            executable='joy2cmdvel.py',
            name='joy2cmdvel',
            output='screen',
            required=True,
            parameters=[
                {
                    'namespace': ['/', robot_name],
                    'velmax': velmax
                }
            ],
            remappings=[
                (
                    '/catvehicle/cmd_vel',
                    ['/', robot_name, '/cmd_vel']
                )
            ]
        )
    ])