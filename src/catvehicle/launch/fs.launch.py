"""

Author: Rahul Kumar Bhadani
Copyright (c) 2018 Arizona Board of Regents
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    leader = LaunchConfiguration('leader')
    v_ref = LaunchConfiguration('v_ref')
    d_relative = LaunchConfiguration('d_relative')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='catvehicle'),
        DeclareLaunchArgument('leader', default_value='toyota'),
        DeclareLaunchArgument('v_ref', default_value='30.0'),
        DeclareLaunchArgument(
            'd_relative',
            default_value='/catvehicle/distanceEstimator/dist'
        ),

        Node(
            package='fs',
            executable='fs_node',
            name=['fs_', robot],
            namespace=robot,
            output='screen',
            remappings=[
                ('d_relative', d_relative),
                ('cmd_vel', 'cmd_control_vel')
            ]
        ),

        Node(
            package='hoffmansubsystem',
            executable='hoffmansubsystem_node',
            name=['hoffmansubsystem_', robot],
            namespace=robot,
            output='screen'
        )
    ])