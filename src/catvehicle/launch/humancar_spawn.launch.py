"""
Author: Jonathan Sprinkle, Sam Taylor, Alex Warren, Rahul Bhadani
Copyright (c) 2015-2020 Arizona Board of Regents
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

Summary: 
=======

This launch file loads the autonomous car in stationary state into the world"

Prerequisite:
=============
1. Load the world
roslaunch humancar humancar_empty.launch

How to execute this file?
========================

roslaunch humancar humancar_spawn.launch robot:=acar_sim X:=0 Y:=0 Z:=0 roll:=0 pitch:=0 yaw:=0

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot = LaunchConfiguration('robot')
    X = LaunchConfiguration('X')
    Y = LaunchConfiguration('Y')
    Z = LaunchConfiguration('Z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    catvehicle_share = FindPackageShare('catvehicle')
    xacro_file = PathJoinSubstitution([catvehicle_share, 'urdf', 'humancar.xacro'])

    return LaunchDescription([

        DeclareLaunchArgument('robot', default_value='humancar'),
        DeclareLaunchArgument('X', default_value='0'),
        DeclareLaunchArgument('Y', default_value='0'),
        DeclareLaunchArgument('Z', default_value='0'),
        DeclareLaunchArgument('roll', default_value='0'),
        DeclareLaunchArgument('pitch', default_value='0'),
        DeclareLaunchArgument('yaw', default_value='0'),

        GroupAction([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name=['robot_state_publisher_', robot],
                output='screen',
                parameters=[{
                    'robot_description': ''
                }]
            ),

            Node(
                package='xacro',
                executable='xacro',
                name=['xacro_', robot],
                output='screen'
            )
        ])
    ])