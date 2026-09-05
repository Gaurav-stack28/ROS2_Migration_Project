"""

Author: Jonathan Sprinkle, Sam Taylor, Alex Warren
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
IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Summary: 
========

This launch file loads the worlds and models for the catvehicle with the name catvehicle in stationary state. 

How to execute this file?
========================

roslaunch catvehicle catvehicle_skidpan.launch

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import FindPackageShare
from launch.substitutions import Command


# Author: Jonathan Sprinkle, Sam Taylor, Alex Warren
# Copyright (c) 2015 Arizona Board of Regents
#
# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that the
# above copyright notice and the following two paragraphs appear in
# all copies of this software.
#
# IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
# ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
# IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES...
#
# Summary:
# This launch file loads the worlds and models for the catvehicle
# with the name catvehicle in stationary state.
#
# How to execute this file?
# roslaunch catvehicle catvehicle_skidpan.launch


def generate_launch_description():

    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    debug = LaunchConfiguration('debug')
    obstaclestopper = LaunchConfiguration('obstaclestopper')

    world_file = PathJoinSubstitution([
        FindPackageShare('catvehicle'),
        'worlds',
        'skidpan.world'
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('catvehicle'),
        'urdf',
        'catvehicle.xacro'
    ])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' roboname:=catvehicle'
        ]),
        value_type=str
    )

    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])

    catvehicle_launch = PathJoinSubstitution([
        FindPackageShare('catvehicle'),
        'launch',
        'catvehicle.launch.py'
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'paused',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'gui',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'headless',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'debug',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'obstaclestopper',
            default_value='true'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': world_file,
                'gui': gui,
                'pause': paused,
                'verbose': debug
            }.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='catvehicle',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(catvehicle_launch),
            launch_arguments={
                'robot_name': 'catvehicle',
                'init_pose': '-x 1 -y 1 -z 0',
                'config_file': 'catvehicle_control.yaml',
                'obstaclestopper': obstaclestopper
            }.items()
        )

        # Uncomment this to get immediate motion from the car
        # Node(
        #     package='safeopenloopcircle',
        #     executable='safeopenloopcircle_node',
        #     name='openLoopCircle'
        # )
    ])