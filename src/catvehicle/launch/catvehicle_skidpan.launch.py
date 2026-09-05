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
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    debug = LaunchConfiguration('debug')
    obstaclestopper = LaunchConfiguration('obstaclestopper')

    catvehicle_share = FindPackageShare('catvehicle')
    gazebo_share = FindPackageShare('gazebo_ros')

    world_path = PathJoinSubstitution([
        catvehicle_share,
        'worlds',
        'skidpan.world'
    ])

    catvehicle_launch = PathJoinSubstitution([
        catvehicle_share,
        'launch',
        'catvehicle.launch.py'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('obstaclestopper', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    gazebo_share,
                    'launch',
                    'gazebo.launch.py'
                ])
            ),
            launch_arguments={
                'world': world_path,
                'gui': gui,
                'pause': paused,
                'verbose': debug
            }.items()
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
    ])