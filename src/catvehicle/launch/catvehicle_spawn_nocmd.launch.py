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
roslaunch catvehicle catvehicle_empty.launch

How to execute this file?
========================

roslaunch catvehicle catvehicle_spawn_nocmd.launch robot1:=acar_sim X:=0 Y:=0 Z:=0 roll:=0 pitch:=0 yaw:=0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot1 = LaunchConfiguration('robot1')
    X = LaunchConfiguration('X')
    Y = LaunchConfiguration('Y')
    Z = LaunchConfiguration('Z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    obstaclestopper = LaunchConfiguration('obstaclestopper')
    catvehicle_share = FindPackageShare('catvehicle')
    xacro_file = PathJoinSubstitution([catvehicle_share, 'urdf', 'catvehicle.xacro'])
    catvehicle_launch = PathJoinSubstitution([catvehicle_share, 'launch', 'catvehicle.launch.py'])
    robot_description = Command(['xacro ', xacro_file, ' roboname:=', robot1])
    return LaunchDescription([
        DeclareLaunchArgument('robot1', default_value='catvehicle'),
        DeclareLaunchArgument('X', default_value='0'),
        DeclareLaunchArgument('Y', default_value='0'),
        DeclareLaunchArgument('Z', default_value='0'),
        DeclareLaunchArgument('roll', default_value='0'),
        DeclareLaunchArgument('pitch', default_value='0'),
        DeclareLaunchArgument('yaw', default_value='0'),
        DeclareLaunchArgument('obstaclestopper', default_value='true'),
        GroupAction([
            Node(package='robot_state_publisher', executable='robot_state_publisher', name=['robot_state_publisher', robot1], output='screen', parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(catvehicle_launch), launch_arguments={'robot_name': robot1, 'init_pose': ['-x ', X, ' -y ', Y, ' -z ', Z, ' -R ', roll, ' -P ', pitch, ' -Y ', yaw], 'config_file': 'catvehicle_control.yaml', 'obstaclestopper': obstaclestopper}.items())
        ])
    ])