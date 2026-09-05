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
IS ON AN "AS IS" BASIS, AND THE ARIZONA BOARD OF REGENTS HAS NO OBLIGATION
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Summary: 
This launch file loads the worlds and models for the catvehicle
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    paused = LaunchConfiguration('paused')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    debug = LaunchConfiguration('debug')
    worldfile = LaunchConfiguration('worldfile')
    front_laser_points = LaunchConfiguration('front_laser_points')
    velodyne_points = LaunchConfiguration('velodyne_points')
    camera_right = LaunchConfiguration('camera_right')
    camera_left = LaunchConfiguration('camera_left')
    velodyne_max_angle = LaunchConfiguration('velodyne_max_angle')
    velodyne_min_angle = LaunchConfiguration('velodyne_min_angle')

    catvehicle_share = FindPackageShare('catvehicle')

    world_path = PathJoinSubstitution([
        catvehicle_share,
        'worlds',
        worldfile
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
        DeclareLaunchArgument('worldfile', default_value='enter_worldfile_no_path.world'),

        DeclareLaunchArgument('front_laser_points', default_value='true'),
        DeclareLaunchArgument('velodyne_points', default_value='true'),
        DeclareLaunchArgument('camera_right', default_value='true'),
        DeclareLaunchArgument('camera_left', default_value='true'),

        DeclareLaunchArgument('velodyne_max_angle', default_value='0.4'),
        DeclareLaunchArgument('velodyne_min_angle', default_value='-0.4'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('gazebo_ros'),'launch','gazebo.launch.py'])),
            launch_arguments={
                'world': world_path,
                'gui': gui,
                'pause': paused,
                'verbose': debug
            }.items()
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time
                }
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(catvehicle_launch),
            launch_arguments={
                'robot_name': 'catvehicle',
                'init_pose': '-x 0 -y 0 -z 0',
                'config_file': 'catvehicle_control.yaml',
                'obstaclestopper': 'false'
            }.items()
        )
    ])