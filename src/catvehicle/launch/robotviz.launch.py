"""

Author: Jonathan Sprinkle
Copyright (c) 2016 Arizona Board of Regents
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
This launch file loads the worlds and models for the azcar during playback
or whenever gazebo is not running

How to execute it:
=================
roslaunch catvehicle robotviz.launch

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import FindPackageShare


def generate_launch_description():

    robot_name = LaunchConfiguration('robot_name')
    front_laser_points = LaunchConfiguration('front_laser_points')
    velodyne_points = LaunchConfiguration('velodyne_points')
    camera_right = LaunchConfiguration('camera_right')
    camera_left = LaunchConfiguration('camera_left')
    velodyne_max_angle = LaunchConfiguration('velodyne_max_angle')
    velodyne_min_angle = LaunchConfiguration('velodyne_min_angle')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('catvehicle'),
        'urdf',
        'catvehicle.xacro'
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' roboname:=',
            robot_name,
            ' front_laser_points:=',
            front_laser_points,
            ' velodyne_points:=',
            velodyne_points,
            ' camera_right:=',
            camera_right,
            ' camera_left:=',
            camera_left,
            ' velodyne_max_angle:=',
            velodyne_max_angle,
            ' velodyne_min_angle:=',
            velodyne_min_angle
        ]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='catvehicle'
        ),

        DeclareLaunchArgument(
            'front_laser_points',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'velodyne_points',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'camera_right',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'camera_left',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'velodyne_max_angle',
            default_value='0.4'
        ),

        DeclareLaunchArgument(
            'velodyne_min_angle',
            default_value='-0.4'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        )
    ])