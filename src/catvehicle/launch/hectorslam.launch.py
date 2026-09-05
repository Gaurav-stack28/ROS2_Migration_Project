"""

Author: Jonathan Sprinkle
Copyright (c) 2015-2016 Arizona Board of Regents
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
This launch file integrates hector slam with the car's various topics
and frames, including the laser topic from the front of the car.

How to execute it:
=================
After starting up a simulation that includes the CAT Vehicle, then:

roslaunch catvehicle hectorslam.launch

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_name',
            default_value='catvehicle'
        ),

        Node(
            package='hector_mapping',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            parameters=[
                {
                    'pub_map_odom_transform': True,
                    'map_frame': 'map',
                    'scan_topic': ['/', robot_name, '/front_laser_points'],
                    'base_frame': [robot_name, '/base_link'],
                    'odom_frame': [robot_name, '/odom'],
                    'map_resolution': 1.0,
                    'map_size': 200,
                    'map_pub_period': 0.5,
                    'scan_subscriber_queue_size': 100
                }
            ]
        )
    ])