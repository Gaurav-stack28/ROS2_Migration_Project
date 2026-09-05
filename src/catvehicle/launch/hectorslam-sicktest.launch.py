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

This launch file loads the SLAM algorithms using the hector_slam package

How to execute this file?
========================

roslaunch catvehicle catvehicle_skidpan.launch

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    trajectory_source_frame_name = LaunchConfiguration('trajectory_source_frame_name')
    trajectory_update_rate = LaunchConfiguration('trajectory_update_rate')
    trajectory_publish_rate = LaunchConfiguration('trajectory_publish_rate')

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_name',
            default_value='catvehicle'
        ),

        DeclareLaunchArgument(
            'trajectory_source_frame_name',
            default_value='/slamodom'
        ),

        DeclareLaunchArgument(
            'trajectory_update_rate',
            default_value='4'
        ),

        DeclareLaunchArgument(
            'trajectory_publish_rate',
            default_value='0.25'
        ),

        Node(
            package='hector_mapping',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            parameters=[
                {'pub_map_odom_transform': True},
                {'map_frame': 'map'},
                {'scan_topic': '/scan'},
                {'base_frame': '/laser'},
                {'odom_frame': '/slamodom'},
                {'map_resolution': 0.1},
                {'map_size': 500},
                {'map_pub_period': 0.5},
                {'scan_subscriber_queue_size': 100}
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=['map2odom_tf_', robot_name],
            arguments=[
                '0',
                '0',
                '0',
                '0',
                '0',
                '0',
                '/slamodom',
                '/laser'
            ],
            output='screen'
        ),

        Node(
            package='hector_trajectory_server',
            executable='hector_trajectory_server',
            name='hector_trajectory_server',
            output='screen',
            parameters=[
                {'target_frame_name': '/map'},
                {'source_frame_name': trajectory_source_frame_name},
                {'trajectory_update_rate': trajectory_update_rate},
                {'trajectory_publish_rate': trajectory_publish_rate}
            ]
        ),

        Node(
            package='hector_geotiff',
            executable='geotiff_node',
            name='hector_geotiff_node',
            output='screen',
            prefix='nice -n 15',
            parameters=[
                {'map_file_path': [
                    FindExecutable(name='echo'),
                    '/maps'
                ]},
                {'map_file_base_name': 'hector_slam_map'},
                {'geotiff_save_period': 0.0},
                {'draw_background_checkerboard': True},
                {'draw_free_space_grid': True}
            ],
            remappings=[
                ('map', '/dynamic_map')
            ]
        )
    ])