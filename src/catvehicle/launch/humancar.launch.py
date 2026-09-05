"""

Author: Jonathan Sprinkle, Rahul Bhadani
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
IF THE ARIZONA BOARD OF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.

THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, 
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
IS ON AN "AS IS" BASIS, AND THE ARIZONA BOARD OF REGENTS HAS NO OBLIGATION
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Summary: 
=======
This launch file loads the worlds and models for the humancars

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_name = LaunchConfiguration('robot_name')
    init_pose = LaunchConfiguration('init_pose')
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([

        DeclareLaunchArgument('robot_name'),
        DeclareLaunchArgument('init_pose'),
        DeclareLaunchArgument('config_file'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=['urdf_spawner', robot_name],
            output='screen',
            arguments=['-entity', robot_name, '-topic', 'robot_description']
        ),

        Node(
            package='controller_manager',
            executable='spawner.py',
            name=['controller_spawner', robot_name],
            namespace=robot_name,
            output='screen',
            arguments=[
                'joint1_velocity_controller',
                'joint2_velocity_controller',
                'front_left_steering_position_controller',
                'front_right_steering_position_controller',
                'joint_state_controller'
            ]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=['robot_state_publisher', robot_name],
            output='screen',
            remappings=[
                ('/joint_states', ['/', robot_name, '/joint_states'])
            ]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name=['joint_state_publisher', robot_name],
            output='screen',
            remappings=[
                ('/joint_states', ['/', robot_name, '/joint_states'])
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=['base_link2slamodom_tf_', robot_name],
            arguments=[
                '0', '0', '0', '0', '0', '1',
                [robot_name, '/base_link'],
                [robot_name, '/slamodom'],
                '5'
            ],
            output='screen'
        ),

        Node(
            package='catvehicle',
            executable='cmdvel2gazebo.py',
            name=['cmdvel2gazebo', robot_name],
            output='screen'
        ),

        Node(
            package='catvehicle',
            executable='odom2path.py',
            name=['odom2path', robot_name],
            respawn=True,
            output='screen',
            arguments=['-n', ['/', robot_name]]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=['global_frame_tf_', robot_name],
            arguments=[
                '0', '0', '0', '0', '0', '1',
                '/world',
                [robot_name, '/odom'],
                '100'
            ],
            output='screen'
        )
    ])