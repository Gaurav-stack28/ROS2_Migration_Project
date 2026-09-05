"""
Author: Jonathan Sprinkle, Rahul Bhadani
Copyright (c) 2016-2020 Arizona Board of Regents
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
This launch file is used to broadcast tf and robot parameters for playback
or live visualization of data through rviz

How to execute it:
=================
roslaunch catvehicle catvehicle-tf.launch

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_name = LaunchConfiguration('robot_name')
    tyre_height = LaunchConfiguration('tyre_height')
    halftyre_height = LaunchConfiguration('halftyre_height')
    car_width = LaunchConfiguration('car_width')
    car_length = LaunchConfiguration('car_length')
    car_height = LaunchConfiguration('car_height')
    tyre_front_x = LaunchConfiguration('tyre_front_x')
    tyre_back_x = LaunchConfiguration('tyre_back_x')

    return LaunchDescription([

        DeclareLaunchArgument('robot_name', default_value='catvehicle'),
        DeclareLaunchArgument('tyre_height', default_value='0.7'),
        DeclareLaunchArgument('halftyre_height', default_value='0.32'),
        DeclareLaunchArgument('car_width', default_value='0.77'),
        DeclareLaunchArgument('car_length', default_value='1.55'),
        DeclareLaunchArgument('car_height', default_value='1.5837572084'),
        DeclareLaunchArgument('tyre_front_x', default_value='1.52'),
        DeclareLaunchArgument('tyre_back_x', default_value='-1.1'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['odom2ins_', robot_name], arguments=['-0.8', ['-', car_width], '0.0', '0', '0', '0', 'catvehicle/odom', 'catvehicle/base_link', '10'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['baselink2velodyne_', robot_name], arguments=['-0.6', '0', '2.12', '0', '0', '0', 'catvehicle/base_link', 'velodyne', '10'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['velodyne2velodyne_link_', robot_name], arguments=['0', '0', '0', '0', '0', '0', 'velodyne', 'catvehicle/velodyne_link', '10'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['baselinkr2front_laser_link_tf_', robot_name], arguments=['2.5', '0.0', '1.1', '0.0', '0', '0', 'catvehicle/base_link', 'catvehicle/front_laser_link', '75'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['baselink_laser_tf_', robot_name], arguments=['2.5', '0.0', '1.1', '0.0', '0', '0', 'catvehicle/base_link', 'laser', '75'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['baselink2mainmass_tf_', robot_name], arguments=['0', '0', '0', '0', '0', '0', 'catvehicle/base_link', 'catvehicle/main_mass', '20'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['baselink2leftFtire_tf_', robot_name], arguments=[tyre_front_x, ['-', car_width], tyre_height, '3.14159265359', '0', '0', 'catvehicle/base_link', 'catvehicle/front_left_wheel_link', '10'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['baselink2rightFtire_tf_', robot_name], arguments=[tyre_front_x, car_width, tyre_height, '3.14159265359', '0', '0', 'catvehicle/base_link', 'catvehicle/front_right_wheel_link', '10'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['baselink2leftRtire_tf_', robot_name], arguments=[tyre_back_x, ['-', car_width], tyre_height, '3.14159265359', '0', '0', 'catvehicle/base_link', 'catvehicle/back_left_wheel_link', '10'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['baselink2rightRtire_tf_', robot_name], arguments=[tyre_back_x, car_width, tyre_height, '3.14159265359', '0', '0', 'catvehicle/base_link', 'catvehicle/back_right_wheel_link', '10'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['rightFtire2rightFtiresteering_tf_', robot_name], arguments=['0', '0', '0', '0', '0', '0', 'catvehicle/front_right_wheel_link', 'catvehicle/front_right_steering_link', '10'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['leftFtire2leftFtiresteering_tf_', robot_name], arguments=['0', '0', '0', '0', '0', '0', 'catvehicle/front_left_wheel_link', 'catvehicle/front_left_steering_link', '10'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['velodynelink2cameraleft_tf_', robot_name], arguments=['-0.56', '0.44', '0', '0', '0', '0', 'catvehicle/velodyne_link', 'catvehicle/camera_left_link', '5'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['velodynelink2cameraright_tf_', robot_name], arguments=['-0.56', '-0.44', '0', '0', '0', '0', 'catvehicle/velodyne_link', 'catvehicle/camera_right_link', '5'], output='screen'),

        Node(package='tf2_ros', executable='static_transform_publisher', name=['velodynelink2camera_tf_', robot_name], arguments=['0.54', '0', '-0.08', '0', '0', '0', 'catvehicle/velodyne_link', 'catvehicle/triclops_link', '5'], output='screen'),
    ])