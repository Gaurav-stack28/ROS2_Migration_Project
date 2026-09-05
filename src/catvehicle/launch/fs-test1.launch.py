"""

Author: Rahul Kumar Bhadani
Copyright (c) 2020 Arizona Board of Regents
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

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    leader = LaunchConfiguration('leader')
    dx_min = LaunchConfiguration('dx_min')
    dx_activate = LaunchConfiguration('dx_activate')
    velodyne_points = LaunchConfiguration('velodyne_points')
    camera_left = LaunchConfiguration('camera_left')
    camera_right = LaunchConfiguration('camera_right')
    triclops = LaunchConfiguration('triclops')
    ego_laser_sensor = LaunchConfiguration('ego_laser_sensor')
    leader_laser_sensor = LaunchConfiguration('leader_laser_sensor')
    obstaclestopper = LaunchConfiguration('obstaclestopper')
    rosbag = LaunchConfiguration('rosbag')
    csvfile = LaunchConfiguration('csvfile')
    dbcfile = LaunchConfiguration('dbcfile')
    catvehicle_share = FindPackageShare('catvehicle')

    return LaunchDescription([

        DeclareLaunchArgument('robot', default_value='catvehicle'),
        DeclareLaunchArgument('leader', default_value='toyota'),
        DeclareLaunchArgument('dx_min', default_value='4.5'),
        DeclareLaunchArgument('dx_activate', default_value='6.0'),
        DeclareLaunchArgument('velodyne_points', default_value='false'),
        DeclareLaunchArgument('camera_left', default_value='false'),
        DeclareLaunchArgument('camera_right', default_value='false'),
        DeclareLaunchArgument('triclops', default_value='false'),
        DeclareLaunchArgument('ego_laser_sensor', default_value='true'),
        DeclareLaunchArgument('leader_laser_sensor', default_value='true'),
        DeclareLaunchArgument('obstaclestopper', default_value='false'),
        DeclareLaunchArgument('rosbag', default_value='false'),
        DeclareLaunchArgument('csvfile', default_value='/home/ivory/CyverseData/JmscslgroupData/PandaData/2020_07_08/2020-07-08-15-15-54_2T3MWRFVXLW056972_CAN_Messages.csv'),
        DeclareLaunchArgument('dbcfile', default_value='/home/ivory/VersionControl/Jmscslgroup/strym/examples/newToyotacode.dbc'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    catvehicle_share,
                    'launch',
                    'catvehicle_empty.launch.py'
                ])
            )
        ),

        Node(
            package='catvehicle',
            executable='drive_lead.py',
            name=['drive_lead_', leader],
            namespace=leader,
            output='screen',
            arguments=[csvfile, dbcfile],
            remappings=[
                ('cmd_vel', 'cmd_control_vel')
            ]
        ),

        Node(
            package='hoffmansubsystem',
            executable='hoffmansubsystem_node',
            name=['hoffmansubsystem_', leader],
            namespace=leader,
            output='screen'
        ),

        Node(
            package='ros2bag',
            executable='record',
            name='recorder',
            output='screen',
            arguments=[
                '-o',
                [
                    '/home/ivory/VersionControl/Jmscslgroup/safetyfs/bagfiles/fs-test1_dxmin_',
                    dx_min,
                    '_dx_activate_',
                    dx_activate
                ],
                '-a'
            ],
            condition=IfCondition(rosbag)
        ),

        Node(
            package='fs',
            executable='fs_node',
            name=['fs_', robot],
            namespace=robot,
            output='screen',
            remappings=[
                ('d_relative', [
                    '/',
                    robot,
                    '/distanceEstimator/dist'
                ]),
                ('cmd_vel', 'cmd_control_vel')
            ]
        ),

        Node(
            package='hoffmansubsystem',
            executable='hoffmansubsystem_node',
            name=['hoffmansubsystem_', robot],
            namespace=robot,
            output='screen'
        )
    ])