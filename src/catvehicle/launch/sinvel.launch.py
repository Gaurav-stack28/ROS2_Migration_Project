"""

Author: Rahul Kumar Bhadani
Copyright (c) 2018 Arizona Board of Regents
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
========

This launch file applies a velocity profile to the leader car already loaded into the simulator.

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    bias = LaunchConfiguration('Bias')
    amplitude = LaunchConfiguration('Amp')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='catvehicle'
        ),

        DeclareLaunchArgument(
            'Bias',
            default_value='12.0'
        ),

        DeclareLaunchArgument(
            'Amp',
            default_value='10.0'
        ),

        Node(
            package='sinvel',
            executable='sinvel_node',
            name=['sinvel_', robot],
            namespace=robot,
            output='screen',
            parameters=[{
                'bias': ParameterValue(bias, value_type=float),
                'amplitude': ParameterValue(amplitude, value_type=float)
            }]
        )
    ])