from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    robot = LaunchConfiguration('robot')
    leader = LaunchConfiguration('leader')
    rosbag = LaunchConfiguration('rosbag')
    policy_model = LaunchConfiguration('policy_model')
    vf_model = LaunchConfiguration('vf_model')
    distance_topic = LaunchConfiguration('distance_topic')
    leadvel_topic = LaunchConfiguration('leadvel_topic')

    return LaunchDescription([

        DeclareLaunchArgument('robot', default_value='catvehicle'),
        DeclareLaunchArgument('leader', default_value='toyota'),
        DeclareLaunchArgument('rosbag', default_value='false'),
        DeclareLaunchArgument(
            'policy_model',
            default_value='/home/ivory/CyverseData/JmscslgroupData/trained_model/policy'
        ),
        DeclareLaunchArgument(
            'vf_model',
            default_value='/home/ivory/CyverseData/JmscslgroupData/trained_model/vf'
        ),
        DeclareLaunchArgument(
            'distance_topic',
            default_value=['/', robot, '/distanceEstimatorSteeringBased']
        ),
        DeclareLaunchArgument(
            'leadvel_topic',
            default_value=['/', leader, '/vel']
        ),

        Node(
            package='catvehicle',
            executable='rlpredict.py',
            name=['rlpredict_', robot],
            namespace=robot,
            output='screen',
            arguments=[
                policy_model,
                vf_model,
                distance_topic,
                leadvel_topic
            ]
        ),

        Node(
            package='rosbag',
            executable='record',
            name=['recorder_', robot],
            namespace=robot,
            output='screen',
            arguments=[
                '-o',
                '/home/ivory/CyverseData/JmscslgroupData/Bagfiles/transfer_test/RL_Prediction',
                '-a'
            ],
            condition=IfCondition(rosbag)
        )
    ])