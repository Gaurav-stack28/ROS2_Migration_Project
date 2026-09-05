from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    robot = LaunchConfiguration('robot')
    rosbag = LaunchConfiguration('rosbag')
    csvfile = LaunchConfiguration('csvfile')
    dbcfile = LaunchConfiguration('dbcfile')

    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='toyota'),
        DeclareLaunchArgument('rosbag', default_value='false'),
        DeclareLaunchArgument('csvfile', default_value='/home/ivory/CyverseData/JmscslgroupData/PandaData/2020_02_18/2020-02-18-13-00-42-209119__CAN_Messages.csv'),
        DeclareLaunchArgument('dbcfile', default_value='/home/ivory/VersionControl/Jmscslgroup/strym/examples/newToyotacode.dbc'),

        Node(
            package='catvehicle',
            executable='drive_lead.py',
            name=['drive_lead_', robot],
            namespace=robot,
            output='screen',
            arguments=[csvfile, dbcfile],
        ),

        Node(
            package='rosbag',
            executable='record',
            name='recorder',
            output='screen',
            arguments=['-o', '/home/ivory/CyverseData/JmscslgroupData/Bagfiles/RNN_Prediction/CAN2ROS', '-a'],
            condition=IfCondition(rosbag),
        ),
    ])