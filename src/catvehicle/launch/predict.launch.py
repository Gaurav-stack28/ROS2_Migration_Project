from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():

    rosbag = LaunchConfiguration('rosbag')
    robot = LaunchConfiguration('robot')

    return LaunchDescription([

        DeclareLaunchArgument(
            'rosbag',
            default_value='true'
        ),

        DeclareLaunchArgument(
            'robot',
            default_value='catvehicle'
        ),

        Node(
            package='catvehicle',
            executable='predict.py',
            name='predict',
            namespace=robot,
            output='screen',
            required=True
        ),

        Node(
            package='rosbag',
            executable='record',
            name='recorder',
            output='screen',
            arguments=[
                '-o',
                '/home/ivory/CyverseData/JmscslgroupData/Bagfiles/RNN_Prediction/model_prediction',
                '-a'
            ],
            condition=IfCondition(rosbag)
        )
    ])