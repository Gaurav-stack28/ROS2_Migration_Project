from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot = LaunchConfiguration('robot')
    csvfile = LaunchConfiguration('csvfile')
    time_col = LaunchConfiguration('time_col')
    vel_col = LaunchConfiguration('vel_col')
    str_angle = LaunchConfiguration('str_angle')
    input_type = LaunchConfiguration('input_type')

    return LaunchDescription([

        DeclareLaunchArgument(
            'robot',
            default_value='toyota'
        ),

        DeclareLaunchArgument(
            'csvfile',
            default_value='/home/ivory/CyverseData/JmscslgroupData/ARED/2016-07-28/data_by_test/CSVData/test5/test5_01.csv'
        ),

        DeclareLaunchArgument(
            'time_col',
            default_value='Time'
        ),

        DeclareLaunchArgument(
            'vel_col',
            default_value='Speed'
        ),

        DeclareLaunchArgument(
            'str_angle',
            default_value='2.82321111'
        ),

        DeclareLaunchArgument(
            'input_type',
            default_value='CSV'
        ),

        DeclareLaunchArgument(
            'decoupled',
            default_value='false'
        ),

        Node(
            package='catvehicle',
            executable='velinjector.py',
            name=['velinjector_', robot],
            namespace=robot,
            output='screen',
            arguments=[
                csvfile,
                time_col,
                vel_col,
                str_angle,
                input_type
            ],
            remappings=[
                ('cmd_vel', 'cmd_control_vel')
            ]
        ),

        # Original ROS 1 commented section:
        #
        # <node name="model_state_$(arg robot)"
        #       pkg="sparkle"
        #       type="model_state"
        #       output="screen"
        #       if="$(arg decoupled)">
        #     <param name="enableTwist" value="false"/>
        # </node>

        Node(
            package='hoffmansubsystem',
            executable='hoffmansubsystem_node',
            name=['hoffmansubsystem_', robot],
            namespace=robot,
            output='screen',
            required=True
        )
    ])