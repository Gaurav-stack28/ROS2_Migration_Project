from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_name = LaunchConfiguration('robot_name')
    init_pose = LaunchConfiguration('init_pose')
    config_file = LaunchConfiguration('config_file')
    obstaclestopper = LaunchConfiguration('obstaclestopper')

    catvehicle_share = FindPackageShare('catvehicle')

    config_path = PathJoinSubstitution([
        catvehicle_share,
        'config',
        config_file
    ])

    return LaunchDescription([

        # ------------------------------------------------------------
        # Launch arguments
        # ------------------------------------------------------------

        DeclareLaunchArgument(
            'robot_name',
            default_value='catvehicle'
        ),

        DeclareLaunchArgument(
            'init_pose',
            default_value='0 0 0'
        ),

        DeclareLaunchArgument(
            'config_file',
            default_value='catvehicle_control.yaml'
        ),

        DeclareLaunchArgument(
            'obstaclestopper',
            default_value='false'
        ),

        # ------------------------------------------------------------
        # Spawn CATvehicle in Gazebo
        # ------------------------------------------------------------

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=[
                'urdf_spawner',
                robot_name
            ],
            output='screen',
            arguments=[
                '-entity',
                robot_name,
                '-topic',
                'robot_description'
            ]
        ),

        # ------------------------------------------------------------
        # Joint 1 velocity controller
        # ------------------------------------------------------------

        Node(
            package='controller_manager',
            executable='spawner.py',
            name=[
                'joint1_velocity_controller_spawner',
                robot_name
            ],
            namespace=robot_name,
            output='screen',
            arguments=[
                'joint1_velocity_controller',
                '--controller-manager',
                '/controller_manager'
            ]
        ),

        # ------------------------------------------------------------
        # Joint 2 velocity controller
        # ------------------------------------------------------------

        Node(
            package='controller_manager',
            executable='spawner.py',
            name=[
                'joint2_velocity_controller_spawner',
                robot_name
            ],
            namespace=robot_name,
            output='screen',
            arguments=[
                'joint2_velocity_controller',
                '--controller-manager',
                '/controller_manager'
            ]
        ),

        # ------------------------------------------------------------
        # Front left steering controller
        # ------------------------------------------------------------

        Node(
            package='controller_manager',
            executable='spawner.py',
            name=[
                'front_left_steering_position_controller_spawner',
                robot_name
            ],
            namespace=robot_name,
            output='screen',
            arguments=[
                'front_left_steering_position_controller',
                '--controller-manager',
                '/controller_manager'
            ]
        ),

        # ------------------------------------------------------------
        # Front right steering controller
        # ------------------------------------------------------------

        Node(
            package='controller_manager',
            executable='spawner.py',
            name=[
                'front_right_steering_position_controller_spawner',
                robot_name
            ],
            namespace=robot_name,
            output='screen',
            arguments=[
                'front_right_steering_position_controller',
                '--controller-manager',
                '/controller_manager'
            ]
        ),

        # ------------------------------------------------------------
        # Joint state broadcaster
        # ------------------------------------------------------------

        Node(
            package='controller_manager',
            executable='spawner.py',
            name=[
                'joint_state_controller_spawner',
                robot_name
            ],
            namespace=robot_name,
            output='screen',
            arguments=[
                'joint_state_controller',
                '--controller-manager',
                '/controller_manager'
            ]
        ),

        # ------------------------------------------------------------
        # Joint state publisher
        # ------------------------------------------------------------

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name=[
                'joint_state_publisher',
                robot_name
            ],
            output='screen',
            remappings=[
                (
                    '/joint_states',
                    [
                        '/',
                        robot_name,
                        '/joint_states'
                    ]
                )
            ]
        ),

        # ------------------------------------------------------------
        # Static TF:
        # <robot>/base_link -> <robot>/slamodom
        # ------------------------------------------------------------

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[
                'base_link2slamodom_tf_',
                robot_name
            ],
            arguments=[
                '0',
                '0',
                '0',
                '0',
                '0',
                '1',
                [
                    '/',
                    robot_name,
                    '/base_link'
                ],
                [
                    '/',
                    robot_name,
                    '/slamodom'
                ]
            ],
            output='screen'
        ),

        # ------------------------------------------------------------
        # cmdvel2gazebo - normal mode
        #
        # obstaclestopper=false
        # /<robot>/cmd_vel
        # ------------------------------------------------------------

        Node(
            package='catvehicle',
            executable='cmdvel2gazebo.py',
            name=[
                'cmdvel2gazebo',
                robot_name
            ],
            output='screen',
            arguments=[
                '-n',
                robot_name
            ],
            condition=IfCondition(
                PythonExpression([
                    "'",
                    obstaclestopper,
                    "' == 'false'"
                ])
            )
        ),

        # ------------------------------------------------------------
        # cmdvel2gazebo - obstacle stopper mode
        #
        # obstaclestopper=true
        # /<robot>/cmd_vel_safe
        # ------------------------------------------------------------

        Node(
            package='catvehicle',
            executable='cmdvel2gazebo.py',
            name=[
                'cmdvel2gazebo_safe',
                robot_name
            ],
            output='screen',
            arguments=[
                '-n',
                robot_name
            ],
            remappings=[
                (
                    [
                        '/',
                        robot_name,
                        '/cmd_vel'
                    ],
                    [
                        '/',
                        robot_name,
                        '/cmd_vel_safe'
                    ]
                )
            ],
            condition=IfCondition(obstaclestopper)
        ),

        # ------------------------------------------------------------
        # Obstacle stopper
        # ------------------------------------------------------------

        Node(
            package='obstaclestopper',
            executable='obstaclestopper_node',
            name=[
                'obstacleStopper',
                robot_name
            ],
            output='screen',
            condition=IfCondition(obstaclestopper)
        ),

        # ------------------------------------------------------------
        # Odom to path
        # ------------------------------------------------------------

        Node(
            package='catvehicle',
            executable='odom2path.py',
            name=[
                'odom2path',
                robot_name
            ],
            output='screen',
            respawn=True,
            arguments=[
                '-n',
                [
                    '/',
                    robot_name
                ]
            ]
        ),

        # ------------------------------------------------------------
        # Static TF:
        # /world -> /<robot>/odom
        # ------------------------------------------------------------

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=[
                'global_frame_tf_',
                robot_name
            ],
            arguments=[
                '0',
                '0',
                '0',
                '0',
                '0',
                '0',
                '/world',
                [
                    '/',
                    robot_name,
                    '/odom'
                ]
            ],
            output='screen'
        ),
    ])
