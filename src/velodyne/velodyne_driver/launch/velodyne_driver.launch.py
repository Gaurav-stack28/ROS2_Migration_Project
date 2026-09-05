from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    device_ip = LaunchConfiguration('device_ip')
    frame_id = LaunchConfiguration('frame_id')
    model = LaunchConfiguration('model')
    pcap = LaunchConfiguration('pcap')
    port = LaunchConfiguration('port')
    read_fast = LaunchConfiguration('read_fast')
    read_once = LaunchConfiguration('read_once')
    repeat_delay = LaunchConfiguration('repeat_delay')
    rpm = LaunchConfiguration('rpm')
    gps_time = LaunchConfiguration('gps_time')
    pcap_time = LaunchConfiguration('pcap_time')
    cut_angle = LaunchConfiguration('cut_angle')
    timestamp_first_packet = LaunchConfiguration(
        'timestamp_first_packet')
    diagnostic_frequency_tolerance = LaunchConfiguration(
        'diagnostic_frequency_tolerance')


    return LaunchDescription([

        DeclareLaunchArgument(
            'device_ip',
            default_value=''),

        DeclareLaunchArgument(
            'frame_id',
            default_value='velodyne'),

        DeclareLaunchArgument(
            'model',
            default_value='64E'),

        DeclareLaunchArgument(
            'pcap',
            default_value=''),

        DeclareLaunchArgument(
            'port',
            default_value='2368'),

        DeclareLaunchArgument(
            'read_fast',
            default_value='false'),

        DeclareLaunchArgument(
            'read_once',
            default_value='false'),

        DeclareLaunchArgument(
            'repeat_delay',
            default_value='0.0'),

        DeclareLaunchArgument(
            'rpm',
            default_value='600.0'),

        DeclareLaunchArgument(
            'gps_time',
            default_value='false'),

        DeclareLaunchArgument(
            'pcap_time',
            default_value='false'),

        DeclareLaunchArgument(
            'cut_angle',
            default_value='-0.01'),

        DeclareLaunchArgument(
            'timestamp_first_packet',
            default_value='false'),

        DeclareLaunchArgument(
            'diagnostic_frequency_tolerance',
            default_value='0.1'),


        ComposableNodeContainer(
            name='velodyne_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[

                ComposableNode(
                    package='velodyne_driver',
                    plugin='velodyne_driver::VelodyneDriver',
                    name='velodyne_driver',
                    parameters=[
                        {
                            'device_ip': device_ip,
                            'frame_id': frame_id,
                            'model': model,
                            'pcap': pcap,
                            'port': port,
                            'read_fast': read_fast,
                            'read_once': read_once,
                            'repeat_delay': repeat_delay,
                            'rpm': rpm,
                            'gps_time': gps_time,
                            'pcap_time': pcap_time,
                            'cut_angle': cut_angle,
                            'timestamp_first_packet':
                                timestamp_first_packet,
                            'diagnostic_frequency_tolerance':
                                diagnostic_frequency_tolerance,
                        }
                    ]
                )

            ],
            output='screen'
        )
    ])