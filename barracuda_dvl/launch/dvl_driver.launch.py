from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dvl_host = LaunchConfiguration('dvl_host')
    dvl_port = LaunchConfiguration('dvl_port')
    frame_id = LaunchConfiguration('frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    publish_tf = LaunchConfiguration('publish_tf')
    connection_timeout = LaunchConfiguration('connection_timeout')
    reconnect_interval = LaunchConfiguration('reconnect_interval')
    publish_static_tf = LaunchConfiguration('publish_static_tf')

    return LaunchDescription([
        DeclareLaunchArgument('dvl_host', default_value='192.168.2.95'),
        DeclareLaunchArgument('dvl_port', default_value='16171'),
        DeclareLaunchArgument('frame_id', default_value='dvl_link'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('connection_timeout', default_value='5.0'),
        DeclareLaunchArgument('reconnect_interval', default_value='2.0'),
        DeclareLaunchArgument('publish_static_tf', default_value='true'),

        Node(
            package='barracuda_dvl',
            executable='barracuda-dvl-ros-driver.py',
            name='barracuda_dvl_driver',
            output='screen',
            parameters=[{
                'dvl_host': dvl_host,
                'dvl_port': dvl_port,
                'frame_id': frame_id,
                'odom_frame_id': odom_frame_id,
                'publish_tf': publish_tf,
                'connection_timeout': connection_timeout,
                'reconnect_interval': reconnect_interval,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_dvl',
            output='screen',
            arguments=['0', '0', '-0.1', '0', '0', '0', 'base_link', 'dvl_link'],
            condition=IfCondition(publish_static_tf),
        ),
    ])
