from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dvl_host = LaunchConfiguration('dvl_host')
    dvl_port = LaunchConfiguration('dvl_port')

    return LaunchDescription([
        DeclareLaunchArgument('dvl_host', default_value='192.168.4.148'),
        DeclareLaunchArgument('dvl_port', default_value='16171'),

        Node(
            package='barracuda_dvl',
            executable='barracuda-dvl-ros-driver.py',
            name='dvl_publisher',
            namespace='barracuda',
            output='screen',
            parameters=[{
                'dvl_host': dvl_host,
                'dvl_port': dvl_port,
            }],
        ),
    ])
