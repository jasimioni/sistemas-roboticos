from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='jasimioni',
            namespace='',
            executable='pose_subscriber_printer',
            name='pose_subscriber_printer'
        ),
    ])