# Desenvolvendo um controle de posição!
# Receba uma posição (x,y);
# Faça com que a tartaruga chegue até a posição;

# ros2 topic pub /turtle1/go_to_position geometry_msgs/Point "{x: 1.0, y: 1.0, z: 0.0}" --once

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
            executable='move_to_pos',
            name='move_to_pos'
        ),
    ])