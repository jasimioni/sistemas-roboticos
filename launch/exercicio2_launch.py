# Escrever um nó em Python que publica no tópico de
# velocidade (/turtle1/cmd_vel) para fazer a tartaruga
# desenhar um quadrado automaticamente;

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
            executable='draw_square',
            name='draw_square'
        ),
    ])