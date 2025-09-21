# Observar a comunicação entre os nós do turtlesim
# usando as ferramentas de linha de comando;
# Escrever um nó em Python que assina o tópico de
# pose (/turtle1/pose) e imprime a posição e orientação
# da tartaruga;

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