# Iniciar uma simulação do Gazebo com o Turtlebot3 em um ambiente
# e iniciar o módulo Nav2 para navegação autônoma 
# export TURTLEBOT3_MODEL=waffle; ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
# Usar o /goal_pose para definir a posição desejada do robô e medir pelo Odom.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

import os

os.environ["TURTLEBOT3_MODEL"] = "burger"

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='jasimioni',
            namespace='',
            executable='waffer_2_pos',
            name='waffer_2_pos',
        ),
        
    ])
