# No Turtlebot3, utiliza o Fuzzy para navegar com o robô até
# uma posição desejada;

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
