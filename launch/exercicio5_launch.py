# Faça com que o TurtleBot seja capaz de navegar no
# ambiente, com uma velocidade fixa definida, desviando
# de obstáculos de acordo com o que é visto pelo sensor
# LiDAR;

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

os.environ["TURTLEBOT3_MODEL"] = "burger"

def generate_launch_description():
    
    my_package_share = get_package_share_directory('turtlebot3_gazebo')

    # Define the path to the child launch file
    child_launch_file = os.path.join(
        my_package_share,
        'launch',
        'turtlebot3_world.launch.py'
    )    
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(child_launch_file)
        ),
        Node(
            package='jasimioni',
            namespace='',
            executable='turtlebot_mover',
            name='turtlebot_mover'
        ),
    ])