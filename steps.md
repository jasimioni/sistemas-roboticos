```
ros2 pkg create --build-type ament_python --license Apache-2.0 jasimioni --maintainer-name "Joao Andre Simioni" --maintainer-email jasimioni@gmail.com --description "Exercícios Sistemas Robóticos Inteligentes" --dependencies rclpy geometry_msgs turtlesim

colcon build --packages-select jasimioni

source install/local_setup.sh

ros2 launch jasimioni exercicio1_launch.py

ros2 launch jasimioni exercicio2_launch.py

ros2 launch jasimioni exercicio3_launch.py
ros2 topic pub /turtle1/go_to_position geometry_msgs/Point "{x: 1.0, y: 1.0, z: 0.0}" --once
```