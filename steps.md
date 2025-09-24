```
ros2 pkg create --build-type ament_python --license Apache-2.0 jasimioni --maintainer-name "Joao Andre Simioni" --maintainer-email jasimioni@gmail.com --description "Exercícios Sistemas Robóticos Inteligentes" --dependencies rclpy geometry_msgs turtlesim math
```

```
git clone

colcon build --packages-select jasimioni

source install/local_setup.sh

ros2 launch jasimioni exercicio1_launch.py

ros2 launch jasimioni exercicio2_launch.py

ros2 launch jasimioni exercicio3_launch.py
ros2 topic pub /turtle1/go_to_position geometry_msgs/Point "{x: 1.0, y: 1.0, z:0.0}" --once


TURTLEBOT3_MODEL=burger ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 topic pub /go_to_position geometry_msgs/Point "{x: 1.0, y: 1.0, z: 0.0}" --once

```

### Ex.1

```
ros2 launch jasimioni exercicio1_launch.py
```

### Ex.2

```
ros2 launch jasimioni exercicio2_launch.py
```

### Ex.3

```
ros2 launch jasimioni exercicio3_launch.py

# Set a desired position:
ros2 topic pub /turtle1/go_to_position geometry_msgs/Point "{x: 1.0, y: 1.0, z:0.0}" --once
```

### Ex. 4

```
ros2 launch jasimioni exercicio4_launch.py
# Move the ROBOT using the TELEOP on Gazebo
```


### Ex. 5

```
ros2 launch jasimioni exercicio5_launch.py
```

### Ex. 6

```
ros2 launch jasimioni exercicio6_launch.py
ros2 run jasimioni turtlebot_mover.py
# OR
python3 src/jasimioni/jasimioni/turtlebot_mover.py
```

Notes: Using the python approach works better. With the Node in a launch file
the sync between odom / lidar is broken for some reason. Using the console script
works well, but the python call is better all the time.

### Ex.7

```
ros2 launch jasimioni exercicio7_launch.py

# Set a desired position:
ros2 topic pub /go_to_position geometry_msgs/Point "{x: 3.0, y: 1.0, z:0.0}" --once
```

### Ex.8

```
ros2 launch jasimioni exercicio8_launch.py

# Set a desired position:
ros2 topic pub /go_to_position geometry_msgs/Point "{x: 3.0, y: 1.0, z:0.0}" --once
```


### Ex.9

```
ros2 launch jasimioni exercicio8_launch.py

# Set a desired position:
ros2 topic pub /go_to_position geometry_msgs/Point "{x: 3.0, y: 1.0, z:0.0}" --once
```