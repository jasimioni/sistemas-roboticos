import rclpy
from rclpy.node import Node
import math
import json
import time
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


class Lidar2DScanner(Node):
    def __init__(self):
        super().__init__('Lidar2DScanner')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

        self.publisher_  # prevent unused variable warning
        self.subscription  # prevent unused variable warning
        
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        self.change_velocity(self.linear_velocity, self.angular_velocity)

    def change_velocity(self, linear, angular):
        self.linear_velocity = linear
        self.angular_velocity = angular

        msg = TwistStamped()

        # Preenche o cabeçalho (Header) da mensagem com o tempo atual e o frame de referência
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Preenche a parte 'twist' da mensagem com as velocidades desejadas
        msg.twist.linear.x = self.linear_velocity
        msg.twist.angular.z = self.angular_velocity

        # Publica a mensagem no tópico /cmd_vel
        self.publisher_.publish(msg)
        
    def listener_callback(self, msg):
        front_ranges = [ 3.5 if math.isinf(x) else x for x in (msg.ranges[0:15] + msg.ranges[345:]) if not math.isnan(x) ]
        left_ranges = [ 3.5 if math.isinf(x) else x for x in msg.ranges[15:75] if not math.isnan(x) ]
        right_ranges = [ 3.5 if math.isinf(x) else x for x in msg.ranges[285:345] if not math.isnan(x) ]

        front_str = ' '.join(f'{x:.2f}' for x in front_ranges)
        left_str = ' '.join(f'{x:.2f}' for x in left_ranges)
        right_str = ' '.join(f'{x:.2f}' for x in right_ranges)

        print(f'Front: {front_str}')
        print(f'Left: {left_str}')
        print(f'Right: {right_str}')

        front = sum(front_ranges) / len(front_ranges) if front_ranges else 0.0
        left = sum(left_ranges) / len(left_ranges) if left_ranges else 0.0
        right = sum(right_ranges) / len(right_ranges) if right_ranges else 0.0
        
        left_min = min(left_ranges) if left_ranges else 0.0
        right_min = min(right_ranges) if right_ranges else 0.0

        print(f'Front: {front:.2f}, Left: {left:.2f}, Right: {right:.2f}')
        
        if front > right and front > left:
            direction = 'front'
        elif left > right:
            direction = 'left'
        else:
            direction = 'right'
            
        if right_min < 0.5:
            direction = 'left'
        
        if left_min < 0.5:
            direction = 'right'
        
        
        if direction == 'front':
            self.get_logger().info('Moving Forward')
            self.change_velocity(.5, 0.0)
        elif direction == 'left':   
            self.get_logger().info('Moving to the Left')
            self.change_velocity(.5, 1.0)
        else:
            self.get_logger().info('Moving to the Right')
            self.change_velocity(.5, -1.0)
        
        
def main(args=None):
    rclpy.init(args=args)

    subscriber = Lidar2DScanner()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()