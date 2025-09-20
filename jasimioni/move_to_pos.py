import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import math


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('CmdVelPublisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.publisher_  # prevent unused variable warning

    def move_forward(self, speed=3.0):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Moving foward at speed: {speed}')
        
    def rotate(self, angular_speed=1.0):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'Rotating at angular speed: {angular_speed}')


class PosePositionSubscriber(Node):
    def __init__(self):
        super().__init__('PosePositionSubscriber')
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_listener_callback,
            10)
        self.position_subscription = self.create_subscription(
            Point,
            '/turtle1/go_to_position',
            self.position_listener_callback,
            10)
        self.pose_subscription  # prevent unused variable warning
        self.position_subscription  # prevent unused variable warning
        self.cmd_vel_publisher = CmdVelPublisher()
        
        self.desired_x = None
        self.desired_y = None

        self.get_logger().info('PositionSubscriber has been started. To publish to it use:')
        self.get_logger().info('ros2 topic pub /turtle1/go_to_position geometry_msgs/Point "{x: 5.0, y: 5.0}"')

    def position_listener_callback(self, msg):
        self.get_logger().info(
            f'Desired Position: x={msg.x:.2f}, y={msg.y:.2f}' 
        )
        self.desired_x = msg.x
        self.desired_y = msg.y

    def pose_listener_callback(self, msg):
        if msg.linear_velocity == 0.0 and msg.angular_velocity == 0.0:
            
            if self.desired_x is not None and self.desired_y is not None:
                self.get_logger().info(
                    f'Turtle Pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}, linear_velocity={msg.linear_velocity:.2f}, angular_velocity={msg.angular_velocity:.2f}\n'
                )
                print(f'Desired Position: x={self.desired_x:.2f}, y={self.desired_y:.2f}')
                error_x = self.desired_x - msg.x
                error_y = self.desired_y - msg.y
                distance = (error_x**2 + error_y**2)**0.5
                
                if distance < 0.1:
                    self.get_logger().info('Reached the desired position.')
                    self.desired_x = None
                    self.desired_y = None
                    return
                
                angle_to_goal = math.atan2(error_y, error_x)
                angle_diff = angle_to_goal - msg.theta
                
                if abs(angle_diff) > math.pi:
                    angle_diff -= (2 * math.pi) * (angle_diff / abs(angle_diff))
                
                self.get_logger().info(f'Angle to goal: {angle_to_goal:.2f}, Current angle: {msg.theta:.2f}, Angle difference: {angle_diff:.2f}, Distance: {distance:.2f}')
                
                if abs(angle_diff) > 0.01:
                    self.cmd_vel_publisher.rotate(angle_diff)
                else:
                    self.cmd_vel_publisher.move_forward(min(3.0, distance))

    def destroy(self):
        self.cmd_vel_publisher.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    pose_position_subscriber = PosePositionSubscriber()
    
    rclpy.spin(pose_position_subscriber)

    pose_position_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()