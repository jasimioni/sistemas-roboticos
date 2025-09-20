import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

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


class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('PoseSubscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.cmd_vel_publisher = CmdVelPublisher()
        
        self.steps = [
            (self.cmd_vel_publisher.move_forward, 2.0),  # Move forward for 2 seconds
            (self.cmd_vel_publisher.rotate, 1.56),  # Rotate 90 degrees (π/2 radians)
            (self.cmd_vel_publisher.move_forward, 2.0),  # Move forward for 2 seconds
            (self.cmd_vel_publisher.rotate, 1.56),  # Rotate 90 degrees (π/2 radians)
            (self.cmd_vel_publisher.move_forward, 2.0),  # Move forward for 2 seconds
            (self.cmd_vel_publisher.rotate, 1.56),  # Rotate 90 degrees (π/2 radians)
            (self.cmd_vel_publisher.move_forward, 2.0),  # Move forward for 2 seconds
            (self.cmd_vel_publisher.rotate, 1.56)   # Rotate 90
            
        ]
        
        self.i = 0

    def listener_callback(self, msg):
        if msg.linear_velocity == 0.0 and msg.angular_velocity == 0.0:
            self.get_logger().info(
                f'Turtle Pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}, linear_velocity={msg.linear_velocity:.2f}, angular_velocity={msg.angular_velocity:.2f}'
            )
            action, value = self.steps[self.i]
            action(value)
            self.i = (self.i + 1) % len(self.steps)
            
    def destroy(self):
        self.cmd_vel_publisher.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    subscriber = PoseSubscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()