import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose


class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('PoseSubscriber')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Turtle Pose: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}, linear_velocity={msg.linear_velocity:.2f}, angular_velocity={msg.angular_velocity:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)

    subscriber = PoseSubscriber()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()