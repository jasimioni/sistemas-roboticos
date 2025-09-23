import rclpy
from rclpy.node import Node
import math
import json
import time
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class PIDController:
    """A simple PID controller class."""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        """Calculates the PID output."""
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
    def clean(self):
        self.prev_error = 0.0
        self.integral = 0.0


class MoverPID(Node):
    def __init__(self):
        super().__init__('MoverPID')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.store_last_scan,
            10)
        
        self.pose_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.store_last_pose,
            10)
    
        self.position_subscription = self.create_subscription(
            Point,
            '/go_to_position',
            self.position_listener_callback,
            10)
        

        self.publisher_  # prevent unused variable warning
        self.subscription  # prevent unused variable warning
        self.pose_subscription  # prevent unused variable warning
        
        self.last_pose = None
        self.last_scan = None
        self.last_time = time.time()
        
        self.max_linear_velocity = 0.5
        self.max_angular_velocity = 2.5
        
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        self.desired_x = 0.0
        self.desired_y = 0.0
    
        self.distance_pid = PIDController(kp=1.5, ki=0.01, kd=0.05)
        self.orientation_pid = PIDController(kp=1.5, ki=0.1, kd=0.1)
        
        self.loop_interval = 0.1  # seconds
        self.circle_control = []
        
        self.timer = self.create_timer(self.loop_interval, self.pid_set_position_callback)

    def euler_from_quaternion(self, q):
        """Converts a quaternion to Euler angles (roll, pitch, yaw)."""
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z
        
    def change_velocity(self, linear=None, angular=None):
        if linear is not None:
            self.linear_velocity = min(linear, self.max_linear_velocity)
        if angular is not None:
            if abs(angular) > self.max_angular_velocity:
                angular = self.max_angular_velocity * (angular / abs(angular))
            
            self.angular_velocity = angular

        msg = TwistStamped()

        # Preenche o cabeçalho (Header) da mensagem com o tempo atual e o frame de referência
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Preenche a parte 'twist' da mensagem com as velocidades desejadas
        msg.twist.linear.x = float(self.linear_velocity)
        msg.twist.angular.z = float(self.angular_velocity)

        # Publica a mensagem no tópico /cmd_vel
        self.get_logger().info(f'Velocities set to -> Linear: {self.linear_velocity:.2f}, Angular: {self.angular_velocity:.2f}')
        self.publisher_.publish(msg)

    def store_last_pose(self, msg):
        self.last_pose = msg
        
    def store_last_scan(self, msg):
        self.last_scan = msg

    def position_listener_callback(self, msg):
        self.get_logger().info(
            f'Desired Position: x={msg.x:.2f}, y={msg.y:.2f}' 
        )
        self.desired_x = msg.x
        self.desired_y = msg.y
        
    def pid_set_position_callback(self):
        msg = self.last_pose
        if msg is None or self.desired_x is None or self.desired_y is None:
            self.get_logger().info('Waiting for pose and desired position...')
            return
        
        current_position_x = msg.pose.pose.position.x
        current_position_y = msg.pose.pose.position.y
        _, _, theta = self.euler_from_quaternion(msg.pose.pose.orientation)
        linear_x = msg.twist.twist.linear.x
        angular_z = msg.twist.twist.angular.z
        

        self.get_logger().info(
            f'Current Position -> x: {current_position_x:.2f}, y: {current_position_y:.2f}, '
            f'Linear.x: {linear_x:.2f}, Angular.y: {angular_z:.2f}, '
            f'Orientation.z: {theta:.2f}'
        )
        
        dx = self.desired_x - current_position_x
        dy = self.desired_y - current_position_y
        
        distance_error = math.sqrt( (dx)**2 + (dy)**2 )

        # Calcula o ângulo que a tartaruga deveria ter para apontar para o alvo
        desired_angle = math.atan2( dy, dx )

        # Calcula a diferença entre o ângulo desejado e o atual
        angle_error = math.atan2(math.sin(desired_angle - theta), 
                                 math.cos(desired_angle - theta))

        if distance_error < 0.1:
            self.get_logger().info('Reached the desired position.')
            self.change_velocity(linear=0, angular=0)
            self.distance_pid.clean()
            self.orientation_pid.clean()
            self.circle_control = []
            return
        
        dt = time.time() - self.last_time
        self.last_time = time.time()
        linear_velocity = self.distance_pid.update(distance_error, dt)
        angular_velocity = self.orientation_pid.update(angle_error, dt)
             
        if not self.circle_control or abs(distance_error - self.circle_control[-1]) < 0.1:
            self.get_logger().info(f"Adding circle controle instance: {distance_error}")
            self.circle_control.append(distance_error)
            
        if len(self.circle_control) > 30:
            # Spinning in circles - just stop to realign
            self.get_logger().info(f"Stopping to avoid running in circles")
            linear_velocity = 0
            self.circle_control = []
    
        
        # Linear velocity control (Proportional only)
        self.get_logger().info(f'Distance Error: {distance_error:.2f}, Linear Velocity (before clamp): {linear_velocity:.2f}')
        self.get_logger().info(f'Angle Error: {angle_error:.2f}, Angular Velocity (before clamp): {angular_velocity:.2f}')

        # Only move forward if reasonably aligned with the target
        #if abs(angle_error) > math.pi / 4:  # Less than 45 degrees error
        #    linear_velocity = 0.0  # Stop moving forward if not aligned
        
        # Obstacle avoidance using LIDAR (PID control for angular velocity)
        if self.last_scan is not None:
            front_ranges = [3.5 if math.isinf(x) else x for x in (self.last_scan.ranges[0:60] + self.last_scan.ranges[300:]) if not math.isnan(x)]
            left_ranges = [3.5 if math.isinf(x) else x for x in self.last_scan.ranges[30:90] if not math.isnan(x)]
            right_ranges = [3.5 if math.isinf(x) else x for x in self.last_scan.ranges[270:330] if not math.isnan(x)]

            front = sum(front_ranges) / len(front_ranges) if front_ranges else 0.0
            left = sum(left_ranges) / len(left_ranges) if left_ranges else 0.0
            right = sum(right_ranges) / len(right_ranges) if right_ranges else 0.0
            
            front_min = min(front_ranges) if front_ranges else 0.0
            left_min = min(left_ranges) if left_ranges else 0.0
            right_min = min(right_ranges) if right_ranges else 0.0

            if front_min < 0.3:
                linear_velocity = 0.1
                if front_min < 0.2:
                    linear_velocity = 0.05
                if front_min < 0.15:
                    linear_velocity = 0.0
                    
                self.get_logger().info('Obstacle too close! Stopping forward movement.')
                if left_min > right_min:
                    self.get_logger().info('Moving to the Left')
                    angular_velocity = 0.5
                else:
                    self.get_logger().info('Moving to the Right')
                    angular_velocity = -0.5
                
                self.orientation_pid.clean()
                
        self.change_velocity(linear=linear_velocity, angular=angular_velocity)
        
def main(args=None):
    rclpy.init(args=args)

    subscriber = MoverPID()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()