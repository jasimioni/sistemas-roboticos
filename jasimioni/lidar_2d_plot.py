import rclpy
from rclpy.node import Node
import math
import json
import time
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan

class LidarVisualizer:
    """
    A class to create a non-blocking, live-updating plot for LIDAR data.
    """
    def __init__(self):
        # 1. Enable Interactive Mode
        plt.ion()
        
        # 2. Create the figure and axes ONCE
        self.fig, self.ax = plt.subplots()
        
        # Keep track of the scatter plot object to update it
        self.lidar_scatter = None
        
        # --- Configure the plot's static elements ---
        # Plot the robot's position as a red triangle
        self.ax.scatter(0, 0, color='red', marker='^', s=100, label='Robot')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Live LIDAR Scan - 2D Coordinates')
        self.ax.axis('equal') # Set aspect ratio to be equal
        self.ax.grid(True)
        self.ax.legend()

    def update_plot(self, points):
        """
        Clears the old LIDAR points and plots the new ones.
        """
        if not points:
            return

        # 3. Remove the old LIDAR scatter points if they exist
        if self.lidar_scatter:
            self.lidar_scatter.remove()

        # Unzip the points into x and y lists
        x_vals, y_vals = zip(*points)

        # 4. Plot the new data
        self.lidar_scatter = self.ax.scatter(x_vals, y_vals, s=5, label='LIDAR Points')

        # Dynamically adjust plot limits to fit the data, with a margin
        self.ax.set_xlim(min(x_vals) - 0.5, max(x_vals) + 0.5)
        self.ax.set_ylim(min(y_vals) - 0.5, max(y_vals) + 0.5)

        # 5. Redraw the canvas and process GUI events
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        # A short pause is essential to allow the GUI to update
        plt.pause(0.001)


class Lidar2DScanner(Node):

    def __init__(self):
        super().__init__('Lidar2DScanner')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning
        self.visualizer = LidarVisualizer()

    def listener_callback(self, msg):
        points = []
        for i, distance in enumerate(msg.ranges):
            # Ignore invalid readings
            if math.isinf(distance) or math.isnan(distance):
                continue

            # Calcula o ângulo para a medição atual
            # ângulo = ângulo_inicial + (índice_da_leitura * incremento_angular)
            angle = msg.angle_min + i * msg.angle_increment

            # Convert to cartesian coordinates
            point_x = distance * math.cos(angle)
            point_y = distance * math.sin(angle)

            # Rotate it 90 degrees CCW: (x', y') = (-y, x)
            point_x, point_y = -point_y, point_x

            points.append((point_x, point_y))

        if points:
            self.visualizer.update_plot(points)
            #x_vals, y_vals = zip(*points)
            #plt.figure()
            #plt.scatter(x_vals, y_vals, s=5)
            #plt.scatter(0, 0, color='red', marker='^', s=100)
            #plt.xlabel('X (m)')
            #plt.ylabel('Y (m)')
            #plt.title('LIDAR Scan - 2D Coordinates')
            #plt.axis('equal')
            #plt.ion()

            print(json.dumps(points, indent=2))


def main(args=None):
    rclpy.init(args=args)

    subscriber = Lidar2DScanner()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()