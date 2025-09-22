import rclpy
from rclpy.node import Node
import math
import json
import time
import matplotlib.pyplot as plt

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # This import is necessary for do_transform_point

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
        self.robot_marker = None
        
        # --- Configure the plot's static elements ---
        # Plot the robot's initial position as a red triangle
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Live LIDAR Scan - 2D Coordinates')
        self.ax.axis('equal') # Set aspect ratio to be equal
        self.ax.grid(True)
        self.ax.legend()

    def update_plot(self, robot_position, points):
        """
        Clears the old LIDAR points and plots the new ones.
        """
        if not points:
            return

        # 3. Remove the old LIDAR scatter points if they exist
        if self.lidar_scatter:
            self.lidar_scatter.remove()
            
        if self.robot_marker:
            self.robot_marker.remove()

        # Unzip the points into x and y lists
        x_vals, y_vals = zip(*points)
        
        # Rotate points 90 degrees CCW: (x', y') = (-y, x)
        x_vals_adjusted = [-y for y in y_vals]
        y_vals_adjusted = x_vals

        robot_x = - robot_position[1]
        robot_y = robot_position[0]

        # 4. Plot the new data
        self.lidar_scatter = self.ax.scatter(x_vals_adjusted, y_vals_adjusted, s=2, color='blue', label='LIDAR Points')
        self.robot_marker = self.ax.scatter(robot_x, robot_y, color='red', marker='^', s=100, label='Robot')

        # Dynamically adjust plot limits to fit the data, with a margin
        self.ax.set_xlim(min(x_vals_adjusted) - 0.5, max(x_vals_adjusted) + 0.5)
        self.ax.set_ylim(min(y_vals_adjusted) - 0.5, max(y_vals_adjusted) + 0.5)

        # 5. Redraw the canvas and process GUI events
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        # A short pause is essential to allow the GUI to update
        plt.pause(0.001)


class LidarOdom2DScanner(Node):

    def __init__(self):
        super().__init__('LidarOdom2DScanner')

        # Create a TF2 buffer and listener
        # The buffer stores received transforms, and the listener populates it.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning
        self.points_odom = set()
        self.robot_position = (0.0, 0.0)
        self.visualizer = LidarVisualizer()

    def listener_callback(self, msg):
        points = []
        points_odom = []
        
        target_frame = 'odom'
        source_frame = 'base_scan'  # Assuming the LIDAR frame is 'base_scan'

        try:
            # Look up the transform from the LIDAR frame to the odom frame
            # This is the "magic" that gets the robot's position and orientation.
            #transform = self.tf_buffer.lookup_transform(
            #    target_frame,
            #    source_frame,
            #    rclpy.time.Time(), # <-- Gets the transform at the EXACT time of the scan
            #)            
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                msg.header.stamp, # <-- Gets the transform at the EXACT time of the scan
                timeout=rclpy.duration.Duration(seconds=0.1) # Best practice: add a timeout
            )            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform from {source_frame} to {target_frame}: {e}')
            return        
        
        for i, distance in enumerate(msg.ranges):
            # Ignore invalid readings
            if math.isinf(distance) or math.isnan(distance):
                continue

            angle = msg.angle_min + i * msg.angle_increment

            # Convert to cartesian coordinates
            point_x = distance * math.cos(angle)
            point_y = distance * math.sin(angle)

            # Rotate it 90 degrees CCW: (x', y') = (-y, x)

            points.append((point_x, point_y))
            
            # Create a PointStamped message for the point in the LIDAR frame
            point_in_lidar_frame = PointStamped()
            point_in_lidar_frame.header = msg.header
            point_in_lidar_frame.point.x = point_x
            point_in_lidar_frame.point.y = point_y
            point_in_lidar_frame.point.z = 0.0 # Assuming a 2D LIDAR            
           
            point_in_odom_frame = tf2_geometry_msgs.do_transform_point(point_in_lidar_frame, transform)
            
            odom_x = point_in_odom_frame.point.x
            odom_y = point_in_odom_frame.point.y
            
            self.points_odom.add( (odom_x, odom_y) )
            points_odom.append( (odom_x, odom_y) )

        #if self.points_odom:
        if points_odom:
            point_in_lidar_frame = PointStamped()
            point_in_lidar_frame.header = msg.header
            point_in_lidar_frame.point.x = 0.0
            point_in_lidar_frame.point.y = 0.0
            point_in_lidar_frame.point.z = 0.0 # Assuming a 2D LIDAR            

            point_in_odom_frame = tf2_geometry_msgs.do_transform_point(point_in_lidar_frame, transform)
            self.robot_position = (point_in_odom_frame.point.x, point_in_odom_frame.point.y)
            
            self.visualizer.update_plot(self.robot_position, list(self.points_odom))
            # self.visualizer.update_plot(self.robot_position, points_odom)
            #  print(json.dumps(list(self.points_odom), indent=2))

def main(args=None):
    rclpy.init(args=args)

    subscriber = LidarOdom2DScanner()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()