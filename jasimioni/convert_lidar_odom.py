import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # This import is necessary for do_transform_point
import math

class LidarOdomTransformer(Node):
    """
    A node that subscribes to a LaserScan, converts the points to Cartesian coordinates,
    and transforms them into the 'odom' frame.
    """
    def __init__(self):
        super().__init__('lidar_odom_transformer')
        
        # 1. Create a TF2 buffer and listener
        # The buffer stores received transforms, and the listener populates it.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 2. Create a subscriber to the '/scan' topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.get_logger().info('Lidar to Odom Transformer node has been started.')

    def scan_callback(self, msg: LaserScan):
        """
        Callback function for the LaserScan subscriber.
        """
        target_frame = 'odom'
        source_frame = msg.header.frame_id

        try:
            # 3. Look up the transform from the LIDAR frame to the odom frame
            # This is the "magic" that gets the robot's position and orientation.
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time() # Get the latest available transform
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform from {source_frame} to {target_frame}: {e}')
            return

        # 4. Convert polar coordinates (ranges) to Cartesian (x, y) and transform them
        points_in_odom_frame = []
        for i, range_reading in enumerate(msg.ranges):
            # Skip invalid range readings (inf or nan)
            if not math.isfinite(range_reading):
                continue

            # Calculate the angle for the current reading
            angle = msg.angle_min + i * msg.angle_increment
            
            # Convert polar to Cartesian in the LIDAR's frame
            x_lidar = range_reading * math.cos(angle)
            y_lidar = range_reading * math.sin(angle)
            
            # Create a PointStamped message for the point in the LIDAR frame
            point_in_lidar_frame = PointStamped()
            point_in_lidar_frame.header = msg.header
            point_in_lidar_frame.point.x = x_lidar
            point_in_lidar_frame.point.y = y_lidar
            point_in_lidar_frame.point.z = 0.0 # Assuming a 2D LIDAR

            # 5. Apply the transform to the point
            point_in_odom_frame = tf2_geometry_msgs.do_transform_point(point_in_lidar_frame, transform)
            points_in_odom_frame.append(point_in_odom_frame.point)

        # Log the first 5 transformed points for verification
        self.get_logger().info(f'--- Successfully transformed {len(points_in_odom_frame)} points ---')
        for i, point in enumerate(points_in_odom_frame[:5]):
             self.get_logger().info(f'Point {i} in odom frame: (x: {point.x:.2f}, y: {point.y:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = LidarOdomTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()