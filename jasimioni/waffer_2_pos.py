import rclpy
from rclpy.node import Node
import math
import tf2_ros
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped

class WafferMover(Node):
    def __init__(self):
        super().__init__('WafferMover')

        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.pos = None
    
        self.positions = [
            (2.0, 0.0),
            (-2.0, 0.0),
            (0.0, 2.0),
            (0.0, -2.0),
        ]
        
        self.target = None
        self.go_to_next_position = True
        
        self.stuck_counter = 0
        self.prev_pos = None

    def timer_callback(self):
        
        to_frame = 'map'
        from_frame = 'base_link'
        
        try:
            # Look up the transform from 'base_link' to 'map'
            # We are asking "what is the pose of from_frame in the to_frame?"
            t = self.tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            
            # Extract position (translation)
            pos = t.transform.translation
            
            self.pos = {
                'x': pos.x,
                'y': pos.y
            }
            
            # Extract orientation (quaternion) and convert to yaw
            quat = t.transform.rotation
            _, _, yaw = self.euler_from_quaternion(quat)
            yaw_deg = math.degrees(yaw)
            
            self.get_logger().info(
                f"Robot is at (x={pos.x:.2f}, y={pos.y:.2f}) with yaw={yaw_deg:.2f}° in '{to_frame}' frame."
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Could not transform {from_frame} to {to_frame}: {ex}')   
            return
            
        if self.go_to_next_position:
            target = self.positions.pop(0) if self.positions else None
            if target is None:
                self.get_logger().info('All positions reached.')
                self.go_to_next_position = False
                self.target = None
                return
            
            self.positions.append(target)  # Re-add to the end of the list
            
            self.target = {
                'x': target[0],
                'y': target[1]
            }

            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = target[0]
            goal_msg.pose.position.y = target[1]
            goal_msg.pose.position.z = 0.0
            goal_msg.pose.orientation.w = 1.0  # Facing forward
            
            self.publisher.publish(goal_msg)
            self.get_logger().info(f'Published new goal position: {target}')
            
            self.go_to_next_position = False
            
        else:
            current_target = self.target
            if current_target is None:
                return
            distance = math.sqrt((self.pos['x'] - current_target['x'])**2 + (self.pos['y'] - current_target['y'])**2)
            if distance < 0.25:
                self.get_logger().info(f'Reached target position: {current_target}')
                self.go_to_next_position = True
            elif self.stuck_counter > 5:
                self.get_logger().warn('Robot seems to be stuck, moving to next target.')
                self.go_to_next_position = True
                self.stuck_counter = 0
                self.prev_pos = None
            else:
                if self.prev_pos is not None:
                    if distance == self.prev_pos:
                        self.stuck_counter += 1
                    else:
                        self.stuck_counter = 0
                self.prev_pos = distance
                    
                self.get_logger().info(f'Current position: ({self.pos["x"]:.2f}, {self.pos["y"]:.2f}), Target: ({current_target["x"]}, {current_target["y"]}), Distance: {distance:.2f}')
                
    def euler_from_quaternion(self, q):
        """Convert a quaternion into euler angles (roll, pitch, yaw)."""
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
     
        return roll_x, pitch_y, yaw_z # in radians
        
        
def main(args=None):
    rclpy.init(args=args)

    subscriber = WafferMover()

    rclpy.spin(subscriber)

    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()