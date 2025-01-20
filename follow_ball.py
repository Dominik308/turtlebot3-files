import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
from typing import Optional

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        
        # Set up QoS profile for LiDAR communication
        self.qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to the ball position topic
        self.subscription = self.create_subscription(
            Point,
            '/ball_position',
            self.ball_position_callback,
            self.qos_profile
        )
        
        # Publisher for the robot's velocity
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.target_position = None
        self.get_logger().info("BallFollower node has been started.")

    def ball_position_callback(self, msg: Point) -> None:
        """Callback to handle the ball position."""
        self.get_logger().info(f"Received ball position: x={msg.x}, y={msg.y}, z={msg.z}")
        self.target_position = msg
        self.drive_to_ball()

    def drive_to_ball(self) -> None:
        """Drive the robot to the ball position."""
        if self.target_position is None:
            self.get_logger().warn("No target position set.")
            return
        
        # Calculate the distance and angle to the target position
        distance = sqrt(self.target_position.x**2 + self.target_position.y**2)
        angle = atan2(self.target_position.y, self.target_position.x)
        
        self.get_logger().info(f"Calculated distance: {distance}, angle: {angle}")
        
        twist = Twist()
        
        if abs(angle) > 0.1:  # Threshold to avoid oscillation
            twist.linear.x = 0.0
            twist.angular.z = max(min(angle, 1.5), -1.5) 
        else:
            # Move forward towards the ball
            twist.linear.x = min(distance, 0.2)
            twist.angular.z = 0.0
                
        # Publish the velocity command
        self.publisher.publish(twist)
        self.get_logger().info(f"Published velocity command: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = BallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
