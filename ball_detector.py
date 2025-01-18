import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.optimize import least_squares

class BallDetector(Node):
    def __init__(self):
        super().__init__('lidar_arc_detector')
        
        # QoS profile to handle the LiDAR sensor's best-effort communication
        self.qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to the LiDAR /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            self.qos_profile)
        
        # Publisher for the detected ball position
        self.publisher = self.create_publisher(
            Point,
            '/ball_position',
            self.qos_profile)

    def lidar_callback(self, msg):
        """Process LiDAR scan and detect ball using circle fitting."""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter valid LiDAR points
        valid = (ranges > msg.range_min) & (ranges < msg.range_max)
        if not np.any(valid):
            return
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])

        if len(x) < 5:  # Not enough points to fit a circle
            return

        # Fit a circle to the detected arc
        circle_params = self.fit_circle(x, y)
        if circle_params is not None:
            xc, yc, radius = circle_params
            self.get_logger().info(f"Ball detected at ({xc:.2f}, {yc:.2f}), Radius: {radius:.2f}")
            
            # Publish the detected ball position
            ball_position = Point()
            ball_position.x = xc
            ball_position.y = yc
            ball_position.z = 0.0  # Assuming the ball is on a flat surface
            self.publisher.publish(ball_position)

    def fit_circle(self, x, y):
        """Fits a circle to the given points using Least Squares."""
        x_m = np.mean(x)
        y_m = np.mean(y)

        def algebraic_circle(params, x, y):
            xc, yc, r = params
            return (x - xc) ** 2 + (y - yc) ** 2 - r ** 2

        initial_guess = [x_m, y_m, np.mean(np.sqrt((x - x_m) ** 2 + (y - y_m) ** 2))]
        result = least_squares(algebraic_circle, initial_guess, args=(x, y))

        if result.success:
            return result.x  # (xc, yc, radius)
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
