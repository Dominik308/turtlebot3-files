import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from scipy.optimize import least_squares
from typing import Optional, Tuple
import random
import tf2_ros
import tf2_geometry_msgs

class BallDetector(Node):
    def __init__(self):
        super().__init__('lidar_arc_detector')
        self.user_input = None  # Set user_input as an instance variable
        self.map_frame_id = 'odom'  # Use 'odom' as the frame ID for consistency
        
        #Set up QoS profile for LiDAR communication
        self.qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        #Publisher for the visualization marker
        self.publisher_ = self.create_publisher(
            Marker, 
            '/visualization_marker', 
            10
        )
        
        #Publisher for the detected ball position
        self.publisher = self.create_publisher(
            Point,
            '/ball_position',
            self.qos_profile
        )
        
        #Subscribe to the LiDAR /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            self.qos_profile
        )

    def lidar_callback(self, msg: LaserScan) -> None:
        #Process LiDAR scan and detect ball using circle fitting.
        ranges = np.array(msg.ranges)  #Distance measurements from LiDAR
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))  #Corresponding angles
        #print(angles)
        # Filter valid LiDAR points
        valid_points = (ranges > msg.range_min) & (ranges < msg.range_max)
        if not np.any(valid_points):
            return
        
        x_coords = ranges[valid_points] * np.cos(angles[valid_points])
        y_coords = ranges[valid_points] * np.sin(angles[valid_points])

        if len(x_coords) > 5:
            #Fit a circle to the detected points
            circle_params = self.fit_circle(x_coords, y_coords)
            if circle_params is not None:
                x_center, y_center, radius = circle_params
                self.get_logger().info(f"Ball detected at ({x_center:.2f}, {y_center:.2f}), Radius: {radius:.2f}")
                
                #Publish the detected ball position
                ball_position = Point()
                ball_position.x = x_center
                ball_position.y = y_center
                ball_position.z = 0.0  #Assuming the ball is on a flat surface
                self.publisher.publish(ball_position)
                
                # Text marker
                text_marker = Marker()
                text_marker.header.frame_id = self.map_frame_id  # Ensure the frame ID matches the odometry frame
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = "image_description"
                text_marker.id = random.randint(0, 1000)
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD

                # Adjust the marker's position based on odometry and description coordinates
                text_marker.pose.position.x = 2.0
                text_marker.pose.position.y = -1.0
                text_marker.pose.position.z = 0.5
                text_marker.scale.z = 0.2
                text_marker.color.a = 1.0
                text_marker.color.r = 0.0
                text_marker.color.g = 1.0  # Green for visibility
                text_marker.color.b = 0.0
                text_marker.text = "Ball" 

                self.publisher_.publish(text_marker)

    def fit_circle(self, x_coords: np.ndarray, y_coords: np.ndarray) -> Optional[Tuple[float, float, float]]:
        #Fits a circle to the given points using Least Squares.
        x_mean = np.mean(x_coords)
        y_mean = np.mean(y_coords)

        def algebraic_circle(params: Tuple[float, float, float], x: np.ndarray, y: np.ndarray) -> np.ndarray:
            x_center, y_center, radius = params
            return (x - x_center) ** 2 + (y - y_center) ** 2 - radius ** 2

        initial_guess = [x_mean, y_mean, np.mean(np.sqrt((x_coords - x_mean) ** 2 + (y_coords - y_mean) ** 2))]
        result = least_squares(algebraic_circle, initial_guess, args=(x_coords, y_coords))

        if result.success:
            return result.x
        else:
            return None

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = BallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
