import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import numpy as np
from sklearn.cluster import DBSCAN
import tf2_ros

class SimpleCircleDetector(Node):
    def __init__(self):
        super().__init__('simple_circle_detector')
        
        # Set up QoS profile for LiDAR
        self.qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publisher for the visualization marker
        self.publisher_ = self.create_publisher(
            Marker, 
            '/visualization_marker', 
            10
        )
        
        # Subscribe to the LiDAR /scan topic
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            self.qos_profile
        )
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.map_frame_id = 'odom'

    def lidar_callback(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        distances = np.array(msg.ranges)
        valid = ~np.isinf(distances) & ~np.isnan(distances)

        x = distances[valid] * np.cos(angles[valid])
        y = distances[valid] * np.sin(angles[valid])
        points = np.column_stack((x, y))
        
        if len(points) < 5:
            return

        clusters = self.cluster_points(points)

        for cluster in clusters:
            center, radius = self.fit_circle(cluster)
            
            if center and (0.10 < radius < 0.15):
                x_lidar, y_lidar = center
                success = self.publish_transformed_marker(x_lidar, y_lidar, radius, msg)

    def cluster_points(self, points):
        clustering = DBSCAN(eps=0.15, min_samples=3).fit(points)
        clusters = [points[clustering.labels_ == i] for i in set(clustering.labels_) if i != -1]
        return clusters

    def fit_circle(self, points):
        if len(points) < 10:
            return None, None
        
        x, y = points[:, 0], points[:, 1]
        A = np.column_stack([x, y, np.ones_like(x)])
        B = -(x**2 + y**2)
        D, E, F = np.linalg.lstsq(A, B, rcond=None)[0]
        a, b = -D/2, -E/2
        radius = np.sqrt(a**2 + b**2 - F)
        return (a, b), radius

    def publish_transformed_marker(self, x_lidar, y_lidar, radius, msg):
        # Create PointStamped in LiDAR frame
        point_lidar = PointStamped()
        point_lidar.header.frame_id = msg.header.frame_id
        point_lidar.header.stamp = msg.header.stamp
        point_lidar.point.x = x_lidar
        point_lidar.point.y = y_lidar
        point_lidar.point.z = 0.5

        try:
            # Get transform to map frame using the latest available transform
            # Allow some time for the transform to become available
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                point_lidar.header.frame_id,
                rclpy.time.Time(),  # Use latest available transform
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            # Transform point to map frame
            point_map = do_transform_point(point_lidar, transform)
            
            # Publish marker
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.scale.x = marker.scale.y = marker.scale.z = 2 * radius
            marker.color.a = 0.5
            marker.color.r = 1.0
            marker.pose.position.x = point_map.point.x
            marker.pose.position.y = point_map.point.y
            self.publisher_.publish(marker)
            
            self.get_logger().info(f"Ball at ({point_map.point.x:.2f}, {point_map.point.y:.2f})", throttle_duration_sec=1)
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF error: {str(e)}", throttle_duration_sec=1)
            return False

def main(args=None):
    rclpy.init(args=args)
    detector = SimpleCircleDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
