import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import numpy as np
from sklearn.cluster import DBSCAN

class SimpleCircleDetector(Node):
    def __init__(self):
        super().__init__('simple_circle_detector')
        
        # Set up QoS profile for LiDAR communication
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
        
        self.map_frame_id = 'map'

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
                self.get_logger().info(f"Ball detected at {center} with radius {radius:.2f}m")
                self.publish_marker(center[0], center[1], radius)

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

    def publish_marker(self, x, y, radius):
        marker = Marker()
        marker.header.frame_id = self.map_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.scale.x = marker.scale.y = marker.scale.z = 2 * radius
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        self.publisher_.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    detector = SimpleCircleDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
