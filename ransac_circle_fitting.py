import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import numpy as np
from sklearn.cluster import DBSCAN

class HalfCircleDetector(Node):
    def __init__(self):
        super().__init__('half_circle_detector')
        
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
        
        #Subscribe to the LiDAR /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            self.qos_profile
        )
        self.map_frame_id = 'map'  # Adjust to your frame

    def cluster_points(self, points):
        """Cluster points using DBSCAN to isolate the ball."""
        clustering = DBSCAN(eps=0.15, min_samples=3).fit(points)
        labels = clustering.labels_
        clusters = []
        for label in np.unique(labels):
            if label == -1:
                continue  # Skip noise
            clusters.append(points[labels == label])
        return clusters

    def ransac_circle_fit(self, points, max_iters=50, threshold=0.03):
        """Robust circle fitting using RANSAC."""
        best_inliers = []
        best_center = None
        best_radius = None
        
        if len(points) < 3:
            return None, None, None
            
        for _ in range(max_iters):
            sample = points[np.random.choice(len(points), 3, replace=False)]
            try:
                D, E, F, a, b, radius = self.algebraic_circle_fit(sample[:,0], sample[:,1])
                distances = np.sqrt((points[:,0]-a)**2 + (points[:,1]-b)**2)
                inliers = np.abs(distances - radius) < threshold
                if np.sum(inliers) > np.sum(best_inliers):
                    best_inliers = inliers
                    best_center = (a, b)
                    best_radius = radius
            except:
                continue
                
        return best_center, best_radius, best_inliers

    def algebraic_circle_fit(self, x, y):
        """Helper function for algebraic circle fitting."""
        A = np.column_stack([x, y, np.ones_like(x)])
        B = -(x**2 + y**2)
        D, E, F = np.linalg.lstsq(A, B, rcond=None)[0]
        a = -D/2.0
        b = -E/2.0
        radius = np.sqrt(a**2 + b**2 - F)
        return D, E, F, a, b, radius

    def lidar_callback(self, msg):
        # Convert to Cartesian coordinates
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        valid = ~np.isinf(ranges) & ~np.isnan(ranges)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        points = np.column_stack((x, y))
        
        if len(points) < 5:
            return

        # Cluster points to isolate the ball
        clusters = self.cluster_points(points)
        
        for cluster in clusters:
            # Robust circle fitting with RANSAC
            center, radius, inliers = self.ransac_circle_fit(cluster)
            
            if center is None or radius is None:
                continue
                
            # Validate expected ball size (12.5cm radius)
            if not (0.10 < radius < 0.15):
                continue
                
            # Require minimum 5 points forming the arc
            if np.sum(inliers) >= 5:
                self.get_logger().info(f"Detected ball at {center} r={radius:.2f}m")
                self.publish_marker(center[0], center[1], radius)

    def publish_marker(self, x, y, radius):
        marker = Marker()
        marker.header.frame_id = self.map_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 2*radius
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    detector = HalfCircleDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
