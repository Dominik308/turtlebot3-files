import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.neighbors import NearestNeighbors
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
        
        # Publishers
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.base_publisher = self.create_publisher(PointStamped, '/ball_position_base', 10)
        
        # Subscribe to LiDAR
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
        self.robot_base_frame = 'base_link'  # Change this to your robot's base frame
        
        self.min_cluster_points = 15
        self.max_cluster_diameter = 0.5  # 50cm

    def lidar_callback(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        distances = np.array(msg.ranges)
        valid = ~np.isinf(distances) & ~np.isnan(distances)

        x = distances[valid] * np.cos(angles[valid])
        y = distances[valid] * np.sin(angles[valid])
        points = np.column_stack((x, y))
        
        if len(points) < 10:
            return

        clusters = self.cluster_points(points)

        for cluster in clusters:
            center, radius = self.fit_circle(cluster)
            
            if center and (0.10 < radius < 0.15):
                x_lidar, y_lidar = center
                self.publish_transformed_marker(x_lidar, y_lidar, radius, msg)

    def cluster_points(self, points):
        """Enhanced clustering pipeline with preprocessing and validation"""
        # Step 1: Median filtering for noise reduction
        if len(points) > 5:
            points = np.array([np.median(points[max(0,i-2):i+3], axis=0) 
                             for i in range(len(points))])
        
        # Step 2: Adaptive parameter calculation
        eps = self.dynamic_eps_calculation(points)
        min_samples = max(3, int(len(points)*0.05))  # 5% of points as min samples
        
        # Step 3: DBSCAN clustering with optimized parameters
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        clusters = [points[clustering.labels_ == i] 
                   for i in set(clustering.labels_) if i != -1]
        
        # Step 4: Post-processing validation
        valid_clusters = []
        for cluster in clusters:
            if len(cluster) < self.min_cluster_points:
                continue
            if not self.validate_cluster_density(cluster):
                continue
            valid_clusters.append(cluster)
            
        return valid_clusters

    def dynamic_eps_calculation(self, points):
        """Calculate adaptive EPS based on point density"""
        if len(points) < 4:
            return 0.12  # Default value
            
        nn = NearestNeighbors(n_neighbors=4).fit(points)
        distances, _ = nn.kneighbors(points)
        return np.percentile(distances[:, -1], 50) * 1.2  # Median distance * 1.2

    def validate_cluster_density(self, cluster):
        """Ensure cluster has sufficient point density"""
        pairwise_dist = np.linalg.norm(cluster[:, None] - cluster, axis=2)
        avg_density = np.mean(np.sum(pairwise_dist < 0.15, axis=1))
        return avg_density > 4

    def validate_cluster_shape(self, cluster):
        """Check cluster size and aspect ratio"""
        min_coords = np.min(cluster, axis=0)
        max_coords = np.max(cluster, axis=0)
        bbox_size = max_coords - min_coords
        
        # Check maximum dimension
        if np.linalg.norm(bbox_size) > self.max_cluster_diameter:
            return False
            
        # Check aspect ratio (should be roughly circular)
        aspect_ratio = max(bbox_size) / min(bbox_size)
        return aspect_ratio < 2.5

    def is_valid_circle(self, center, radius):
        """Validate circle parameters"""
        if not center or not radius:
            return False
        return (0.08 < radius < 0.18 and 
                abs(center[0]) < 5.0 and 
                abs(center[1]) < 5.0)

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
            # Get transforms
            transform_to_odom = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                point_lidar.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            transform_to_base = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                point_lidar.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            # Transform points
            point_odom = do_transform_point(point_lidar, transform_to_odom)
            point_base = do_transform_point(point_lidar, transform_to_base)

            # Publish odom marker
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.type = Marker.SPHERE
            marker.scale.x = 2 * radius
            marker.scale.y = 2 * radius
            marker.scale.z = 2 * radius
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.position.x = point_odom.point.x
            marker.pose.position.y = point_odom.point.y
            self.marker_publisher.publish(marker)

            # Publish base_link coordinates
            base_point = PointStamped()
            base_point.header.frame_id = self.robot_base_frame
            base_point.header.stamp = self.get_clock().now().to_msg()
            base_point.point = point_base.point
            self.base_publisher.publish(base_point)

            # Log both coordinates
            self.get_logger().info(
                f"Global (odom): ({point_odom.point.x:.2f}, {point_odom.point.y:.2f}) | "
                f"Relative (base): ({point_base.point.x:.2f}, {point_base.point.y:.2f})",
                throttle_duration_sec=1
            )

        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
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
