import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
#from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, PoseStamped
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
        
        # Publishers
        self.base_publisher = self.create_publisher(PointStamped, '/ball_position_base', 10)
        self.robot_position_publisher = self.create_publisher(PoseStamped, '/robot_position_odom', 10)  # NEW

        
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
        
        self.robot_base_frame = 'base_scan'  # Change this to your robot's base frame
        self.odom_frame = 'odom'  # Global reference frame

        self.timer = self.create_timer(0.1, self.publish_robot_position)  # NEW


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
            
            if center and (0.11 < radius < 0.16):
                x_lidar, y_lidar = center
                self.publish_transformed_marker(x_lidar, y_lidar, radius, msg)

    def cluster_points(self, points):
        clustering = DBSCAN(eps=0.12, min_samples=5).fit(points)
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
        point_lidar.header.frame_id = self.robot_base_frame  # Should be 'base_scan'
        point_lidar.header.stamp = msg.header.stamp
        point_lidar.point.x = x_lidar
        point_lidar.point.y = y_lidar
        point_lidar.point.z = 0.23  # Ground level

        try:
            # Transform to odom frame FIRST
            transform_to_odom = self.tf_buffer.lookup_transform(
                self.odom_frame,
                point_lidar.header.frame_id,
                rclpy.time.Time(),
            )
            point_odom = do_transform_point(point_lidar, transform_to_odom)

            # Then transform to base_link
            transform_to_base = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                self.odom_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            point_base = do_transform_point(point_odom, transform_to_base)

            base_point = PointStamped()
            base_point.header.frame_id = self.robot_base_frame
            base_point.point = point_base.point
            self.base_publisher.publish(base_point)
            
            self.get_logger().info(f"Ball position: ({point_base.point.x:.2f}, {point_base.point.y:.2f})")

        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"TF error: {str(e)}")
        
        
    def publish_robot_position(self):  # NEW METHOD
        try:
            # Get robot's position in odom frame
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Create and publish PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.odom_frame
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation
            
            self.get_logger().info(f"Robot position: ({pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f})")
            
            self.robot_position_publisher.publish(pose_msg)
            
        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Robot position TF error: {str(e)}", throttle_duration_sec=1)

def main(args=None):
    rclpy.init(args=args)
    detector = SimpleCircleDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
