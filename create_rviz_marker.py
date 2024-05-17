
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import time

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.array = ['Object1', 'Object2', 'Object3']
        self.user_input = None  # Define user_input as an instance variable
        self.map_frame_id = 'map'  # Define the map frame ID
        self.print_array_and_get_input()

    def odom_callback(self, msg):
        if self.user_input is not None:
            position = msg.pose.pose.position
            self.publish_marker(self.user_input, position)

    def print_array_and_get_input(self):
        for index, item in enumerate(self.array):
            print(f'{index}: {item}')
        
        self.user_input = int(input('Enter the number corresponding to the object to display: '))

    def publish_marker(self, index, position):
        if index < 0 or index >= len(self.array):
            self.get_logger().error('Invalid index')
            return

        # Text marker
        text_marker = Marker()
        text_marker.header.frame_id = "odom"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "image_description"
        text_marker.id = index + 1000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        print("x: " + str(position.x) + " y: " + str(position.y) + " z: " + str(position.z))
        text_marker.pose.position.x = position.x + 0.3
        text_marker.pose.position.y = position.y
        text_marker.pose.position.z = position.z + 1.0
        text_marker.pose.orientation.x = 0.5
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.2
        text_marker.color.a = 1.0
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
        text_marker.text = self.array[index]

        self.publisher_.publish(text_marker)
        self.get_logger().info(f'Published text marker for: {self.array[index]}')


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    
    try:
        rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
