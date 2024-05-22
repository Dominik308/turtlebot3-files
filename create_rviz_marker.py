import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.description_subscription = self.create_subscription(String, '/description', self.description_callback, 1)
        self.user_input = None  # Set user_input as an instance variable
        self.map_frame_id = 'odom'  # Use 'odom' as the frame ID for consistency
        self.descriptions = []  # To store the parsed descriptions

    def odom_callback(self, msg):
        if self.user_input is not None and self.descriptions:
            position = msg.pose.pose.position
            self.publish_marker(self.user_input, position)

    def description_callback(self, msg):
        self.get_logger().info('Received descriptions message')
        raw_descriptions = eval(msg.data)  # Convert the string to a list
        self.get_logger().info(f'Raw descriptions: {raw_descriptions}')

        # Parse descriptions
        parsed_descriptions = []
        for desc_str in raw_descriptions:
            desc_str = desc_str.strip()  # Remove leading and trailing whitespace
            parts = desc_str.split()
            if len(parts) == 7:
                try:
                    parsed_desc = {
                        'type': parts[0],
                        'id': int(parts[1]),
                        'x': float(parts[2]),
                        'y': float(parts[3]),
                        'width': float(parts[4]),
                        'height': float(parts[5]),
                    }
                    parsed_descriptions.append(parsed_desc)
                except ValueError as e:
                    self.get_logger().error(f'Error parsing description: {e}')
            else:
                self.get_logger().error(f'Invalid description format: {desc_str}')
        
        self.descriptions = parsed_descriptions
        self.descriptions.sort(key=lambda x: x['type'])
        self.get_logger().info(f'Received and parsed descriptions: {self.descriptions}')
        self.user_input = get_user_input(self.descriptions)

    def publish_marker(self, index, position):
        if index < 0 or index >= len(self.descriptions):
            self.get_logger().error('Invalid index')
            return

        description = self.descriptions[index]

        # Text marker
        text_marker = Marker()
        text_marker.header.frame_id = self.map_frame_id  # Ensure the frame ID matches the odometry frame
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "image_description"
        text_marker.id = description['id'] + 1000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        print("Robots position x: " + str(position.x) + " y: " + str(position.y))

        # Adjust the marker's position based on odometry and description coordinates
        text_marker.pose.position.x = position.x + description['x']
        text_marker.pose.position.y = (position.y - description['y']) - description['y']
        text_marker.pose.position.z = position.z + 1.0  # Elevate the marker slightly for visibility
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.2
        text_marker.color.a = 1.0
        text_marker.color.r = 0.0
        text_marker.color.g = 1.0  # Green for visibility
        text_marker.color.b = 0.0
        text_marker.text = f"{description['type']}"

        self.publisher_.publish(text_marker)
        self.get_logger().info(f'Published text marker for: {description["type"]}')

def get_user_input(descriptions):
    print("Choose an item to display:")
    for index, desc in enumerate(descriptions):
        print(f'{index}: {desc["type"]} (ID: {desc["id"]}, x: {desc["x"]}, y: {desc["y"]}, width: {desc["width"]}, height: {desc["height"]})')
    
    while True:
        try:
            user_input = int(input('Enter the number corresponding to the item to display: '))
            if 0 <= user_input < len(descriptions):
                return user_input
            else:
                print("Invalid index. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
