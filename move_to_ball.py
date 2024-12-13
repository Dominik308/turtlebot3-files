import socket
import struct
import pickle
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from math import atan2
from gpiozero import Button

class CoordinateToCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('coordinate_to_cmd_vel_publisher')

        # Initialize the button (bumper) sensor connected to GPIO pin 4
        self.bumper = Button(4, pull_up=False)
        
        # Publisher to publish cmd_vel messages
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publisher for bumper state
        self.bumper_publisher = self.create_publisher(Bool, 'BumperSensorInfo', 1)

        # Timer to publish bumper state
        self.bumper_timer = self.create_timer(0.1, self.publish_bumper)  # Adjust timer_period as necessary

        # Set up the socket for receiving data
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', 9999))  # Listening for connections on port 9998
        self.server_socket.listen(1)
        self.get_logger().info("Waiting for a connection...")

        try:
            self.conn, addr = self.server_socket.accept()
            self.get_logger().info(f"Connection established with {addr}")
        except Exception as e:
            self.get_logger().error(f"Failed to establish connection: {e}")
            self.server_socket.close()
            raise

        self.bump_pressed = False  # Initialize bumper state

    def receive_data(self):
        try:
            # Receive size of the incoming data
            payload_size = struct.calcsize("Q")
            data = self.conn.recv(payload_size)
            if not data:
                return None

            data_size = struct.unpack("Q", data)[0]  # Extract size of the data

            # Now receive the actual payload
            data = b""
            while len(data) < data_size:
                packet = self.conn.recv(4096)
                if not packet:
                    return None
                data += packet

            return data
        except Exception as e:
            self.get_logger().error(f"Error receiving data: {e}")
            return None

    def publish_bumper(self):
        """Publish the bumper state."""
        msg = Bool()  # Create a new Bool message
        msg.data = self.bumper.is_pressed  # Set the message to the state of the bumper
        self.bumper_publisher.publish(msg)  # Publish the message
        self.get_logger().info('Publishing bumper state: "%s"' % msg.data)

    def calculate_cmd_vel(self, coordinates):
        """Calculate the command velocities to drive towards the ball."""
        target_x = coordinates[0]
        target_y = coordinates[1]  # Assuming 2D coordinates
        twist = Twist()

        # Calculate Euclidean distance to the ball
        distance_to_ball = (target_x**2 + target_y**2)**0.5
        
        if distance_to_ball > 0.5 and not self.bump_pressed:  # Move towards the ball if it's more than 0.5 meters away
            twist.linear.x = min(0.5, distance_to_ball)  # Move forward at max speed of 0.5 m/s
            angle_to_ball = atan2(target_y, target_x)
            twist.angular.z = angle_to_ball  # Simple proportional control for turning
        else:
            twist.linear.x = 0.0  # Stop moving forward
            twist.angular.z = 0.0  # Stop turning

        return twist

    def run(self):
        try:
            while True:
                coordinates_data = self.receive_data()
                if coordinates_data is None:
                    self.get_logger().info("Connection closed.")
                    break

                # Deserialize the coordinates into a tuple
                coordinates = pickle.loads(coordinates_data)

                # Ensure the received data is a tuple
                if isinstance(coordinates, tuple):
                    self.get_logger().info(f"Ball center at: {coordinates}")

                    # Calculate and publish cmd_vel message
                    cmd_vel_msg = self.calculate_cmd_vel(coordinates)
                    self.cmd_vel_publisher.publish(cmd_vel_msg)
                    self.get_logger().info(f"Published cmd_vel: linear.x={cmd_vel_msg.linear.x}, angular.z={cmd_vel_msg.angular.z}")
                else:
                    self.get_logger().warning("Received data is not a tuple.")

        except Exception as e:
            self.get_logger().error(f"Error in run loop: {e}")
        finally:
            self.conn.close()
            self.server_socket.close()

def main(args=None):
    rclpy.init(args=args)
    publisher_node = CoordinateToCmdVelPublisher()
    try:
        publisher_node.run()
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
