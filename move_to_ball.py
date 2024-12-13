import socket
import struct
import pickle
import rclpy
from std_msgs.msg import String, Bool
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Button
import time


class CoordinateToCmdVelPublisher(Node):
    def __init__(self):
        super().__init__('coordinate_to_cmd_vel_publisher')
        
        # Publisher to publish cmd_vel messages
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Define base speeds as class attributes
        self.base_turning_speed = 0.9
        self.base_driving_speed = 0.2

        # Initialize Bumper of TurtleBot3
        self.bumper = Button(4, pull_up=False)

        # Set timer period for sensor data publishing
        timer_period = 0.1
        self.bumper_publisher = self.create_publisher(Bool, 'BumperSensorInfo', 1)
        self.bumper_timer = self.create_timer(timer_period, self.publish_bumper)
        
        # Set up the socket for receiving data
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', 9999))
        self.server_socket.listen(1)
        self.get_logger().info("Waiting for a connection...")
        
        try:
            self.conn, addr = self.server_socket.accept()
            self.get_logger().info(f"Connection established with {addr}")
        except Exception as e:
            self.get_logger().error(f"Failed to establish connection: {e}")
            self.server_socket.close()
            raise

    # Publish bumper sensor data
    def publish_bumper(self):
        msg = Bool()
        msg.data = self.bumper.is_pressed
        self.bumper_publisher.publish(msg)
        self.get_logger().info(f'Publishing bumper state: {msg.data}')

    # Clean up method when the object is deleted
    def __del__(self):
        self.bumper.close()  # Close the bumper sensor

    # Method to receive data over socket
    def receive_data(self):
        try:
            payload_size = struct.calcsize("Q")
            data = self.conn.recv(payload_size)
            if not data:
                return None

            data_size = struct.unpack("Q", data)[0]
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

    @staticmethod
    def lerp(start, end, t):
        return start + t * (end - start)

    def calculate_cmd_vel(self, coordinates):
        # Extract x-coordinate to determine position
        target_x = coordinates[0]

        # Create a Twist message to set movement<
        twist = Twist()
        twist.linear.x = 0.0  # Initialize forward motion to zero
        twist.angular.z = 0.0  # Initialize turning motion to zero

        # Define the interpolation factor
        t = 0.9  # Adjust this value to control the smoothness

        # Move forward if within centered target range
        if 330 <= target_x <= 510:
            twist.linear.x = self.lerp(twist.linear.x, self.base_driving_speed, t)
            twist.angular.z = self.lerp(twist.angular.z, 0.0, t)  # Stop turning
            self.get_logger().info(f"Moving forward with speed {twist.linear.x}")
        else:
            # Determine direction based on x-coordinate
            if target_x < 330:
                twist.angular.z = self.lerp(twist.angular.z, self.base_turning_speed, t)  # Turn left
                self.get_logger().info(f"Turning left with angular speed {twist.angular.z}")
            elif target_x > 510:
                twist.angular.z = self.lerp(twist.angular.z, -self.base_turning_speed, t)  # Turn right
                self.get_logger().info(f"Turning right with angular speed {twist.angular.z}")

        return twist

    def run(self):
        try:
            while rclpy.ok():
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

                    # Check if the bumper is pressed
                    if self.bumper.is_pressed:
                        self.get_logger().info("Bumper pressed! Pushing the ball for a short duration.")
                        
                        # Continue moving forward for a short duration to push the ball
                        push_duration = 0.4  # Duration in seconds
                        push_twist = Twist()
                        push_twist.linear.x = self.base_driving_speed
                        push_twist.angular.z = 0.0
                        self.cmd_vel_publisher.publish(push_twist)
                        
                        # Wait for the specified duration
                        time.sleep(push_duration)
                        
                        # Stop the robot after the delay
                        self.get_logger().info("Stopping the robot.")
                        stop_twist = Twist()  # Create a zeroed Twist message to stop the robot
                        self.cmd_vel_publisher.publish(stop_twist)
                        
                        # Wait for user input to continue
                        input("Press Enter to continue...")
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
