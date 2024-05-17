import socket
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DescriptionPublisher(Node):
    def __init__(self):
        super().__init__('description_publisher')
        self.publisher_ = self.create_publisher(String, '/description', 10)
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_description)
        self.description = None

    def set_description(self, s):
        self.description = s

    def publish_description(self):
        if self.description is not None:
            msg = String()
            msg.data = self.description
            self.publisher_.publish(msg)

def main():
    rclpy.init()
    description_publisher = DescriptionPublisher()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(description_publisher)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("0.0.0.0", 12346))
    sock.listen(10)

    try:
        while rclpy.ok():
            conn, addr = sock.accept()
            try:
                data = conn.recv(struct.calcsize(">L"))
                if len(data) != 4:
                    print("Not enough data to unpack")
                else:
                    length = struct.unpack(">L", data)[0]
                    received_bytes = b""
                    while len(received_bytes) < length:
                        to_read = length - len(received_bytes)
                        received_bytes += conn.recv(4096 if to_read > 4096 else to_read)

                    string = received_bytes.decode()
                    print("Received string:", string)
                    description_publisher.set_description(string)
            except Exception as e:
                print(f"Error receiving or processing data: {e}")
            finally:
                if conn:
                    conn.close()

            rclpy.spin_once(description_publisher, timeout_sec=0.1)
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        sock.close()
        description_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
