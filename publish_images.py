import cv2
import numpy as np
import socket
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'detectedImage/image', 10)
        self.bridge = CvBridge()

    def publish_image(self, cv_image):
        msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    image_publisher = ImagePublisher()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("0.0.0.0", 12345))
    sock.listen(10)
    counter = 1
    conn = None
    try:
        while rclpy.ok():
            conn, addr = sock.accept()
            try:    
                data = conn.recv(struct.calcsize(">L"))

                if len(data) != 4:
                    print("Not enough data to unpack")
                else:
                    length = struct.unpack(">L", data)[0]

                    image_bytes = b""
                    while len(image_bytes) < length:
                        to_read = length - len(image_bytes)
                        image_bytes += conn.recv(4096 if to_read > 4096 else to_read)

                    image_array = np.frombuffer(image_bytes, dtype=np.uint8)

                    print("Received Image: " + str(counter) + " from: " + str(addr))
                    counter = counter + 1
                    width = 1280
                    height = 736

                    image_array = np.reshape(image_array, (height, width, 3))
                    image_publisher.publish_image(image_array)
            except Exception as e:
                print(f"Error receiving or processing image: {e}")
                if conn:
                    conn.close()
    except Exception as e:
        print(f"Error in main loop: {e}")

if __name__ == '__main__':
    main()
