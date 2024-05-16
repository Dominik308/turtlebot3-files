import cv2
import numpy as np
import socket
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/detectedImage/Image', 10)
        self.bridge = CvBridge()
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_image)
        self.cv_image = None

    def set_image(self, cv_image):
        self.cv_image = cv_image

    def publish_image(self):
        if self.cv_image is not None:
            msg = self.bridge.cv2_to_imgmsg(self.cv_image, 'bgr8')
            self.publisher_.publish(msg)
        print("Published Image---------------------")

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
                    
                    # Save the image to a file
                    #filename = "received_image.jpg"
                    #cv2.imwrite(filename, image_array)
                    
                    image_array = cv2.resize(image_array, (width // 2, height // 2))
                                       
                    #image_publisher.publish_image(image_array)
                    
                    image_publisher.set_image(image_array)
                    rclpy.spin_once(image_publisher)
                    print("Finished Spinning")
                    
            except Exception as e:
                print(f"Error receiving or processing image: {e}")
                if conn:
                    conn.close()
    except Exception as e:
        print(f"Error in main loop: {e}")

if __name__ == '__main__':
    main()
