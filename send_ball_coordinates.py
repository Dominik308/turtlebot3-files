import cv2
import socket
import struct
import pickle
from jetcam.csi_camera import CSICamera
import numpy as np

# BallDetector class for detecting a colored ball (red in this case)
class BallDetector:
    def __init__(self, ball_color):
        self.ball_color = ball_color
        self.lower = np.array([160, 100, 20], np.uint8)  # HSV lower range for red
        self.upper = np.array([179, 255, 255], np.uint8)  # HSV upper range for red

    def detect(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.erode(mask, None, iterations=5)
        mask = cv2.dilate(mask, None, iterations=5)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            if M["m00"] != 0:  # Check to avoid division by zero
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if radius > 10:  # Minimum radius to consider
                    # Draw a green rectangle around the ball
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return center

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    server_socket.connect(("10.1.1.0", 9999))
except socket.error as e:
    print(f"Socket error: {e}")
    exit(1)

# Initialize the CSI Camera (specific to Jetson Nano)
cam = CSICamera(width=840, height=560, capture_width=840, capture_height=560, capture_fps=10)

# Initialize the ball detector for detecting a red ball
detector = BallDetector('red')

try:
    # Continue to send video frames
    while True:
        # Read frame from the Jetson Nano camera
        img = cam.read()
        if img is None:
            print("Failed to capture frame")
            continue
        
        # Detect the ball in the frame
        center = detector.detect(img)
        
        # Optional: Print the ball center coordinates (for debugging)
        if center is not None:
            # Serialize the center coordinates for sending
            center_data = pickle.dumps(center)
            # Pack the frame size (Q: unsigned long long -> 8 bytes)
            frame_size = struct.pack("Q", len(center_data))

            # Send the packed frame size followed by the frame data
            try:
                server_socket.sendall(frame_size + center_data)
            except socket.error as e:
                print(f"Error sending data: {e}")
                break
        else:
            print("No balls detected.")
finally:
    # Clean up
    cam.cap.release()
    server_socket.close()
