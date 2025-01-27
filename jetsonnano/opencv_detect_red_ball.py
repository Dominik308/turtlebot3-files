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
        self.lower = np.array([0, 100, 20], np.uint8)
        self.upper = np.array([10, 255, 255], np.uint8)
        self.set_color(ball_color)

    def set_color(self, ball_color):
        if ball_color == 'red':
            # HSV range for red color
            self.lower = np.array([160, 100, 20], np.uint8)
            self.upper = np.array([179, 255, 255], np.uint8)
    
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
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if radius > 10:
                # Draw a green rectangle around the ball
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return center

# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Host IP and port number
host_ip = '0.0.0.0'  # Use your actual server's IP
port = 9999

# Bind and listen for incoming connections
server_socket.bind((host_ip, port))
server_socket.listen(5)
print("Listening at", (host_ip, port))

# Accept a connection
client_socket, addr = server_socket.accept()
print('Connection from:', addr)

# Initialize the CSI Camera (specific to Jetson Nano)
cam = CSICamera(width=840, height=560, capture_width=840, capture_height=560, capture_fps=15)

# Initialize the ball detector for detecting a red ball
detector = BallDetector('red')

try:
    # Continue to send video frames
    while True:
        # Read frame from the Jetson Nano camera
        img = cam.read()
        if img is None:
            print("Failed to capture frame")
            break
        
        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Create a mask for the red color
        mask = cv2.inRange(hsv, detector.lower, detector.upper)
        
        # Apply the mask to keep only red pixels
        red_only = cv2.bitwise_and(img, img, mask=mask)
        
        # Detect the ball in the frame
        center = detector.detect(red_only)
        
        # Optional: Print the ball center coordinates (for debugging)
        if center is not None:
            print(f"Ball center at: {center}")
        
        # Compress the frame using JPEG to reduce data size
        ret, buffer = cv2.imencode('.jpg', red_only)
        
        if not ret:
            print("Error: Failed to encode frame")
            continue  # Skip this frame if encoding failed
        
        # Serialize the frame (convert to byte stream)
        frame_data = pickle.dumps(buffer)
        
        # Pack the frame size (Q: unsigned long long -> 8 bytes)
        frame_size = struct.pack("Q", len(frame_data))
        
        # Send the packed frame size followed by the frame data
        try:
            client_socket.sendall(frame_size + frame_data)
        except ConnectionResetError:
            print("Connection reset by peer")
            break
finally:
    # Clean up
    cam.release()
    client_socket.close()
    server_socket.close()
