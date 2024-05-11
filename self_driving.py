import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile
import random

class SelfDriving(Node):

    FORWARD = 0
    STOP = 1
    MEASURING = 2
    TURNING = 3
    CHECKING = 4
    CALCULATING = 5

    def __init__(self):
        super().__init__('SelfDriving')
        qos = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            String,
            'SonarSensorInfo',
            self.distance_callback,
            qos,
        )
        self.distance = None
        self.bumper_subscription = self.create_subscription(
            Bool,
            'BumperSensorInfo',
            self.bumper_callback,
            qos,
        )
        self.bumper_pressed = False
        self.state = SelfDriving.FORWARD
        self.turn_direction = 1  # Start by turning right
        self.measure_stage = 0  # 0: 45 degrees, 1: 90 degrees
        self.right_distance = [0, 0]  # Distances at 45 and 90 degrees
        self.left_distance = [0, 0]  # Distances at 45 and 90 degrees
        self.last_turn_time = time.time()  # Time of the last turn
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 10
        )
        self.timer = self.create_timer(0.2, self.self_drive_callback)
        self.checking_start_time = None

    def distance_callback(self, msg):
        self.distance = float(msg.data)

    def bumper_callback(self, msg):
        self.bumper_pressed = msg.data

    def self_drive_callback(self):
        if self.distance is None or self.bumper_pressed is None:
            return

        twist = Twist()

        if self.state == SelfDriving.FORWARD:
            if self.distance <= 0.30 or self.bumper_pressed:
                self.state = SelfDriving.STOP
            else:
                twist.linear.x = 0.15
                twist.angular.z = 0.0
        elif self.state == SelfDriving.STOP:
            print("Stopped robot")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.state = SelfDriving.MEASURING
            self.turn_direction = 1  # Start by turning left
            self.measure_stage = 1 
            #trigger nano detect.py here
        elif self.state == SelfDriving.MEASURING:
            print("MEASURING")
            if time.time() - self.last_turn_time < 1.1:  # Wait for 1.05 seconds
                twist.angular.z = 1.2 * self.turn_direction  # Maintain the turning speed
            else:
                twist.angular.z = 0.0  # Stop turning
                if self.measure_stage == 1: #Turning left
                    self.right_distance[0] = self.distance if self.turn_direction == 1 else 0
                    self.left_distance[0] = self.distance if self.turn_direction == -1 else 0
                    self.measure_stage = 2  # Next, turn right
                    self.last_turn_time = time.time()  # Reset the timer
                elif self.measure_stage == 2:  # Turning right
                    self.right_distance[1] = self.distance if self.turn_direction == 1 else 0
                    self.left_distance[1] = self.distance if self.turn_direction == -1 else 0
                    if self.turn_direction == 1:
                        self.turn_direction = -1
                        self.measure_stage = 1
                        self.last_turn_time = time.time()  # Reset the timer
                    else:
                        self.state = SelfDriving.TURNING  # Finished measuring
                        self.turn_direction = 1 if sum(self.left_distance) >= sum(self.right_distance) else -1
        elif self.state == SelfDriving.CALCULATING:
            print("MEASURING")
            if time.time() - self.last_turn_time < 5:  # Wait for one second
                self.state = SelfDriving.MEASURING
                            
        elif self.state == SelfDriving.TURNING:
            twist.angular.z = 1.2 * self.turn_direction  # Turn in the chosen direction
            if self.distance > 0.30:
                self.state = SelfDriving.FORWARD
                print("Going forward")

        for i in range(2):
            print(f"Right distance {i}: {self.right_distance[i]}")
            print(f"Left distance {i}: {self.left_distance[i]}")

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    selfDriving = SelfDriving()
    try:
        rclpy.spin(selfDriving)
    except KeyboardInterrupt:
        print("Interrupted by keyboard, cleaning up GPIOs...")
    finally:
        selfDriving.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
