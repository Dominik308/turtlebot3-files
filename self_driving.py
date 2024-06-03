import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile

class SelfDriving(Node):

    FORWARD = 0
    STOP = 1
    MEASURING = 2
    TURNING = 3
    RETURNING = 4

    def __init__(self):
        super().__init__('self_driving')
        qos = QoSProfile(depth=1)
        
        # Initialize Twist object
        self.twist = Twist()

        self.subscription = self.create_subscription(
            String,
            'SonarSensorInfo',
            self.distance_callback,
            qos
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
        self.turn_direction = 1
        self.measure_stage = 0
        self.right_distance = 0
        self.left_distance = 0
        self.last_turn_time = time.time()  # Time of the last turn
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 1
        )
        
        self.timer = self.create_timer(0.1, self.self_drive_callback)
        self.checking_start_time = None
        self.measure_count = 0  # Count consecutive measurements without moving forward

    def distance_callback(self, msg):
        self.distance = float(msg.data)

    def bumper_callback(self, msg):
        self.bumper_pressed = msg.data

    def handle_forward_state(self):
        if self.distance < 0.30 or self.bumper_pressed:
            self.state = SelfDriving.STOP
        else:
            self.twist.linear.x = 0.20
            self.twist.angular.z = 0.0
            self.measure_count = 0  # Reset the measure count when moving forward

    def handle_stop_state(self):
        print("Stopped robot")
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.state = SelfDriving.MEASURING
        self.turn_direction = 1  # Start by turning left
        self.measure_stage = 1
        self.last_turn_time = time.time()  # Reset the timer
        self.measure_count += 1  # Increment measure count

    def handle_measuring_state(self):
        if time.time() - self.last_turn_time < 1.5:  # Wait for 1.5 seconds
            self.twist.angular.z = 1.3 * self.turn_direction  # Maintain the turning speed
        else:
            self.twist.angular.z = 0.0  # Stop turning
            if self.measure_stage == 1:  # First turn (left)
                self.left_distance = self.distance
                self.measure_stage = 2  # Next, return to original position
                self.turn_direction = -1  # Turn right to return to original position
                self.last_turn_time = time.time()  # Reset the timer
            elif self.measure_stage == 2:  # Returning to original position
                self.measure_stage = 3  # Next, turn right
                self.turn_direction = -1  # Continue turning right
                self.last_turn_time = time.time()  # Reset the timer
            elif self.measure_stage == 3:  # Second turn (right)
                self.right_distance = self.distance
                self.measure_stage = 4  # Next, return to original position again
                self.turn_direction = 1  # Turn left to return to original position
                self.last_turn_time = time.time()  # Reset the timer
            elif self.measure_stage == 4:  # Returned to original position
                self.state = SelfDriving.TURNING  # Finished measuring
                self.turn_direction = 1 if self.left_distance >= self.right_distance else -1
                if (self.turn_direction == -1):
                    print("Turning right")
                else:
                    print("Turning left")

        # Check if measuring state has been entered 3 times consecutively without moving forward
        if self.measure_count >= 3:
            self.state = SelfDriving.RETURNING
            self.turn_direction = 1  # Prepare to turn 180 degrees
            self.last_turn_time = time.time()  # Reset the timer

    def handle_turning_state(self):
        self.twist.angular.z = 1.3 * self.turn_direction  # Turn in the chosen direction
        if self.distance >= 0.30:
            self.state = SelfDriving.FORWARD  # Move forward if there is no obstacle

    def handle_returning_state(self):
        if time.time() - self.last_turn_time < 3:  # Perform 180-degree turn
            self.twist.angular.z = 1.3  # Adjust angular speed as needed
            self.twist.linear.x = 0.0
        else:
            self.twist.angular.z = 0.0
            self.state = SelfDriving.FORWARD
            self.measure_count = 0  # Reset measure count after returning
            
    def self_drive_callback(self):
        if self.distance is None or self.bumper_pressed is None:
            return

        if self.state == SelfDriving.FORWARD:
            self.handle_forward_state()
            print("Moving forward")

        elif self.state == SelfDriving.STOP:
            self.handle_stop_state()
        elif self.state == SelfDriving.MEASURING:
            self.handle_measuring_state()
        elif self.state == SelfDriving.TURNING:
            self.handle_turning_state()
        elif self.state == SelfDriving.RETURNING:
            self.handle_returning_state()
            print("Got stuck, returning")


        # Publish the twist message
        self.cmd_vel_pub.publish(self.twist)

def main():
    rclpy.init()
    self_driving = SelfDriving()
    try:
        rclpy.spin(self_driving)
    except KeyboardInterrupt:
        print("Interrupted by keyboard, cleaning up...")
        self_driving.stop()
        self_driving.self.cmd_vel_pub.publish(self_driving.self.twist)
    finally:
        self_driving.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
