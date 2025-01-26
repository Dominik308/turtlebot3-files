import time  # Import the time module for time-related operations
import rclpy  # Import the ROS 2 Python client library
from rclpy.node import Node  # Import the Node class from the ROS 2 library
from geometry_msgs.msg import Twist  # Import the Twist message type from geometry_msgs
from std_msgs.msg import String, Bool  # Import the String and Bool message types from std_msgs
from rclpy.qos import QoSProfile  # Import QoSProfile for configuring the Quality of Service settings
import random # Import the random module for generating random numbers

"""
SelfDriving class implements a basic self-driving robot using ROS 2.
The robot uses sonar and bumper sensors to navigate and avoid obstacles.
"""
class SelfDriving(Node):
    # Define the various states the robot can be in
    FORWARD = 0  # The robot is moving forward
    STOP = 1  # The robot is stopped
    MEASURING = 2  # The robot is in the process of measuring its surroundings
    TURNING = 3  # The robot is turning to a new direction
    RETURNING = 4  # The robot is returning to its starting position
    
    def __init__(self):
        """
        Initialize the SelfDriving node, set up subscriptions, publishers, and the initial state.
        """
        super().__init__('self_driving')
        qos = QoSProfile(depth=1)  # Set up Quality of Service profile with a depth of 1
        
        # Initialize Twist object to control the robot's velocity
        self.twist = Twist()

        # Subscription to the SonarSensorInfo topic
        self.sonar_subscription = self.create_subscription(
            String,
            'SonarSensorInfo',
            self.distance_callback,
            qos
        )
        self.distance = None  # Initialize distance variable to None
        
        # Subscription to the BumperSensorInfo topic
        self.bumper_subscription = self.create_subscription(
            Bool,
            'BumperSensorInfo',
            self.bumper_callback,
            qos,
        )
        
        # Initialize other instance variables
        self.bumper_pressed = False
        self.state = SelfDriving.FORWARD
        self.turn_direction = 1
        self.measure_stage = 0
        self.right_distance = 0
        self.left_distance = 0
        self.last_turn_time = time.time()  # Record the time of the last turn
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 1
        )
        
        # Create a timer to periodically call the self_drive_callback method
        self.timer = self.create_timer(0.1, self.self_drive_callback)
        self.checking_start_time = None
        self.measure_count = 0  # Count consecutive measurements without moving forward

    def distance_callback(self, msg):
        """
        Callback function for SonarSensorInfo subscription.
        Updates the distance based on sensor data.
        """
        self.distance = float(msg.data)

    def bumper_callback(self, msg):
        """
        Callback function for BumperSensorInfo subscription.
        Updates the bumper_pressed status based on sensor data.
        """
        self.bumper_pressed = msg.data

    def handle_forward_state(self):
        """
        Handle the robot's behavior when it is in the FORWARD state.
        The robot moves forward unless an obstacle is detected.
        """
        if self.distance < 0.30 or self.bumper_pressed:
            self.state = SelfDriving.STOP  # Transition to STOP state if obstacle is detected
        else:
            self.twist.linear.x = 0.20  # Move forward with linear velocity
            self.twist.angular.z = 0.0  # No angular velocity
            self.measure_count = 0  # Reset the measure count when moving forward

    def handle_stop_state(self):
        """
        Handle the robot's behavior when it is in the STOP state.
        The robot stops and transitions to the MEASURING state.
        """
        print("Stopped robot")
        self.twist.linear.x = 0.0  # Stop linear motion
        self.twist.angular.z = 0.0  # Stop angular motion
        self.state = SelfDriving.MEASURING  # Transition to MEASURING state
        self.turn_direction = 1  # Start by turning left
        self.measure_stage = 1
        self.last_turn_time = time.time()  # Reset the timer
        self.measure_count += 1  # Increment measure count

    def handle_measuring_state(self):
        """
        Handle the robot's behavior when it is in the MEASURING state.
        The robot measures distances by turning and recording sensor readings.
        """
        if time.time() - self.last_turn_time < 1.5:  # Turn for 1.5 seconds
            self.twist.angular.z = 1.2 * self.turn_direction  # Maintain the turning speed
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
                print("Left distance: %.2f, Right distance: %.2f" % (self.left_distance, self.right_distance))
                # If both distances are equal, choose a random direction
                if self.left_distance == self.right_distance: 
                    self.turn_direction = random.choice([1, -1])
                else:
                    self.turn_direction = 1 if self.left_distance > self.right_distance else -1 
                
                if (self.turn_direction == -1):
                    print("Turning right")
                else:
                    print("Turning left")
                    
                self.last_turn_time = time.time()  # Reset the timer

        # Check if measuring state has been entered 3 times consecutively without moving forward
        if self.measure_count >= 3:
            self.state = SelfDriving.RETURNING  # Transition to RETURNING state
            self.turn_direction = 1  # Prepare to turn 180 degrees
            self.last_turn_time = time.time()  # Reset the timer

    def handle_turning_state(self):
        """
        Handle the robot's behavior when it is in the TURNING state.
        The robot turns to avoid obstacles.
        """
        if time.time() - self.last_turn_time < 1.5:  # Wait for 1.5 seconds
            self.twist.angular.z = 1.3 * self.turn_direction  # Turn in the chosen direction
        else:
            self.twist.angular.z = 0.0
            if self.distance >= 0.30:
                self.state = SelfDriving.FORWARD  # Move forward if there is no obstacle
            else:
                self.state = SelfDriving.STOP

    def handle_returning_state(self):
        """
        Handle the robot's behavior when it is in the RETURNING state.
        The robot performs a 180-degree turn if it gets stuck.
        """
        if time.time() - self.last_turn_time < 3:  # Perform 180-degree turn for 3 seconds
            self.twist.angular.z = 1.3  # Adjust angular speed as needed
            self.twist.linear.x = 0.0  # No linear motion
        else:
            self.twist.angular.z = 0.0  # Stop turning
            self.state = SelfDriving.FORWARD  # Return to FORWARD state
            self.measure_count = 0  # Reset measure count after returning
            
    def self_drive_callback(self):
        """
        Main callback function that handles the robot's state transitions and behavior.
        Called periodically by the timer.
        """
        if self.distance is None or self.bumper_pressed is None:
            return  # If sensor data is not available, do nothing

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

        # Publish the twist message to the cmd_vel topic to control the robot's movement
        self.cmd_vel_pub.publish(self.twist)

def main():
    """
    Main function to initialize the ROS 2 node and start the robot's operation.
    """
    rclpy.init()  # Initialize ROS 2 Python client library
    self_driving = SelfDriving()  # Create an instance of the SelfDriving class
    try:
        rclpy.spin(self_driving)  # Keep the node running until interrupted
    except KeyboardInterrupt:
        print("Interrupted by keyboard, cleaning up...")  # Handle keyboard interrupt
        self_driving.stop()
        self_driving.self.cmd_vel_pub.publish(self_driving.self.twist)  # Stop the robot's motion
    finally:
        self_driving.destroy_node()  # Destroy the node before shutting down
        rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()  # Entry point of the script
