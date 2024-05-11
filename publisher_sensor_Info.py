
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool

from gpiozero import DistanceSensor, Button

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('SensorPublisher')
        self.sensor = DistanceSensor(9, 10)
        self.bumper = Button(4, pull_up=False)
        timer_period = 0.2  # seconds

        self.sonar_publisher = self.create_publisher(String, 'SonarSensorInfo', 10)
        self.sonar_timer = self.create_timer(timer_period, self.publish_sonar)
        self.sonarCount = 0

        self.bumper_publisher = self.create_publisher(Bool, 'BumperSensorInfo', 10)
        self.bumper_timer = self.create_timer(timer_period, self.publish_bumper)
        self.bumbperCount = 0


    def __del__(self):
        # Clean up the sensor
        self.sensor.close()
        self.bumper.close()

    def publish_sonar(self):
        msg = String()
        msg.data = "%s" % self.sensor.distance
        self.sonar_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.sonarCount += 1

    def publish_bumper(self):
        msg = Bool()
        msg.data = self.bumper.is_pressed
        self.bumper_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.bumbperCount += 1

def main(args=None):
    
    rclpy.init(args=args)
    sensorPublisher = SensorPublisher()

    try:
        rclpy.spin(sensorPublisher)
    except KeyboardInterrupt:
        # Handle the keyboard interrupt by printing a message
        print("Interrupted by keyboard, cleaning up GPIOs...")
    finally:
        # Clean up the sensors
        sensorPublisher.sensor.close()
        sensorPublisher.bumper.close()
        # Destroy the node explicitly
        sensorPublisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
