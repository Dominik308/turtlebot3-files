import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped, Point, Twist
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import numpy as np
from stable_baselines3 import PPO
import math
from rclpy.time import Time

class StableBaselines3Inference(Node):
    FACTOR = 1.2
    TWIST_PUB_FREQ = 0.1
    TWIST_ANG_RATE = 2.5 * TWIST_PUB_FREQ
    TWIST_VEL_RATE = 0.5 * TWIST_PUB_FREQ

    def __init__(self):
        super().__init__('stable_baselines3_inference')
        self.get_logger().info("Setting up inference node...")

        self.qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.ball_angle_subscription = self.create_subscription(
            Float32,
            '/ball_angle',
            self.ball_angle_callback,
            10,
        )

        self.sonar_subscription = self.create_subscription(
            Float32,
            'SonarSensorInfo',
            self.sonar_distance_callback,
            self.qos_profile
        )
        self.sonar_distance = None

        self.ball_angle = 0.0
        self.ball_visible = 0
        self.ball_pos = PointStamped()
        self.ball_pos_pub = self.create_publisher(PointStamped, "/ball_pos", 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.model = PPO.load("/home/turtle1/turtlebot3_ws/ball_locator_8999424_steps.zip")

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.twist_vel = 0.0
        self.vel_target = 0.0
        self.twist_ang = 0.0
        self.ang_target = 0.0

        self.create_timer(0.26, self.inference_tick)
        self.create_timer(self.TWIST_PUB_FREQ, self.twist_pub_loop)

        self.ball_dir_vec = (0.0, 0.0)
        self.get_logger().info("Inference node setup finished.")

    def sonar_distance_callback(self, msg):
        self.sonar_distance = msg.data

    def ball_angle_callback(self, msg):
        self.ball_angle = msg.data
        if math.isnan(self.ball_angle):
            self.ball_angle = 0.0
            self.ball_visible = 0
            self.ball_dir_vec = (0.0, 0.0)
        else:
            self.ball_visible = 1
            angle_deg = self.ball_angle * 60.0 
            rad = math.radians(-angle_deg) 
            cos_r = math.cos(rad)
            sin_r = math.sin(rad)
            def normalized(x, y):
                magnitude = math.hypot(x, y)
                if magnitude == 0:
                    return (0.0, 0.0)
                return (x / magnitude, y / magnitude)
            self.ball_dir_vec = normalized(cos_r, sin_r)

            if self.sonar_distance is not None:
                try:
                    # Request the latest transform by using a zero timestamp.
                    map_tf = self.tf_buffer.lookup_transform("map", "base_link", Time(seconds=0).to_msg())
                except Exception as e:
                    self.get_logger().error(f"Transform error: {e}")
                    return
                dir_stamped = PointStamped()
                dir_stamped.header.frame_id = "base_link"
                dir_stamped.header.stamp = self.get_clock().now().to_msg()
                dir_stamped.point = Point()
                # Use sonar distance plus an offset as the forward measurement.
                dir_stamped.point.x = self.sonar_distance + 0.09
                dir_stamped.point.y = 0.0
                dir_stamped.point.z = 0.0
                self.ball_pos = do_transform_point(dir_stamped, map_tf)
                self.ball_pos_pub.publish(self.ball_pos)

    def move_toward(self, src, tgt, step):
        if src < tgt:
            return min(src + step, tgt)
        else:
            return max(src - step, tgt)

    def twist_pub_loop(self):
        self.twist_vel = self.move_toward(self.twist_vel, self.vel_target, self.TWIST_VEL_RATE)
        self.twist_ang = self.move_toward(self.twist_ang, self.ang_target, self.TWIST_ANG_RATE)
        twist = Twist()
        twist.linear.x = self.twist_vel
        twist.angular.z = self.twist_ang
        self.twist_pub.publish(twist)

    def inference_tick(self):
        try:
            # Request the latest transform using a zero time stamp.
            tf_to_local = self.tf_buffer.lookup_transform("base_link", "map", Time(seconds=0).to_msg())
        except Exception as e:
            self.get_logger().error(f"Transform error: {e}")
            return

        ball_pos_rel = do_transform_point(self.ball_pos, tf_to_local)
        self.get_logger().debug(f"TF ball position: x={ball_pos_rel.point.x}, y={ball_pos_rel.point.y}")

        obs = {
            "obs": [
                self.ball_dir_vec[0],
                self.ball_dir_vec[1],
                ball_pos_rel.point.x / self.FACTOR,
                ball_pos_rel.point.y / self.FACTOR,
            ]
        }

        action, _states = self.model.predict(obs)
        obsv = obs["obs"]
        self.get_logger().info(
            f"\nObs: ({obsv[0]:.4f} | {obsv[1]:.4f} | {obsv[2]:.4f} | {obsv[3]:.4f}) \nAction: ({action[0]:.4f} | {action[1]:.4f})"
        )

        self.vel_target = -np.clip(action[0] * 0.20, -0.10, 0.20)
        self.ang_target = -action[1] * 2.0

def main(args=None):
    rclpy.init(args=args)
    inference = StableBaselines3Inference()
    rclpy.spin(inference)
    inference.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
