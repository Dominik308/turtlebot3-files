import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Point
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import do_transform_point
import numpy as np
import tf2_ros
from stable_baselines3 import PPO

class StableBaselines3Inference(Node):
    def __init__(self):
        super().__init__('stable_baselines3_inference')
        
        self.ball_pos_subscription = self.create_subscription(
            PointStamped,
            '/ball_position_base',
            self.ball_pos_callback,
            10
        )
        self.ball_pos_buffer = []
        self.ball_pos: PointStamped | None = None
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.model = PPO.load("./pusher_lidar_final")
        
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        
        # timer for inference tick
        self.create_timer(0.1, self.inference_tick)
    
    def ball_pos_callback(self, msg):
        # Get the latest ball position
        self.ball_pos_buffer.append(msg.point)
        if len(self.ball_pos_buffer) > 5:
            self.ball_pos_buffer.pop(0)
        self.ball_pos = msg
        # self.get_logger().info(f"Ball position: {msg.point}")
        # calc mean ball velocity
        # TODO: implement ball velocity
    
    def inference_tick(self):
        if self.ball_pos is None:
            self.get_logger().warning("No ball position received yet")
        tb_pos = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        point_target = PointStamped()
        point_target.header.frame_id = "base_link"
        point_target.header.stamp = self.get_clock().now().to_msg()
        point_target.point = Point()
        point_target.point.x = 0.0
        point_target.point.y = 0.0
        point_target = do_transform_point(point_target, tb_pos)
        ball_pos = self.ball_pos.point
        obs = {
            "obs": [
                # turtlebot pos
                - tb_pos.transform.translation.y,
                - tb_pos.transform.translation.x, 
                # target area pos
                - point_target.point.y,
                - point_target.point.x, 
                # ball pos
                - ball_pos.y,
                - ball_pos.x, 
                # ball velocity
                0.0, 
                0.0,
            ]
        }
        self.get_logger().info(f"Observation: ({obs['obs'][2]}|{obs['obs'][3]})")
        action, _states = self.model.predict(obs)
        # self.get_logger().info(f"Action: {action}")
        twist = Twist()
        
        twist.linear.x = - action[0] * 0.10
        twist.angular.z = action[1] * 2.0
        
        self.twist_pub.publish(twist)
        pass
    

def main(args=None):
    rclpy.init(args=args)
    inference = StableBaselines3Inference()
    rclpy.spin(inference)
    inference.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
