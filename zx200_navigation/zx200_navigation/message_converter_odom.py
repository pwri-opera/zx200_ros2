#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class FrameIdMapper(Node):
    def __init__(self):
        super().__init__('frame_id_mapper')
        self.declare_parameter('input_topic', "odom_pose")
        self.declare_parameter('output_topic', "fixed_odom_pose")

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic= self.get_parameter('output_topic').get_parameter_value().string_value
    

        self.subscription = self.create_subscription(
            Odometry,
            self.input_topic,
            self.callback,
            10
        )
        self.publisher = self.create_publisher(
            Odometry,
            self.output_topic,
            10
        )
        self.get_logger().info(
            f'Subscribing to "{self.input_topic}", publishing mapped messages to "{self.output_topic}"'
        )

    def callback(self, msg: Odometry):
        # frame_id を "map" に変更
        msg.header.frame_id = 'odom'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrameIdMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
