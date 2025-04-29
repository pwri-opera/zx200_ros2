#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class FrameIdFixer(Node):
    def __init__(self):
        super().__init__('odom_frame_id_fixer')
        # 入力トピック
        self.sub = self.create_subscription(
            Odometry,
            '/zx200/odom',
            self.odom_cb,
            10
        )
        # 出力トピック（書き換え後）
        self.pub = self.create_publisher(
            Odometry,
            '/zx200/fixed_odom',
            10
        )

    def odom_cb(self, msg: Odometry):
        # frame_id を上書き
        msg.header.frame_id = 'odom'
        # 配信
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrameIdFixer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
