#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateRepublisher(Node):
    def __init__(self):
        super().__init__('joint_state_republisher')
        # Subscribe to the original topic
        self.sub = self.create_subscription(
            JointState,
            '/zx200/joint_state',
            self.joint_state_cb,
            10
        )
        # Publish to the renamed topic
        self.pub = self.create_publisher(
            JointState,
            '/zx200/joint_states',
            10
        )

    def joint_state_cb(self, msg: JointState):
        msg.header.frame_id = 'map'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
