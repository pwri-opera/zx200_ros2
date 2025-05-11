#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

class MapToBaselinkTfBroadcaster(Node):
    def __init__(self):
        super().__init__('map_to_baselink_tf_broadcaster')
        self.declare_parameter('map_to_baselink_topic', 'gnss_odom')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')

        odom_topic = self.get_parameter('map_to_baselink_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.get_logger().info(f'Subscribed to "{odom_topic}", broadcasting TF "{self.frame_id}" -> "{self.child_frame_id}"')

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        t.transform.translation.x = msg.pose.pose.position.x - 21395.178
        t.transform.translation.y = msg.pose.pose.position.y - 14034.450
        t.transform.translation.z = msg.pose.pose.position.z - 28.552
        t.transform.rotation = msg.pose.pose.orientation

        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapToBaselinkTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
