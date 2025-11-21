#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math
from tf2_ros import TransformBroadcaster
import tf_transformations
from geometry_msgs.msg import Quaternion
import time

class PoseToOdomNode(Node):
    def __init__(self):
        super().__init__('pose_to_odom')

        odom_header_frame_param = self.declare_parameter('odom_header_frame', 'world')
        odom_child_frame_param = self.declare_parameter('odom_child_frame', '/zx200/gnss/base_link')
        poseStamped_topic_name_param = self.declare_parameter('poseStamped_topic_name', '/zx200/global_pose')
        odom_topic_name_param = self.declare_parameter('odom_topic_name', '/zx200/gnss_odom')

        self.odom_header_frame = odom_header_frame_param.get_parameter_value().string_value
        self.odom_child_frame = odom_child_frame_param.get_parameter_value().string_value
        self.poseStamped_topic_name = poseStamped_topic_name_param.get_parameter_value().string_value
        self.odom_topic_name = odom_topic_name_param.get_parameter_value().string_value

        self.subscription = self.create_subscription(
            PoseStamped,
            self.poseStamped_topic_name, # 実機実験でtopic名が合っているか確認
            self.pose_cb,
            10)
        self.publisher = self.create_publisher(
            Odometry,
            self.odom_topic_name, # 実機実験でtopic名が合っているか確認
            10)
        self.is_sub = False
        self.pose = PoseStamped()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.z_prev = 0.0
        self.counter = 0
        self.dt = 1.0 / 50.0 

    def pose_cb(self, data):
        self.pose = data
        self.is_sub = True

    def quaternion_to_euler_angle(self, quaternion):
        q = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
        euler = tf_transformations.euler_from_quaternion(q)
        x = math.degrees(euler[0])
        y = math.degrees(euler[1])
        z = math.degrees(euler[2])
        return x, y, z

    def run(self):
        while rclpy.ok():
            if self.is_sub:
                (v_roll, v_pitch, v_yaw) = self.quaternion_to_euler_angle(self.pose.pose.orientation)

                x = self.pose.pose.position.x
                y = self.pose.pose.position.y
                z = self.pose.pose.position.z

                yaw = math.radians(v_yaw)

                if self.counter > 0:
                    vel_x_world = (x - self.x_prev) / self.dt
                    vel_y_world = (y - self.y_prev) / self.dt

                    self.x_prev = x
                    self.y_prev = y

                    twist_x = math.cos(yaw) * vel_x_world + math.sin(yaw) * vel_y_world
                    twist_y = math.cos(yaw) * vel_y_world - math.sin(yaw) * vel_x_world

                    odom = Odometry()
                    odom.header.frame_id = self.odom_header_frame # 実機実験でframe名が合っているか確認
                    odom.child_frame_id = self.odom_child_frame # 実機実験でframe名が合っているか確認
                    # odom.header.stamp = self.get_clock().now().to_msg()
                    odom.header.stamp = self.pose.header.stamp

                    odom.pose.pose.position.x = self.pose.pose.position.x
                    odom.pose.pose.position.y = self.pose.pose.position.y
                    odom.pose.pose.position.z = self.pose.pose.position.z

                    odom.pose.pose.orientation.x = self.pose.pose.orientation.x
                    odom.pose.pose.orientation.y = self.pose.pose.orientation.y
                    odom.pose.pose.orientation.z = self.pose.pose.orientation.z
                    odom.pose.pose.orientation.w = self.pose.pose.orientation.w

                    odom.twist.twist.linear.x = twist_x
                    odom.twist.twist.linear.y = twist_y
                    odom.twist.twist.linear.z = (z - self.z_prev) / self.dt
                    self.z_prev = z

                    odom.twist.twist.angular.x = 0.0
                    odom.twist.twist.angular.y = 0.0
                    odom.twist.twist.angular.z = 0.0

                    if self.is_sub == True:
                        self.publisher.publish(odom)
                        self.is_sub = False

                else:
                    self.x_prev = x
                    self.y_prev = y
                    self.z_prev = z
                    self.counter += 1

            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    pose_to_odom_node = PoseToOdomNode()
    pose_to_odom_node.run()

if __name__ == '__main__':
    main()
