#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FrontJointStatesFilter(Node):
    """
    /joint_states を購読し，
    name 配列中の swing_joint ～ bucket_joint までの要素だけを残した
    JointState を /front_joint_states で配信するノード
    """

    def __init__(self):
        super().__init__('front_joint_states_filter')

        # QoS はとりあえずデフォルトの depth=10
        qos = rclpy.qos.QoSProfile(depth=10)

        self._sub = self.create_subscription(
            JointState,
            '/zx200/joint_states',
            self._joint_states_callback,
            qos
        )

        self._pub = self.create_publisher(
            JointState,
            '/zx200/front_joint_states',
            qos
        )

        self.get_logger().info('front_joint_states_filter node started.')

    def _joint_states_callback(self, msg: JointState):
        names = list(msg.name)

        # swing_joint と bucket_joint の index を探す
        try:
            start_idx = names.index('swing_joint')
            end_idx = names.index('bucket_joint')
        except ValueError:
            # どちらかが入っていないメッセージは無視
            return

        # 念のため順番が逆でも動くように
        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx

        indices = list(range(start_idx, end_idx + 1))

        # 新しい JointState メッセージを作成
        out = JointState()
        # header はそのままコピー（stamp も元メッセージと揃える）
        out.header = msg.header

        # name / position / velocity / effort を index でフィルタ
        out.name = [msg.name[i] for i in indices]
        out.name.append('bucket_end_joint')

        if msg.position:
            out.position = [msg.position[i] for i in indices]
            out.position.append(0)
        if msg.velocity:
            out.velocity = [msg.velocity[i] for i in indices]
            out.velocity.append(0)
        if msg.effort:
            out.effort = [msg.effort[i] for i in indices]
            out.effort.append(0)
            
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FrontJointStatesFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
