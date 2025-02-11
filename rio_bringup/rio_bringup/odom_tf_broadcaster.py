#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster


class TFBroadcasterNode(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster_node')

        # Create a TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the tf_odom topic
        self.subscription = self.create_subscription(
            TransformStamped,
            'tf_odom_geometry_msgs',
            self.tf_callback,
            10)

        self.get_logger().info('TF Broadcaster Node has been started')

    def tf_callback(self, msg):
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
