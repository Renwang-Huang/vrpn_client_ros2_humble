#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VrpnToMavrosPose(Node):

    def __init__(self):
        super().__init__('vrpn_to_mavros_pose')

        # Subscriber: VRPN pose
        self.sub = self.create_subscription(
            PoseStamped,
            '/vrpn/realsense_px/pose',
            self.pose_callback,
            10
        )

        # Publisher: MAVROS vision pose
        self.pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )

        self.get_logger().info("\033[1;32m[VRPN->MAVROS] Vision pose relay node started.\033[0m")

    def pose_callback(self, msg: PoseStamped):
        out_msg = PoseStamped()

        out_msg.header.stamp = self.get_clock().now().to_msg()
        out_msg.header.frame_id = msg.header.frame_id

        out_msg.pose.position = msg.pose.position
        out_msg.pose.orientation = msg.pose.orientation

        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)

    node = VrpnToMavrosPose()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
