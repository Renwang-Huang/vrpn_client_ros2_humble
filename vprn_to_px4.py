#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3Stamped

class TwistToVector3(Node):
    def __init__(self):
        super().__init__('twist_to_vector3')
        
        self.pub = self.create_publisher(Vector3Stamped, '/mavros/vision_speed/speed_vector', 10)
        
        self.sub = self.create_subscription(
            TwistStamped,
            '/vrpn/realsense_px/twist',
            self.twist_callback,
            10
        )
        # self.get_logger().info('Twist to Vector3 bridge node started.')
        self.get_logger().info("\033[1;32m[VRPN->MAVROS] Twist to Vector3 bridge node started.\033[0m")

    def twist_callback(self, msg: TwistStamped):
        """收到 TwistStamped 消息，将 linear 速度转发到 Vector3Stamped"""
        vector_msg = Vector3Stamped()
        # 使用 ROS 系统时间戳
        vector_msg.header.stamp = self.get_clock().now().to_msg()
        vector_msg.header.frame_id = msg.header.frame_id

        # 只取 linear 速度
        vector_msg.vector.x = msg.twist.linear.x
        vector_msg.vector.y = msg.twist.linear.y
        vector_msg.vector.z = msg.twist.linear.z

        self.pub.publish(vector_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToVector3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
