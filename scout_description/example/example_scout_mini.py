#!/usr/bin/env python3
# Copyright 2024 Your Name
# Apache-2.0

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading

class DiffDriveTestNode(Node):
    def __init__(self):
        super().__init__('diff_drive_test_node_py')
        self.pub = self.create_publisher(
            Twist,
            '/scout_mini_velocity_controller/cmd_vel_unstamped',
            10
        )

        # 50 Hz 定时发布
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('Python diff_drive_test_node started')

    def timer_callback(self):
        cmd = Twist()
        cmd.linear.x = 0.1
        cmd.angular.z = 0.1
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = DiffDriveTestNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
