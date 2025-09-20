#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class ScanTFChecker(Node):
    def __init__(self):
        super().__init__('scan_tf_checker')

        # 创建 TF Buffer 和 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅 /scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info("ScanTFChecker started. Listening to /scan...")

        # slam_toolbox 使用的 frame 名
        self.map_frame = 'map'
        self.odom_frame = 'odom'
        self.base_frame = 'base_link'

    def scan_callback(self, msg: LaserScan):
        stamp = msg.header.stamp
        scan_frame = msg.header.frame_id

        # 检查 map -> odom
        try:
            self.tf_buffer.lookup_transform(self.map_frame, self.odom_frame, stamp)
            map_odom_ok = True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            map_odom_ok = False
            map_odom_err = str(e)

        # 检查 odom -> base_link
        try:
            self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, stamp)
            odom_base_ok = True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            odom_base_ok = False
            odom_base_err = str(e)

        # 检查 scan_frame -> base_link
        try:
            self.tf_buffer.lookup_transform(self.base_frame, scan_frame, stamp)
            scan_base_ok = True
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            scan_base_ok = False
            scan_base_err = str(e)

        # 输出结果
        if map_odom_ok and odom_base_ok and scan_base_ok:
            self.get_logger().info(
                f"OK: All transforms available for scan at time {stamp.sec}.{stamp.nanosec}"
            )
        else:
            self.get_logger().warn(
                f"FAILED for scan at time {stamp.sec}.{stamp.nanosec}:"
                f"{'' if map_odom_ok else f' map->odom missing ({map_odom_err});'}"
                f"{'' if odom_base_ok else f' odom->base_link missing ({odom_base_err});'}"
                f"{'' if scan_base_ok else f' scan_frame->{self.base_frame} missing ({scan_base_err})'}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ScanTFChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
