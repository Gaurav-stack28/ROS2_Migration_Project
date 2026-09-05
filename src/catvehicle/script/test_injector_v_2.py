#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


class TestInjector(Node):

    def __init__(self):
        super().__init__('test_publisher')

        self.pub = self.create_publisher(
            PointCloud2,
            '/test_point_cloud',
            10
        )

        # Duration for publishing empty test data
        self.empty_data_duration = 5.0

        # Keep track of when publishing started
        self.start_time = self.get_clock().now()

        # Publish at 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.publish_test_data
        )

    def create_empty_point_cloud_message(self):

        msg = PointCloud2()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "catvehicle/velodyne_link"

        msg.height = 1
        msg.width = 0

        msg.fields = []

        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 0

        msg.data = []

        msg.is_dense = True

        return msg

    def publish_test_data(self):

        current_time = self.get_clock().now()

        elapsed_time = (
            current_time - self.start_time
        ).nanoseconds / 1e9

        if elapsed_time < self.empty_data_duration:

            empty_msg = self.create_empty_point_cloud_message()

            self.pub.publish(empty_msg)

            self.get_logger().info(
                "Published empty test point cloud message"
            )

        else:
            # Reset the timer
            self.start_time = self.get_clock().now()


def main(args=None):

    rclpy.init(args=args)

    node = TestInjector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()