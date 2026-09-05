#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool


class DataMultiplexer(Node):

    def __init__(self):
        super().__init__('data_multiplexer')

        # Subscriber for real point cloud data
        self.real_data_sub = self.create_subscription(
            PointCloud2,
            '/output_point_cloud2',
            self.handle_real_data,
            10
        )

        # Subscriber for test point cloud data
        self.test_data_sub = self.create_subscription(
            PointCloud2,
            '/test_point_cloud',
            self.handle_test_data,
            10
        )

        # Publisher for selected point cloud data
        self.output_pub = self.create_publisher(
            PointCloud2,
            '/output_point_cloud2_new',
            10
        )

        # False = real data
        # True  = test data
        self.use_test_data = False

        # Store latest real point cloud
        self.latest_real_data = None

        # ROS 2 service
        self.toggle_service = self.create_service(
            SetBool,
            'toggle_test_data',
            self.handle_toggle_test_data
        )

        self.get_logger().info(
            'Data multiplexer initialized'
        )

    def handle_real_data(self, msg):
        """
        Handle point cloud data from the real sensor.
        """

        self.latest_real_data = msg

        if not self.use_test_data:
            self.output_pub.publish(msg)

    def handle_test_data(self, msg):
        """
        Handle point cloud data from the test publisher.
        """

        if self.use_test_data:
            self.output_pub.publish(msg)

    def handle_toggle_test_data(self, request, response):
        """
        Enable or disable test point cloud data.

        request.data = True:
            Use test point cloud.

        request.data = False:
            Use real point cloud.
        """

        self.use_test_data = request.data

        response.success = True
        response.message = (
            'Toggled test data to {}'.format(
                self.use_test_data
            )
        )

        self.get_logger().info(
            response.message
        )

        return response


def main(args=None):

    rclpy.init(args=args)

    mux = DataMultiplexer()

    try:
        rclpy.spin(mux)

    except KeyboardInterrupt:
        pass

    finally:
        mux.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()