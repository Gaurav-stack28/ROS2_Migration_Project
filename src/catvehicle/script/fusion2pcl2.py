#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, Image, PointField
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs_py import point_cloud2
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
from plc2pxl import pcl2pxl


class PointCloudToPointCloud2(Node):

    def __init__(self):
        super().__init__('pointcloud_to_pointcloud2_converter')

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create a PointCloud2 publisher
        self.publisher = self.create_publisher(
            PointCloud2,
            '/output_point_cloud2',
            10
        )

        # Create Subscribers for the point cloud and image topics
        self.pc_sub = Subscriber(
            self,
            PointCloud2,
            '/catvehicle/scan'
        )

        self.image_sub = Subscriber(
            self,
            Image,
            '/catvehicle/camera_left/image_raw_left'
        )

        # Initialize an ApproximateTimeSynchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.pc_sub, self.image_sub],
            queue_size=10,
            slop=0.1
        )

        self.sync.registerCallback(
            self.synchronized_callback
        )

    def synchronized_callback(self, pointcloud_msg, image_msg):

        # Convert the image message to a NumPy array
        latest_image = self.bridge.imgmsg_to_cv2(
            image_msg,
            desired_encoding='bgr8'
        )

        # Process point cloud data
        point_generator = point_cloud2.read_points(
            pointcloud_msg,
            skip_nans=True,
            field_names=('x', 'y', 'z')
        )

        # Get image dimensions for boundary checks
        img_height, img_width, _ = latest_image.shape

        # Prepare the list of points in the correct format for PointCloud2
        points = []

        # Process points
        for point in point_generator:

            x, y, z = point

            # Convert 3D point to pixel coordinates
            u, v = pcl2pxl(x, y, z)

            # Check if pixel coordinates are within image bounds
            if 0 <= u < img_width and 0 <= v < img_height:

                # Retrieve color at pixel (u, v)
                color_bgr = latest_image[int(v), int(u)]

                # Convert BGR to RGBA
                color_rgba = ColorRGBA(
                    r=color_bgr[2] / 255.0,
                    g=color_bgr[1] / 255.0,
                    b=color_bgr[0] / 255.0,
                    a=1.0
                )

                # Convert RGBA to uint32
                rgba = self.rgba_to_uint32(color_rgba)

                # Add point and color
                points.append((x, y, z, rgba))

        # Create PointCloud2 message
        header = pointcloud_msg.header
        header.frame_id = "catvehicle/velodyne_link"

        fields = [
            PointField(
                name='x',
                offset=0,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='y',
                offset=4,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='z',
                offset=8,
                datatype=PointField.FLOAT32,
                count=1
            ),
            PointField(
                name='rgba',
                offset=12,
                datatype=PointField.UINT32,
                count=1
            )
        ]

        # Create PointCloud2 message
        cloud = point_cloud2.create_cloud(
            header,
            fields,
            points
        )

        # Publish
        self.publisher.publish(cloud)

    def rgba_to_uint32(self, color):

        rgba = (
            int(color.a * 255) << 24 |
            int(color.b * 255) << 16 |
            int(color.g * 255) << 8 |
            int(color.r * 255)
        )

        return rgba

    def run(self):

        # Spin to keep the script running
        rclpy.spin(self)


if __name__ == '__main__':

    rclpy.init()

    converter = PointCloudToPointCloud2()

    try:
        converter.run()

    except KeyboardInterrupt:
        pass

    finally:
        converter.destroy_node()
        rclpy.shutdown()
