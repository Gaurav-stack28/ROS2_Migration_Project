
#!/usr/bin/env python3

import cv2
import numpy
import scipy.signal
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class SimpleLaneKeeping(Node):

    def __init__(self):
        super().__init__('simple_lane_keeping')

        # ------------------------------------------------------------
        # Publisher for vehicle velocity
        # ------------------------------------------------------------
        self.cmd_vel = self.create_publisher(
            Twist,
            '/catvehicle/cmd_vel',
            10
        )

        # ------------------------------------------------------------
        # Subscriber for camera image
        # ------------------------------------------------------------
        self.camera_sub = self.create_subscription(
            Image,
            '/catvehicle/camera_left/image_raw',
            self.camera_callback,
            10
        )

        self.move_cmd = Twist()

        self.img_width = 800
        self.img_height = 800

        self.distance = 0
        self.latest_image = None

        self.get_logger().info(
            'Simple lane keeping node started.'
        )

    # ================================================================
    # Display images
    # ================================================================

    def display_images(self, color_img, processed_img):

        # Original color camera image
        cv2.imshow(
            'Camera Color',
            color_img
        )

        # Processed lane detection image
        cv2.imshow(
            'Lane Detection',
            processed_img
        )

        cv2.waitKey(2)

    # ================================================================
    # Find lane starting points
    # ================================================================

    def find_lane_start(self, img):

        # Find lane lines on the right side
        peaks_r = scipy.signal.find_peaks(
            img[540:550, 470:550].sum(axis=0),
            prominence=(500, None)
        )[0] + 470

        # Find lane lines on the left side
        peaks_l = scipy.signal.find_peaks(
            img[540:550, 220:320].sum(axis=0),
            prominence=(500, None)
        )[0] + 220

        # Default lane positions
        lr = 525
        ll = 275

        if peaks_r.size > 0:
            lr = peaks_r[
                numpy.abs(peaks_r - lr).argmin()
            ]

        if peaks_l.size > 0:
            ll = peaks_l[
                numpy.abs(peaks_l - ll).argmin()
            ]

        return ll, lr

    # ================================================================
    # Move vehicle
    # ================================================================

    def move_vehicle(self, lateral_error):

        # Forward velocity
        forward_velocity = 10.0

        # Steering angle
        steering_angle = -numpy.arctan(
            lateral_error / 10.0
        )

        # Explicit float conversion
        # Required by ROS 2 geometry_msgs
        self.move_cmd.linear.x = float(
            forward_velocity
        )

        self.move_cmd.angular.z = float(
            steering_angle
        )

        self.cmd_vel.publish(
            self.move_cmd
        )

    # ================================================================
    # Camera callback
    # ================================================================

    def camera_callback(self, image_data):

        try:

            # --------------------------------------------------------
            # Convert ROS 2 Image message to NumPy image
            # --------------------------------------------------------

            input_image = numpy.frombuffer(
                image_data.data,
                dtype=numpy.uint8
            ).reshape(
                image_data.height,
                image_data.width,
                -1
            )

            # --------------------------------------------------------
            # Make sure image is BGR
            # --------------------------------------------------------

            if input_image.shape[2] == 4:

                input_image = cv2.cvtColor(
                    input_image,
                    cv2.COLOR_BGRA2BGR
                )

            # --------------------------------------------------------
            # Resize image to expected dimensions
            # --------------------------------------------------------

            input_image = cv2.resize(
                input_image,
                (800, 800)
            )

            # --------------------------------------------------------
            # Convert to grayscale
            # --------------------------------------------------------

            image = cv2.cvtColor(
                input_image,
                cv2.COLOR_BGR2GRAY
            )

            # --------------------------------------------------------
            # Thresholding
            # --------------------------------------------------------

            ret, threshold_image = cv2.threshold(
                image,
                200,
                255,
                cv2.THRESH_BINARY
            )

            # --------------------------------------------------------
            # Gaussian blur
            # --------------------------------------------------------

            edge_image = cv2.GaussianBlur(
                image,
                (5, 5),
                0
            )

            # --------------------------------------------------------
            # Canny edge detection
            # --------------------------------------------------------

            edge_image = cv2.Canny(
                edge_image,
                220,
                255
            )

            # --------------------------------------------------------
            # Combine threshold and edge images
            # --------------------------------------------------------

            combined_image = cv2.add(
                threshold_image,
                edge_image
            )

            # --------------------------------------------------------
            # Find lane starting points
            # --------------------------------------------------------

            lane_left_start, lane_right_start = (
                self.find_lane_start(
                    combined_image
                )
            )

            # --------------------------------------------------------
            # Calculate lateral error
            # --------------------------------------------------------

            lateral_error = float(
                (
                    lane_left_start +
                    lane_right_start -
                    800
                ) / 16.0
            )

            # --------------------------------------------------------
            # Display BOTH images
            # --------------------------------------------------------

            self.display_images(
                input_image,
                combined_image
            )

            # --------------------------------------------------------
            # Move vehicle
            # --------------------------------------------------------

            self.move_vehicle(
                lateral_error
            )

        except Exception as e:

            self.get_logger().error(
                'Camera processing error: {}'.format(e)
            )

    # ================================================================
    # Main
    # ================================================================

def main(args=None):

    rclpy.init(args=args)

    lane_keeper = SimpleLaneKeeping()

    try:

        rclpy.spin(
            lane_keeper
        )

    except KeyboardInterrupt:

        pass

    finally:

        # Stop vehicle before shutting down
        stop_cmd = Twist()

        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0

        lane_keeper.cmd_vel.publish(
            stop_cmd
        )

        lane_keeper.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
