#!/usr/bin/env python3

import cv2
import numpy
import scipy.signal
import scipy.optimize as spo
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from pyclothoids import Clothoid

import camera_transformation as ct


class LaneDetectionNode(Node):

    def __init__(self):

        super().__init__('lane_detection_node')

        # ---------------------------------------------------------
        # Publisher cmd_vel Steuerung Fahrzeug
        # ---------------------------------------------------------

        self.cmd_vel = self.create_publisher(
            Twist,
            '/catvehicle/cmd_vel',
            1
        )

        self.move_cmd = Twist()

        # ---------------------------------------------------------
        # Publisher für Kruemmung
        # ---------------------------------------------------------

        self.pub_co = self.create_publisher(
            Float64,
            '/catvehicle/curvature_opposing',
            10
        )

        self.pub_cl = self.create_publisher(
            Float64,
            '/catvehicle/curvature_left',
            10
        )

        self.pub_cr = self.create_publisher(
            Float64,
            '/catvehicle/curvature_right',
            10
        )

        # ---------------------------------------------------------
        # Kamera
        # ---------------------------------------------------------

        self.img_width = 800
        self.img_height = 800

        cc = 1.3962634

        self.Cam = ct.Camera(
            self.img_width,
            self.img_height,
            0,
            1.75,
            0.75,
            cc,
            cc,
            50,
            50
        )

        # ---------------------------------------------------------
        # Kamera Subscriber
        # ---------------------------------------------------------

        self.camera_sub = self.create_subscription(
            Image,
            '/catvehicle/camera_left/image_raw',
            self.camera_callback,
            10
        )

        self.get_logger().info(
            'Lane detection node started.'
        )

    # =============================================================
    # Display image
    # =============================================================

    def display_image(self, img):

        cv2.imshow(
            'Kamera',
            img
        )

        cv2.waitKey(2)

    # =============================================================
    # Find lane starting points
    # =============================================================

    def find_lane_start(self, img):

        # ---------------------------------------------------------
        # Search areas
        # ---------------------------------------------------------

        box_r = img[540:550, 450:550].sum(axis=0)
        box_l = img[540:550, 220:320].sum(axis=0)
        box_o = img[540:550, 0:100].sum(axis=0)

        # ---------------------------------------------------------
        # Original reference positions
        # ---------------------------------------------------------

        default_r = 500
        default_l = 270
        default_o = 50

        a = numpy.arange(0, 100)

        # ---------------------------------------------------------
        # IMPORTANT FIX:
        # Prevent division by zero when no lane pixels
        # are found in a search region.
        # ---------------------------------------------------------

        sum_r = box_r.sum()
        sum_l = box_l.sum()
        sum_o = box_o.sum()

        if sum_r > 0:
            weight_r = (
                (box_r * a).sum() / sum_r
            ) + 450

            lane_right_start = round(
                weight_r
            )

        else:
            lane_right_start = default_r

        if sum_l > 0:
            weight_l = (
                (box_l * a).sum() / sum_l
            ) + 220

            lane_left_start = round(
                weight_l
            )

        else:
            lane_left_start = default_l

        if sum_o > 0:
            weight_o = (
                (box_o * a).sum() / sum_o
            )

            lane_opposing_start = round(
                weight_o
            )

        else:
            lane_opposing_start = default_o

        return (
            lane_opposing_start,
            lane_left_start,
            lane_right_start,
            545
        )

    # =============================================================
    # Sliding-window algorithm
    # =============================================================

    def sliding_windows(
        self,
        img,
        lstx,
        starty,
        box_height,
        box_width
    ):

        pos_x = lstx
        pos_y = starty

        a = numpy.arange(
            0,
            box_width * 2
        )

        lane_x = []
        lane_y = []

        # ---------------------------------------------------------
        # First box
        # ---------------------------------------------------------

        box = img[
            pos_y - box_height:pos_y,
            pos_x - box_width:pos_x + box_width
        ].sum(axis=0)

        if box.sum() > 500:

            weight = (
                (box * a).sum()
                /
                box.sum(axis=0)
            )

            diff = round(
                weight - box_width
            )

        else:

            diff = 0

        a = numpy.arange(
            0,
            box_width * 2
        )

        pos_x += diff
        pos_y -= box_height

        lane_x.append(pos_x)
        lane_y.append(
            800 - pos_y
        )

        box = img[
            pos_y - box_height:pos_y,
            pos_x - box_width:pos_x + box_width
        ].sum(axis=0)

        # ---------------------------------------------------------
        # Sliding windows
        # ---------------------------------------------------------

        while box.sum() > 500:

            weight = (
                (box * a).sum()
                /
                box.sum(axis=0)
            )

            diff = round(
                weight - box_width
            )

            if pos_y > 470:

                box_height = 2
                box_width = 5

                a = numpy.arange(
                    0,
                    box_width * 2
                )

            pos_x += diff

            box = img[
                pos_y - box_height:pos_y,
                pos_x - box_width:pos_x + box_width
            ].sum(axis=0)

            lane_x.append(pos_x)

            lane_y.append(
                800 - pos_y
            )

            pos_x += (
                pos_x -
                lane_x[-2]
            )

            pos_y -= box_height

            box = img[
                pos_y - box_height:pos_y,
                pos_x - box_width:pos_x + box_width
            ].sum(axis=0)

        lane_x = numpy.array(
            lane_x
        )

        lane_y = numpy.array(
            lane_y
        )

        return (
            lane_x,
            lane_y
        )

    # =============================================================
    # Interpolate middle lane
    # =============================================================

    def interpolate_mid_lane(
        self,
        lx,
        ly,
        rx,
        ry,
        ox,
        oy
    ):

        diff = len(oy) - len(ry)

        if diff > 0:

            oy = oy[
                0:-diff-1
            ]

            ox = ox[
                0:-diff-1
            ]

        elif diff < 0:

            ry = ry[
                0:diff-1
            ]

            rx = rx[
                0:diff-1
            ]

        diff2 = len(ry) - len(ly)

        if diff2 > 0:

            ly = numpy.append(
                ly,
                ry[-diff2:]
            )

            lx = numpy.append(
                lx,
                (
                    ox[-diff2:]
                    +
                    rx[-diff2:]
                ) / 2
            )

        return (
            lx,
            ly,
            rx,
            ry,
            ox,
            oy
        )

    # =============================================================
    # Least squares error
    # =============================================================

    def lsq_error(
        self,
        param,
        lx,
        ly
    ):

        ast, aend, ex = param

        lx = numpy.array(lx)
        ly = numpy.array(ly)

        clothoid0 = Clothoid.G1Hermite(
            lx[0],
            ly[0],
            ast,
            ex,
            ly[-1],
            aend
        )

        cl = numpy.array([
            clothoid0.X(
                (i - ly[0])
                *
                (
                    clothoid0.length
                    /
                    (ly[-1] - ly[0])
                )
            )
            for i in ly
        ])

        diff = numpy.power(
            cl - lx,
            2
        )

        score = diff.sum()

        return score

    # =============================================================
    # Fit clothoid
    # =============================================================

    def fit_clothoid(
        self,
        lx,
        ly
    ):

        dy = (
            ly[-1]
            -
            ly[-5]
        )

        dx = (
            lx[-5]
            -
            lx[-1]
        )

        ang_end_guess = (
            numpy.arctan(dx / dy)
            +
            numpy.pi / 2
        )

        dy = (
            ly[5]
            -
            ly[0]
        )

        dx = (
            lx[0]
            -
            lx[5]
        )

        ang_st_guess = (
            numpy.arctan(dx / dy) / 2
            +
            numpy.pi / 2
        )

        parameters = numpy.array([
            ang_st_guess,
            ang_end_guess,
            lx[-1]
        ])

        result = spo.minimize(
            self.lsq_error,
            parameters,
            args=(lx, ly),
            tol=0.05,
            options={
                'maxiter': 20
            }
        )

        ang_start, ang_end, x_end = result.x

        clothoid0 = Clothoid.G1Hermite(
            lx[0],
            ly[0],
            ang_start,
            x_end,
            ly[-1],
            ang_end
        )

        curvature = []

        for s in ly:

            curvature.append(
                clothoid0.ThetaD(s)
            )

        sample_x, sample_y = (
            clothoid0.SampleXY(
                ly.size
            )
        )

        return (
            numpy.array(curvature),
            sample_x,
            sample_y
        )

    # =============================================================
    # Draw detected lane markings
    # =============================================================

    def draw_lines_on_input(
        self,
        img,
        lx,
        ly
    ):

        for e in range(
            len(ly) - 1
        ):

            img = cv2.line(
                img,
                (
                    int(lx[e]),
                    int(ly[e])
                ),
                (
                    int(lx[e + 1]),
                    int(ly[e + 1])
                ),
                (0, 255, 255),
                7
            )

        return img

    # =============================================================
    # Move vehicle
    # =============================================================

    def move_vehicle(
        self,
        lateral_error,
        vel
    ):

        steering_angle = -numpy.arctan(
            lateral_error / 2.5
        )

        self.move_cmd.linear.x = float(
            vel
        )

        self.move_cmd.angular.z = float(
            steering_angle
        )

        self.cmd_vel.publish(
            self.move_cmd
        )

    # =============================================================
    # Camera callback
    # =============================================================

    def camera_callback(
        self,
        image_data
    ):

        try:

            # -----------------------------------------------------
            # Input image
            # -----------------------------------------------------

            input_image = numpy.frombuffer(
                image_data.data,
                dtype=numpy.uint8
            ).reshape(
                image_data.height,
                image_data.width,
                -1
            )

            # -----------------------------------------------------
            # Convert to grayscale
            # -----------------------------------------------------

            image = cv2.cvtColor(
                input_image,
                cv2.COLOR_BGR2GRAY
            )

            # -----------------------------------------------------
            # ROI
            # -----------------------------------------------------

            image[0:-1][555:800] = 0
            image[0:-1][0:400] = 0

            # -----------------------------------------------------
            # Thresholding
            # -----------------------------------------------------

            ret, threshold_image = cv2.threshold(
                image,
                220,
                255,
                cv2.THRESH_BINARY
            )

            # -----------------------------------------------------
            # Gaussian blur
            # -----------------------------------------------------

            edge_image = cv2.GaussianBlur(
                image,
                (5, 5),
                0
            )

            # -----------------------------------------------------
            # Canny edge detection
            # -----------------------------------------------------

            edge_image = cv2.Canny(
                edge_image,
                200,
                250
            )

            # -----------------------------------------------------
            # Combined image
            # -----------------------------------------------------

            combined_image = (
                edge_image
                +
                threshold_image
            )

            # -----------------------------------------------------
            # Remove ROI edges
            # -----------------------------------------------------

            combined_image[0:-1][390:412] = 0
            combined_image[0:-1][550:560] = 0

            # -----------------------------------------------------
            # No bird's-eye transformation
            # -----------------------------------------------------

            combined_image_bv = combined_image

            # -----------------------------------------------------
            # Find lane starting points
            # -----------------------------------------------------

            (
                lane_opposing_start,
                lane_left_start,
                lane_right_start,
                start_y
            ) = self.find_lane_start(
                combined_image_bv
            )

            # -----------------------------------------------------
            # Left lane
            # -----------------------------------------------------

            (
                lane_points_left_x_px,
                lane_points_left_y_px
            ) = self.sliding_windows(
                combined_image_bv,
                lane_left_start,
                start_y,
                4,
                10
            )

            # -----------------------------------------------------
            # Right lane
            # -----------------------------------------------------

            (
                lane_points_right_x_px,
                lane_points_right_y_px
            ) = self.sliding_windows(
                combined_image_bv,
                lane_right_start,
                start_y,
                4,
                10
            )

            # -----------------------------------------------------
            # Opposing lane
            # -----------------------------------------------------

            (
                lane_points_opposing_x_px,
                lane_points_opposing_y_px
            ) = self.sliding_windows(
                combined_image_bv,
                lane_opposing_start,
                start_y,
                4,
                10
            )

            # -----------------------------------------------------
            # Pixel -> meter
            # -----------------------------------------------------

            (
                lane_points_left_x_m,
                lane_points_left_y_m
            ) = self.Cam.cam_coordinates_to_plane_m(
                lane_points_left_x_px,
                lane_points_left_y_px
            )

            (
                lane_points_right_x_m,
                lane_points_right_y_m
            ) = self.Cam.cam_coordinates_to_plane_m(
                lane_points_right_x_px,
                lane_points_right_y_px
            )

            (
                lane_points_opposing_x_m,
                lane_points_opposing_y_m
            ) = self.Cam.cam_coordinates_to_plane_m(
                lane_points_opposing_x_px,
                lane_points_opposing_y_px
            )

            # -----------------------------------------------------
            # Interpolate middle lane
            # -----------------------------------------------------

            (
                lane_points_left_x_m,
                lane_points_left_y_m,
                lane_points_right_x_m,
                lane_points_right_y_m,
                lane_points_opposing_x_m,
                lane_points_opposing_y_m
            ) = self.interpolate_mid_lane(
                lane_points_left_x_m,
                lane_points_left_y_m,
                lane_points_right_x_m,
                lane_points_right_y_m,
                lane_points_opposing_x_m,
                lane_points_opposing_y_m
            )

            # -----------------------------------------------------
            # Fit clothoids
            # -----------------------------------------------------

            if (
                lane_points_opposing_y_m.size > 20
                and
                lane_points_left_y_m.size > 20
                and
                lane_points_right_y_m.size > 20
            ):

                (
                    curvature_left,
                    clothoid_x_left,
                    clothoid_y_left
                ) = self.fit_clothoid(
                    lane_points_left_x_m[10:],
                    lane_points_left_y_m[10:]
                )

                (
                    curvature_right,
                    clothoid_x_right,
                    clothoid_y_right
                ) = self.fit_clothoid(
                    lane_points_right_x_m[10:],
                    lane_points_right_y_m[10:]
                )

                (
                    curvature_opposing,
                    clothoid_x_opposing,
                    clothoid_y_opposing
                ) = self.fit_clothoid(
                    lane_points_opposing_x_m[10:],
                    lane_points_opposing_y_m[10:]
                )

                # -------------------------------------------------
                # Clothoid -> image pixels
                # -------------------------------------------------

                (
                    cl_x_left_kp,
                    cl_y_left_kp
                ) = self.Cam.plane_m_to_cam_px(
                    clothoid_x_left,
                    clothoid_y_left
                )

                (
                    cl_x_right_kp,
                    cl_y_right_kp
                ) = self.Cam.plane_m_to_cam_px(
                    clothoid_x_right,
                    clothoid_y_right
                )

                (
                    cl_x_opposing_kp,
                    cl_y_opposing_kp
                ) = self.Cam.plane_m_to_cam_px(
                    clothoid_x_opposing,
                    clothoid_y_opposing
                )

                # -------------------------------------------------
                # Draw lanes
                # -------------------------------------------------

                cl_kp_img = numpy.zeros(
                    (800, 800, 3),
                    numpy.uint8
                )

                cl_kp_img = self.draw_lines_on_input(
                    cl_kp_img,
                    cl_x_left_kp,
                    cl_y_left_kp
                )

                cl_kp_img = self.draw_lines_on_input(
                    cl_kp_img,
                    cl_x_right_kp,
                    cl_y_right_kp
                )

                cl_kp_img = self.draw_lines_on_input(
                    cl_kp_img,
                    cl_x_opposing_kp,
                    cl_y_opposing_kp
                )

                cl_kp_img = cv2.addWeighted(
                    input_image,
                    1,
                    cl_kp_img,
                    0.9,
                    0
                )

                self.display_image(
                    cl_kp_img
                )

                # -------------------------------------------------
                # Publish curvature
                # -------------------------------------------------

                msg = Float64()

                msg.data = float(
                    curvature_opposing[0]
                )

                self.pub_co.publish(
                    msg
                )

                msg = Float64()

                msg.data = float(
                    curvature_left[0]
                )

                self.pub_cl.publish(
                    msg
                )

                msg = Float64()

                msg.data = float(
                    curvature_right[0]
                )

                self.pub_cr.publish(
                    msg
                )

                # -------------------------------------------------
                # Time
                # -------------------------------------------------

                current_time = (
                    self.get_clock()
                    .now()
                    .nanoseconds
                    /
                    1e9
                )

                self.get_logger().info(
                    'Right curvature: {:.6f}, '
                    'time: {:.6f}'.format(
                        curvature_right[0],
                        current_time
                    )
                )

                # -------------------------------------------------
                # Lateral error
                # -------------------------------------------------

                lateral_error = (
                    lane_points_left_x_m[0]
                    +
                    lane_points_right_x_m[0]
                ) / 2

                self.move_vehicle(
                    lateral_error,
                    5
                )

            # -----------------------------------------------------
            # Only left and right lanes detected
            # -----------------------------------------------------

            elif (
                lane_points_left_y_m.size > 0
                and
                lane_points_right_y_m.size > 0
            ):

                self.display_image(
                    input_image
                )

                lateral_error = (
                    lane_points_left_x_m[0]
                    +
                    lane_points_right_x_m[0]
                ) / 2

                self.move_vehicle(
                    lateral_error,
                    5
                )

            # -----------------------------------------------------
            # No lanes detected
            # -----------------------------------------------------

            else:

                self.display_image(
                    input_image
                )

        except Exception as exc:

            self.get_logger().error(
                'Error processing camera image: {}'.format(
                    exc
                )
            )


# ================================================================
# Main
# ================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    lane_follower = LaneDetectionNode()

    try:

        rclpy.spin(
            lane_follower
        )

    except KeyboardInterrupt:

        pass

    finally:

        # Stop vehicle
        stop_cmd = Twist()

        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0

        lane_follower.cmd_vel.publish(
            stop_cmd
        )

        lane_follower.destroy_node()

        rclpy.shutdown()

        cv2.destroyAllWindows()


if __name__ == '__main__':

    main()