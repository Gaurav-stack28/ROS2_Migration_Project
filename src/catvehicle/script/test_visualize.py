#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
import matplotlib.pyplot as plt
from collections import deque
import csv
import time


class LaneFollower(Node):

    def __init__(self):
        super().__init__('lane_detection')

        # ROS 2 subscriber
        self.point_cloud_sub = self.create_subscription(PointCloud2,'/output_point_cloud2_new',self.point_cloud_callback,10)

        # ROS 2 publisher for vehicle velocity
        self.cmd_vel_pub = self.create_publisher(Twist,'/catvehicle/cmd_vel',10)
           

        # ROS 2 publisher for navigation precision
        self.navigation_precision_pub = self.create_publisher(Float64,'/navigation_precision',10)

        # 10 Hz control loop
        self.rate = 0.1

        self.timer = self.create_timer(self.rate,self.control_loop)

        self.latest_points = None
        self.target_path = None

        self.base_look_ahead_distance = 5.5
        self.look_ahead_distance = self.base_look_ahead_distance

        self.lane_detected = False
        self.lane_markings_visible = True

        # Setup Matplotlib for interactive mode
        plt.ion()
        self.fig, self.ax = plt.subplots()

        # History buffer for lane clusters
        self.lane_history = deque(maxlen=10)

        self.target_path_buffer = deque(maxlen=2)

        # Initialize metrics
        self.total_frames = 0
        self.successful_detections = 0
        self.processing_times = []

        # CSV file for logging
        self.csv_file = open('lane_detection_metrics.csv',mode='w')

        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow(['Timestamp','Lane_Detection_Accuracy','Navigation_Precision','Processing_Time'])

        self.get_logger().info('Lane follower ROS 2 node initialized')

    def point_cloud_callback(self, msg):

        point_list = []

        for point in pc2.read_points(msg,skip_nans=True,field_names=("x", "y", "z", "rgba")):

            x, y, z, rgba = point

            r = int((rgba >> 16) & 0xFF)
            g = int((rgba >> 8) & 0xFF)
            b = int(rgba & 0xFF)

            # Threshold for white color
            # and range 0 to 10 meters in front
            # y within -2 to 4 meters

            if (
                r > 200
                and g > 200
                and b > 200
                and 0 <= x <= 10
                and -2 <= y <= 4
            ):
                point_list.append((x, y, z))

        self.get_logger().info(
            f"Filtered {len(point_list)} white points"
        )

        if len(point_list) == 0:
            self.lane_markings_visible = False
        else:
            self.lane_markings_visible = True

        self.latest_points = (
            point_list
            if len(point_list) > 0
            else self.latest_points
        )

    def process_point_list(self):

        start_time = time.time()

        self.total_frames += 1

        if (
            self.latest_points is None
            or not self.lane_markings_visible
        ):
            self.lane_detected = False
            self.log_metrics(False, 0)
            return

        points = np.array(self.latest_points)

        if points.size == 0:

            self.get_logger().info(
                "No white points found"
            )

            self.lane_detected = False
            self.log_metrics(False, 0)

            return

        points_2d = points[:, :2]

        # DBSCAN parameters
        eps_value = 2.65
        min_samples_value = 4

        clustering = DBSCAN(
            eps=eps_value,
            min_samples=min_samples_value
        ).fit(points_2d)

        labels = clustering.labels_

        self.get_logger().info(
            f"Number of clusters found: "
            f"{len(set(labels)) - (1 if -1 in labels else 0)}"
        )

        self.get_logger().info(
            f"DBSCAN parameters - "
            f"eps: {eps_value}, "
            f"min_samples: {min_samples_value}"
        )

        lanes = self.extract_lanes(
            points_2d,
            labels
        )

        if len(lanes) >= 1:

            lanes = sorted(
                lanes,
                key=len,
                reverse=True
            )[:2]

            self.lane_history.append(lanes)

            boundary_lines = self.fit_boundary_lines(
                lanes
            )

            self.target_path = self.calculate_target_path(
                boundary_lines
            )

            self.lane_detected = True

            self.update_visualization(
                lanes,
                boundary_lines,
                self.target_path
            )

            self.successful_detections += 1

        else:

            self.get_logger().info(
                "Expected at least 1 lane, but found none."
            )

            self.lane_detected = False

            # If fewer than 1 lane is detected,
            # use the most recent valid lanes from history

            if len(self.lane_history) > 0:

                lanes = self.lane_history[-1]

                boundary_lines = self.fit_boundary_lines(
                    lanes
                )

                self.target_path = self.calculate_target_path(
                    boundary_lines
                )

                self.update_visualization(
                    lanes,
                    boundary_lines,
                    self.target_path
                )

        end_time = time.time()

        processing_time = end_time - start_time

        self.processing_times.append(
            processing_time
        )

        self.log_metrics(
            self.lane_detected,
            processing_time
        )

    def log_metrics(
        self,
        lane_detected,
        processing_time
    ):

        lane_detection_accuracy = (
            self.successful_detections
            / self.total_frames
        ) * 100

        navigation_precision = (
            self.calculate_navigation_precision()
        )

        timestamp = time.time()

        self.csv_writer.writerow([
            timestamp,
            lane_detection_accuracy,
            navigation_precision,
            processing_time
        ])

        self.csv_file.flush()

    def calculate_navigation_precision(self):

        if not self.target_path:
            return 0.0

        target_slope, target_intercept = (
            self.target_path
        )

        deviation = abs(
            np.arctan(target_slope)
        )

        msg = Float64()

        msg.data = float(deviation)

        self.navigation_precision_pub.publish(
            msg
        )

        return deviation

    def extract_lanes(
        self,
        points_2d,
        labels
    ):

        lanes = []

        unique_labels = set(labels)

        for label in unique_labels:

            if label == -1:
                continue

            class_member_mask = (
                labels == label
            )

            xy = points_2d[
                class_member_mask
            ]

            # Ensure a reasonable number of points
            # for a lane and validate lane

            if (
                len(xy) > 6
                and self.is_valid_lane(xy)
            ):
                lanes.append(xy)

        return lanes

    def is_valid_lane(self, xy):

        # Define criteria for a valid lane

        if len(xy) < 6:
            return False

        # Check spread along x-axis

        x_min = np.min(xy[:, 0])
        x_max = np.max(xy[:, 0])

        if (x_max - x_min) < 2.0:
            return False

        # Fit a line and check deviation
        # of points from the line

        line_model = RANSACRegressor()

        X = xy[:, 0].reshape(-1, 1)
        y = xy[:, 1]

        line_model.fit(X, y)

        y_pred = line_model.predict(X)

        residuals = np.abs(
            y - y_pred
        )

        if np.mean(residuals) > 0.5:
            return False

        return True

    def fit_boundary_lines(self, lanes):

        boundary_lines = []

        for lane in lanes:

            # Fit RANSAC model
            # with linear regression

            X = lane[:, 0].reshape(-1, 1)
            y = lane[:, 1]

            ransac = RANSACRegressor()

            ransac.fit(X, y)

            # Extract linear coefficients

            slope = ransac.estimator_.coef_[0]

            intercept = (
                ransac.estimator_.intercept_
            )

            boundary_lines.append(
                (slope, intercept)
            )

        return boundary_lines

    def calculate_target_path(
        self,
        boundary_lines
    ):

        if len(boundary_lines) == 0:
            return None

        else:

            # Determine which boundary line
            # is right and which is left

            if len(boundary_lines) == 2:

                if (
                    boundary_lines[0][1]
                    > boundary_lines[1][1]
                ):

                    right_lane = boundary_lines[0]
                    left_lane = boundary_lines[1]

                else:

                    right_lane = boundary_lines[1]
                    left_lane = boundary_lines[0]

            else:

                # Only one lane detected,
                # treat it as the right lane

                right_lane = boundary_lines[0]
                left_lane = None

            # Hard code target path using
            # right lane slope and adjusted intercept

            if right_lane is not None:

                target_slope = right_lane[0]

                target_intercept = (
                    right_lane[1]
                    - 1.5
                    / np.sqrt(
                        1 + target_slope**2
                    )
                )

            else:

                target_slope = left_lane[0]

                target_intercept = (
                    left_lane[1]
                    + 1.5
                    / np.sqrt(
                        1 + target_slope**2
                    )
                )

            # Store calculated target path
            # in the buffer

            self.target_path_buffer.append(
                (
                    target_slope,
                    target_intercept
                )
            )

            # Compute smoothed target path
            # using moving average

            avg_slope = np.mean([
                path[0]
                for path in self.target_path_buffer
            ])

            avg_intercept = np.mean([
                path[1]
                for path in self.target_path_buffer
            ])

            return (
                avg_slope,
                avg_intercept
            )

    def update_visualization(
        self,
        lanes,
        boundary_lines,
        target_path
    ):

        self.ax.clear()

        # Plot lanes

        for lane in lanes:

            self.ax.plot(
                lane[:, 0],
                lane[:, 1],
                'o',
                label='Current Lane Points'
            )

        # Plot boundary lines

        for (
            (slope, intercept),
            lane
        ) in zip(
            boundary_lines,
            lanes
        ):

            x_vals = np.linspace(
                0,
                10,
                100
            )

            y_vals = (
                slope * x_vals
                + intercept
            )

            self.ax.plot(
                x_vals,
                y_vals,
                '--',
                label='Boundary Line'
            )

        # Plot target path

        if target_path is not None:

            target_slope, target_intercept = (
                target_path
            )

            x_vals = np.linspace(
                0,
                10,
                100
            )

            y_vals = (
                target_slope * x_vals
                + target_intercept
            )

            self.ax.plot(
                x_vals,
                y_vals,
                'r',
                linewidth=2,
                label='Target Path'
            )

        # Plot label for buffered lane points once

        if len(self.lane_history) > 0:

            first_lane_set = (
                self.lane_history[0]
            )

            if len(first_lane_set) > 0:

                first_lane = (
                    first_lane_set[0]
                )

                self.ax.plot(
                    first_lane[:, 0],
                    first_lane[:, 1],
                    'x',
                    alpha=0.3,
                    label='Buffered Lane Points'
                )

        # Plot buffered lane points
        # from history without label

        for lanes_in_history in (
            self.lane_history
        ):

            for lane in lanes_in_history:

                self.ax.plot(
                    lane[:, 0],
                    lane[:, 1],
                    'x',
                    alpha=0.3
                )

        self.ax.set_xlim([0, 10])
        self.ax.set_ylim([-5, 5])

        self.ax.set_aspect(
            'equal',
            adjustable='box'
        )

        self.ax.set_title(
            "Detected Lanes"
        )

        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")

        self.ax.grid(True)
        self.ax.legend()

        plt.draw()
        plt.pause(0.001)

    def calculate_look_ahead_distance(
        self,
        speed
    ):

        # Adjust look-ahead distance
        # based on current speed

        return max(
            self.base_look_ahead_distance,
            speed / 3.6 * 1.5
        )

    def move_vehicle(self):

        if not self.lane_markings_visible:

            # Stop vehicle if no white
            # lane markings are visible

            twist = Twist()

            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.cmd_vel_pub.publish(
                twist
            )

            return

        if (
            not self.lane_detected
            or self.target_path is None
        ):

            # Use most recent valid target path

            if len(self.lane_history) > 0:

                self.target_path = (
                    self.calculate_target_path(
                        self.fit_boundary_lines(
                            self.lane_history[-1]
                        )
                    )
                )

            if self.target_path is None:

                # Stop vehicle if no lanes
                # are detected at all

                twist = Twist()

                twist.linear.x = 0.0
                twist.angular.z = 0.0

                self.cmd_vel_pub.publish(
                    twist
                )

                return

        target_slope, target_intercept = (
            self.target_path
        )

        # Get current speed of vehicle
        # Assume constant speed of 10 m/s

        current_speed = 10.0

        # Calculate dynamic look-ahead distance

        self.look_ahead_distance = (
            self.calculate_look_ahead_distance(
                current_speed
            )
        )

        # Current vehicle position
        # Assuming it is at origin

        current_x = 0.0
        current_y = 0.0

        # Calculate look-ahead point

        look_ahead_x = (
            current_x
            + self.look_ahead_distance
        )

        look_ahead_y = (
            target_slope * look_ahead_x
            + target_intercept
        )

        # Validate look-ahead point

        if (
            look_ahead_y < -5
            or look_ahead_y > 5
        ):

            self.get_logger().info(
                "Invalid look-ahead point, "
                "stopping vehicle"
            )

            twist = Twist()

            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.cmd_vel_pub.publish(
                twist
            )

            return

        # Calculate steering angle
        # using Pure Pursuit formula

        dx = (
            look_ahead_x
            - current_x
        )

        dy = (
            look_ahead_y
            - current_y
        )

        Ld = np.sqrt(
            dx**2 + dy**2
        )

        alpha = np.arctan2(
            dy,
            dx
        )

        steering_angle = (
            2 * dy / (Ld**2)
        )

        # Adjust steering angle for right turns

        if target_slope < 0:

            steering_angle = -abs(
                steering_angle
            )

        else:

            steering_angle = abs(
                steering_angle
            )

        twist = Twist()

        twist.linear.x = current_speed
        twist.angular.z = steering_angle

        self.cmd_vel_pub.publish(
            twist
        )

    def control_loop(self):

        self.process_point_list()
        self.move_vehicle()


def main(args=None):

    rclpy.init(args=args)

    lane_follower = LaneFollower()

    try:

        rclpy.spin(
            lane_follower
        )

    except KeyboardInterrupt:

        pass

    finally:

        lane_follower.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()