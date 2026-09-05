#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
from sensor_msgs_py import point_cloud2 as pc2

import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
import matplotlib.pyplot as plt
from collections import deque


class LaneFollower(Node):

    def __init__(self):
        super().__init__('lane_detection')

        # Subscribe to point cloud
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/output_point_cloud2_new',
            self.point_cloud_callback,
            10
        )

        # Publish vehicle velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/catvehicle/cmd_vel',
            10
        )

        # ROS 2 timer for periodic processing
        # 10 Hz = 0.1 seconds
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

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

        # Buffer for target path smoothing
        self.target_path_buffer = deque(maxlen=2)

    # ---------------------------------------------------------
    # ROS 2 timer callback
    # ---------------------------------------------------------

    def timer_callback(self):
        self.process_point_list()
        self.move_vehicle()

    # ---------------------------------------------------------
    # Point cloud callback
    # ---------------------------------------------------------

    def point_cloud_callback(self, msg):

        point_list = []

        for point in pc2.read_points(
            msg,
            field_names=("x", "y", "z", "rgba"),
            skip_nans=True
        ):

            x, y, z, rgba = point

            # Extract RGB values from packed RGBA value
            rgba_int = int(rgba)

            r = int((rgba_int >> 16) & 0xFF)
            g = int((rgba_int >> 8) & 0xFF)
            b = int(rgba_int & 0xFF)

            # Threshold for white color
            #
            # Range:
            # x: 0 to 10 meters
            # y: -2 to 4 meters
            #
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

        # Keep previous points if no new points were detected
        self.latest_points = (
            point_list
            if len(point_list) > 0
            else self.latest_points
        )

    # ---------------------------------------------------------
    # Process point cloud
    # ---------------------------------------------------------

    def process_point_list(self):

        if (
            self.latest_points is None
            or not self.lane_markings_visible
        ):
            self.lane_detected = False
            return

        points = np.array(self.latest_points)

        if points.size == 0:

            self.get_logger().info(
                "No white points found"
            )

            self.lane_detected = False
            return

        # Use X and Y coordinates for clustering
        points_2d = points[:, :2]

        # DBSCAN parameters
        eps_value = 2.65
        min_samples_value = 4

        clustering = DBSCAN(
            eps=eps_value,
            min_samples=min_samples_value
        ).fit(points_2d)

        labels = clustering.labels_

        number_of_clusters = (
            len(set(labels))
            - (1 if -1 in labels else 0)
        )

        self.get_logger().info(
            f"Number of clusters found: "
            f"{number_of_clusters}"
        )

        self.get_logger().info(
            f"DBSCAN parameters - "
            f"eps: {eps_value}, "
            f"min_samples: {min_samples_value}"
        )

        # Extract lane clusters
        lanes = self.extract_lanes(
            points_2d,
            labels
        )

        if len(lanes) >= 1:

            # Take the two largest clusters
            lanes = sorted(
                lanes,
                key=len,
                reverse=True
            )[:2]

            # Save lane history
            self.lane_history.append(lanes)

            # Fit boundary lines
            boundary_lines = self.fit_boundary_lines(
                lanes
            )

            # Calculate target path
            self.target_path = self.calculate_target_path(
                boundary_lines
            )

            self.lane_detected = True

            # Visualize
            self.update_visualization(
                lanes,
                boundary_lines,
                self.target_path
            )

        else:

            self.get_logger().info(
                "Expected at least 1 lane, but found none."
            )

            self.lane_detected = False

            # Use most recent valid lanes
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

    # ---------------------------------------------------------
    # Extract lane clusters
    # ---------------------------------------------------------

    def extract_lanes(self, points_2d, labels):

        lanes = []

        unique_labels = set(labels)

        for label in unique_labels:

            # Skip DBSCAN noise
            if label == -1:
                continue

            class_member_mask = (
                labels == label
            )

            xy = points_2d[
                class_member_mask
            ]

            # Ensure enough points and validate lane
            if (
                len(xy) > 6
                and self.is_valid_lane(xy)
            ):
                lanes.append(xy)

        return lanes

    # ---------------------------------------------------------
    # Validate lane
    # ---------------------------------------------------------

    def is_valid_lane(self, xy):

        if len(xy) < 6:
            return False

        # Check lane length along X axis
        x_min = np.min(xy[:, 0])
        x_max = np.max(xy[:, 0])

        if (x_max - x_min) < 2.0:
            return False

        # Fit line using RANSAC
        line_model = RANSACRegressor()

        X = xy[:, 0].reshape(-1, 1)
        y = xy[:, 1]

        line_model.fit(X, y)

        y_pred = line_model.predict(X)

        residuals = np.abs(
            y - y_pred
        )

        # Check average deviation
        if np.mean(residuals) > 0.5:
            return False

        return True

    # ---------------------------------------------------------
    # Fit boundary lines
    # ---------------------------------------------------------

    def fit_boundary_lines(self, lanes):

        boundary_lines = []

        for lane in lanes:

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

    # ---------------------------------------------------------
    # Calculate target path
    # ---------------------------------------------------------

    def calculate_target_path(self, boundary_lines):

        if len(boundary_lines) == 0:
            return None

        # Two boundaries detected
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

            # Only one lane detected
            # Treat it as right lane
            right_lane = boundary_lines[0]
            left_lane = None

        # Calculate target path
        if right_lane is not None:

            target_slope = right_lane[0]

            target_intercept = (
                right_lane[1]
                - 1.5
                / np.sqrt(
                    1 + target_slope ** 2
                )
            )

        else:

            target_slope = left_lane[0]

            target_intercept = (
                left_lane[1]
                + 1.5
                / np.sqrt(
                    1 + target_slope ** 2
                )
            )

        # Store target path
        self.target_path_buffer.append(
            (
                target_slope,
                target_intercept
            )
        )

        # Moving average smoothing
        avg_slope = np.mean(
            [
                path[0]
                for path in self.target_path_buffer
            ]
        )

        avg_intercept = np.mean(
            [
                path[1]
                for path in self.target_path_buffer
            ]
        )

        return (
            avg_slope,
            avg_intercept
        )

    # ---------------------------------------------------------
    # Visualization
    # ---------------------------------------------------------

    def update_visualization(
        self,
        lanes,
        boundary_lines,
        target_path
    ):

        self.ax.clear()

        # Plot current lane points
        for lane in lanes:

            self.ax.plot(
                lane[:, 0],
                lane[:, 1],
                'o',
                label='Current Lane Points'
            )

        # Plot boundary lines
        for (
            slope,
            intercept
        ), lane in zip(
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

        # Plot buffered lane points
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

        # Plot remaining history
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

        self.ax.set_xlim(
            [0, 10]
        )

        self.ax.set_ylim(
            [-5, 5]
        )

        self.ax.set_aspect(
            'equal',
            adjustable='box'
        )

        self.ax.set_title(
            "Detected Lanes"
        )

        self.ax.set_xlabel(
            "X"
        )

        self.ax.set_ylabel(
            "Y"
        )

        self.ax.grid(True)

        self.ax.legend()

        plt.draw()

        plt.pause(0.001)

    # ---------------------------------------------------------
    # Calculate look-ahead distance
    # ---------------------------------------------------------

    def calculate_look_ahead_distance(
        self,
        speed
    ):

        return max(
            self.base_look_ahead_distance,
            speed / 3.6 * 1.5
        )

    # ---------------------------------------------------------
    # Vehicle control
    # ---------------------------------------------------------

    def move_vehicle(self):

        # No lane markings visible
        if not self.lane_markings_visible:

            twist = Twist()

            twist.linear.x = 0.0
            twist.angular.z = 0.0

            self.cmd_vel_pub.publish(
                twist
            )

            return

        # No lane detected
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

            # Stop if no target path
            if self.target_path is None:

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

        # Assumed current vehicle speed
        current_speed = 10.0

        # Dynamic look-ahead distance
        self.look_ahead_distance = (
            self.calculate_look_ahead_distance(
                current_speed
            )
        )

        # Vehicle position
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

        # Pure Pursuit calculation
        dx = (
            look_ahead_x
            - current_x
        )

        dy = (
            look_ahead_y
            - current_y
        )

        Ld = np.sqrt(
            dx ** 2
            + dy ** 2
        )

        alpha = np.arctan2(
            dy,
            dx
        )

        steering_angle = (
            2 * dy / (Ld ** 2)
        )

        # Adjust steering direction
        if target_slope < 0:

            steering_angle = -abs(
                steering_angle
            )

        else:

            steering_angle = abs(
                steering_angle
            )

        # Create velocity command
        twist = Twist()

        twist.linear.x = current_speed
        twist.angular.z = steering_angle

        self.cmd_vel_pub.publish(
            twist
        )


# -------------------------------------------------------------
# Main
# -------------------------------------------------------------

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