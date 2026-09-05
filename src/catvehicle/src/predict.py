#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import String, Header, Float64
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point

import sys
import getopt
import time

import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.animation as animation
from matplotlib.ticker import LinearLocator, FormatStrFormatter

import scipy.special as sp

from gazebo_msgs.srv import GetModelState

from sklearn.preprocessing import MinMaxScaler

import tensorflow as tf
import pandas as pd
import pickle


print("Tensorflow Version :{}".format(tf.__version__))


class predict(Node):

    def __init__(self, ns):

        super().__init__("rnn_predict")

        # Transformer for data scaling
        self.model_variables = None

        with open(
            '/home/ivory/VersionControl/CopyCAT-reu2020/saved models/7-20-1-DeltaV/structure.pickle',
            mode="rb"
        ) as f:
            self.model_variables = pickle.load(f)

        print("===========")
        print("Model Data:")
        print(self.model_variables)

        self.history = self.model_variables[0]
        self.n_features = self.model_variables[1]

        self.deltaV_min = self.model_variables[3][0]
        self.deltaV_max = self.model_variables[3][1]

        self.dist_min = self.model_variables[5][0][0]
        self.dist_max = self.model_variables[5][1][0]

        self.accel_min = self.model_variables[5][0][1]
        self.accel_max = self.model_variables[5][1][1]

        self.vel_min = self.model_variables[5][0][2]
        self.vel_max = self.model_variables[5][1][2]

        print("deltaV_min: {}".format(self.deltaV_min))
        print("deltaV_max: {}".format(self.deltaV_max))
        print("dist_min: {}".format(self.dist_min))
        print("dist_max: {}".format(self.dist_max))
        print("accel_min: {}".format(self.accel_min))
        print("accel_max: {}".format(self.accel_max))
        print("vel_min: {}".format(self.vel_min))
        print("vel_max: {}".format(self.vel_max))
        print("===========")

        # self.dist_min = 0.3540591169968693
        # self.dist_max = 252.0000000000338
        # self.accel_min = -3.474197796605082
        # self.accel_max = 3.194018015245345
        # self.vel_min = -0.851840092450414
        # self.vel_max = 112.55704797888872

        self.dist_scaler = MinMaxScaler()
        self.accel_scaler = MinMaxScaler()
        self.vel_scaler = MinMaxScaler()

        self.dist_scaler.fit([[self.dist_min], [self.dist_max]])
        self.accel_scaler.fit([[self.accel_min], [self.accel_max]])
        self.vel_scaler.fit([[self.vel_min], [self.vel_max]])

        # Variables for holding data
        self.dist_list = []
        self.vel_list = []
        self.accel_list = []

        self.last_velocity_time = None
        self.current_velocity_time = None

        self.last_distance_time = None
        self.current_distance_time = None

        self.new_vel_msg = False
        self.new_vel = 0.0

        self.new_distance_msg = False
        self.new_distance = 0.0

        self.ns = ns

        # Publisher
        self.vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            1
        )

        # Load the saved model
        self.model = tf.keras.models.load_model(
            "/home/ivory/VersionControl/CopyCAT-reu2020/saved models/7-20-1-DeltaV"
        )

        # Subscribers
        self.vel_sub = self.create_subscription(
            Twist,
            'vel',
            self.vellcallback,
            10
        )

        self.distance_sub = self.create_subscription(
            Float64,
            'distanceEstimatorSteeringBased/dist',
            self.distance_calback,
            10
        )

        # 20 Hz timer
        self.timer = self.create_timer(
            1.0 / 20.0,
            self.publish
        )

    def vellcallback(self, data):
        """
        Velocity Call Back
        """

        # Retrieve Linear X Component of the Velocity
        self.new_vel = data.linear.x

        if self.new_vel < 0.0:
            self.new_vel = 0.01

        # Add new velocity to the velocity list
        self.vel_list.append(self.new_vel)

        # Assign current velocity time to last velocity time
        # before getting a new time
        self.last_velocity_time = self.current_velocity_time
        self.current_velocity_time = self.get_clock().now()

        if (
            self.last_velocity_time is not None
            and self.current_velocity_time is not None
        ):

            duration = (
                self.current_velocity_time
                - self.last_velocity_time
            )

            deltaT = duration.nanoseconds / 1e9

            if deltaT == 0.0:
                self.new_vel_msg = False
                return

            # Calculate instantaneous acceleration
            accel = (
                self.vel_list[-1] - self.vel_list[-2]
            ) / deltaT

            self.accel_list.append(accel)

        # If a new velocity data point is received
        # then set vel new to true
        self.new_vel_msg = True

    def distance_calback(self, data):
        """
        Distance Call Back
        """

        # Retrieve current distance
        self.new_distance = data.data

        # Add new distance to the distance list
        self.dist_list.append(self.new_distance)

        # Assign current distance time to last distance time
        self.last_distance_time = self.current_distance_time
        self.current_distance_time = self.get_clock().now()

        # If a new distance data point is received
        # then set distance new to true
        self.new_distance_msg = True

    @staticmethod
    def moving_avg(datalist, window_size=10):
        pass

    def publish(self):
        """
        Publish Function
        """

        if (
            self.new_distance_msg is True
            or self.new_vel_msg is True
        ):

            if (
                len(self.vel_list) <= self.history
                or len(self.dist_list) <= self.history
                or len(self.accel_list) <= self.history
            ):
                return

            # scaled_vel_data = self.vel_scaler.transform(
            #     [self.vel_list[-self.history:]]
            # )

            # scaled_dist_data = self.dist_scaler.transform(
            #     [self.dist_list[-self.history:]]
            # )

            # scaled_accel_data = self.accel_scaler.transform(
            #     [self.accel_list[-self.history:]]
            # )

            # TODO
            # Use moving average to smooth out acceleration
            # history before using it to make predictions

            # TODO
            # How we create data vector for input
            # to self.model.predict?

            scaled_data_vector = None
            scaled_data_vector = pd.DataFrame()

            # TODO: reorder data columns as per specification
            #
            # scaled_data_vector['dist'] = scaled_dist_data[0]
            # scaled_data_vector['accel'] = scaled_accel_data[0]
            # scaled_data_vector['speed'] = scaled_vel_data[0]

            scaled_data_vector['dist'] = [
                (x - self.dist_min)
                / (self.dist_max - self.dist_min)
                for x in self.dist_list[-self.history:]
            ]

            scaled_data_vector['accel'] = [
                (x - self.accel_min)
                / (self.accel_max - self.accel_min)
                for x in self.accel_list[-self.history:]
            ]

            scaled_data_vector['speed'] = [
                (x - self.vel_min)
                / (self.vel_max - self.vel_min)
                for x in self.vel_list[-self.history:]
            ]

            predicted_vel_diff = self.model.predict(
                scaled_data_vector
                .tail(n=self.history)
                .to_numpy()
                .reshape(1, self.history, 3)
            )

            predicted_vel_diff_unscaled = (
                predicted_vel_diff[0]
                * (self.deltaV_max - self.deltaV_min)
                + self.deltaV_min
            )

            print(
                "Predicted vel diff is: {}".format(
                    predicted_vel_diff_unscaled
                )
            )

            new_vel = (
                self.vel_list[-1]
                + predicted_vel_diff_unscaled[0]
                + 0.1
            )

            print("New Velocity is: {}".format(new_vel))

            new_vel_msg = Twist()

            new_vel_msg.linear.x = new_vel
            new_vel_msg.linear.y = 0.0
            new_vel_msg.linear.z = 0.0

            new_vel_msg.angular.x = 0.0
            new_vel_msg.angular.y = 0.0
            new_vel_msg.angular.z = 0.0

            self.vel_pub.publish(new_vel_msg)

            self.new_distance_msg = False
            self.new_vel_msg = False


def main(args=None):

    rclpy.init(args=args)

    # ROS 2 node namespace
    node = predict('')

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()