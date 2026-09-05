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
import math
import time
import signal
import subprocess
import shlex
from subprocess import call
import psutil

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

import strym
from strym import strymread

print("Tensforflow Version: {}".format(tf.__version__))


# Initial Date: June 2020
# Author: Rahul Bhadani
# Copyright (c) Rahul Bhadani, Arizona Board of Regents
# All rights reserved.


class launch:
    '''
    `launch`: A class facilitating launch handling.

    NOTE:
    The original implementation used ROS 1 roslaunch.
    That functionality has been removed from this ROS 2
    migration because the class is not used by the main
    drive_lead execution path.
    '''

    def __init__(self, launchfile, **kwargs):
        self.launchfile = launchfile
        self.runtime_args = []

        for key in kwargs.keys():
            self.runtime_args.append(
                "{}:={}".format(key, kwargs[key])
            )

        print(
            "ROS 2 migration: launch helper initialized for {}".format(
                self.launchfile
            )
        )

    def start(self):
        print(
            "ROS 2 migration: launch.start() called for {}".format(
                self.launchfile
            )
        )

    def shutdown(self):
        print(
            "ROS 2 migration: launch.shutdown() called for {}".format(
                self.launchfile
            )
        )


class lead_drive(Node):

    def __init__(self, ns, csvfile, dbcfile):

        super().__init__(
            "rnn_predict",
            namespace=ns
        )

        self.ns = ns

        # Publisher
        self.vel_pub = self.create_publisher(
            Twist,
            "cmd_vel",
            1
        )

        self.r = strymread(
            csvfile=csvfile,
            dbcfile=dbcfile
        )

        # State information of lead from radar traces

        # self.long_dist = self.r.long_dist(np.arange(0, 16))
        # self.lat_dist = self.r.lat_dist(np.arange(0, 16))
        # self.rel_vel = self.r.rel_velocity(np.arange(0, 16))

        # # combine all tracks from radar
        # df_long_dist = pd.concat(long_dist)
        # df_lat_dist = pd.concat(lat_dist)
        # df_rel_vel = pd.concat(rel_vel)

        # # create a consolidated dataframe for vehicle state
        # df_long_dist['Long'] = df_long_dist['Message']
        # df_long_dist['Lat'] = df_lat_dist['Message']
        # df_long_dist['Relvel'] = df_rel_vel['Message']

        # df_long_dist.drop(columns=['Message'])

        # # Keep lead vehicle's information for which there is something in front of the car.
        # df_long_dist = df_long_dist[np.abs(df_long_dist['Lat']) < 1.0]

        # self.lead_state = df_long_dist

        # speed of RAV4 (called as ego vehicle)
        self.ego_speed = self.r.speed()

        # Convert km/h to m/s
        self.ego_speed['Message'] = (
            self.ego_speed['Message'] * 0.277778
        )

        # Remove zero values from the beginning
        # so that car moves immediately
        positive_vales = self.ego_speed[
            self.ego_speed['Message'] > 0.0
        ]

        self.ego_speed = self.ego_speed[
            positive_vales.index[0]:
        ]

        self.current_time = None
        self.next_time = None

    def publish(self):
        """
        Publish Function
        """

        # Check whether data is available
        if self.ego_speed.shape[0] == 0:
            self.get_logger().warning(
                "No ego speed data remaining."
            )
            self.next_time = -1
            return

        new_vel = self.ego_speed.iloc[0]['Message']

        self.current_time = self.ego_speed.iloc[0]['Time']

        if self.ego_speed.shape[0] == 1:
            self.next_time = -1

            # Publish the final velocity
            new_vel_msg = Twist()

            new_vel_msg.linear.x = float(new_vel)
            new_vel_msg.linear.y = 0.0
            new_vel_msg.linear.z = 0.0

            new_vel_msg.angular.x = 0.0
            new_vel_msg.angular.y = 0.0
            new_vel_msg.angular.z = 0.0

            self.vel_pub.publish(new_vel_msg)

            return

        self.next_time = self.ego_speed.iloc[1]['Time']

        # Remove the row just read from the dataframe
        self.ego_speed = self.ego_speed.iloc[1:]

        new_vel_msg = Twist()

        new_vel_msg.linear.x = float(new_vel)
        new_vel_msg.linear.y = 0.0
        new_vel_msg.linear.z = 0.0

        new_vel_msg.angular.x = 0.0
        new_vel_msg.angular.y = 0.0
        new_vel_msg.angular.z = 0.0

        self.vel_pub.publish(new_vel_msg)


def main(argv):

    rclpy.init()

    # Create a temporary node to obtain the namespace
    namespace_node = Node("drive_lead_namespace_helper")

    ns = namespace_node.get_namespace()

    if ns.endswith('/'):
        ns = ns[:-1]

    namespace_node.destroy_node()

    if len(argv) < 2:
        print(
            "Usage: drive_lead.py <csvfile> <dbcfile>"
        )
        rclpy.shutdown()
        return

    csvfile = argv[0]
    dbcfile = argv[1]

    node = lead_drive(
        ns,
        csvfile,
        dbcfile
    )

    try:

        while rclpy.ok():

            node.publish()

            if node.next_time == -1:
                break

            deltaT = node.next_time - node.current_time

            # Keep the original timing behavior
            if deltaT > 0:
                time.sleep(deltaT)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])