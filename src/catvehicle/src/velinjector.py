#!/usr/bin/env python3


# Author : Rahul Bhadani
# Initial Date: Nov 15, 2020
# License: MIT License

#   Permission is hereby granted, free of charge, to any person obtaining
#   a copy of this software and associated documentation files
#   (the "Software"), to deal in the Software without restriction, including
#   without limitation the rights to use, copy, modify, merge, publish,
#   distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject
#   to the following conditions:

#   The above copyright notice and this permission notice shall be
#   included in all copies or substantial portions of the Software.

#!/usr/bin/env python3

# Author : Rahul Bhadani
# Initial Date: Nov 15, 2020
# License: MIT License

import rclpy
from rclpy.node import Node

import numpy as np

from geometry_msgs.msg import Twist

import sys
import getopt
import time
import ntpath

import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import matplotlib.animation as animation

import pandas as pd


class VelInjector(Node):

    def __init__(
        self,
        ns,
        csvfile,
        time_col,
        vel_col,
        str_angle,
        input_type,
        **kwargs
    ):
        """
        Inject velocity from a CSV file to a vehicle in simulation.

        ns: Namespace of the ego vehicle

        csvfile: Filepath of the CSV file

        time_col: Name of the time column

        vel_col: Name of the velocity column

        input_type:
            "CAN": CAN data input
            "CSV": Regular CSV input
            "bag": ROS bag input

        str_angle: Steering angle for commanded velocity
        """

        super().__init__('injector')

        self.ns = ns

        # Publisher
        self.vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            1
        )

        self.str_angle = str_angle

        self.speed = pd.DataFrame()

        current_time = None
        velocity = None

        if input_type == "CSV":

            dataframe = pd.read_csv(csvfile)

            dataframe.dropna(inplace=True)

            if time_col in dataframe.columns:
                current_time = dataframe[time_col]
            else:
                raise KeyError(
                    "{} column not available in {}".format(
                        time_col,
                        csvfile
                    )
                )

            if vel_col in dataframe.columns:
                velocity = dataframe[vel_col]
            else:
                raise KeyError(
                    "{} column not available in {}".format(
                        vel_col,
                        csvfile
                    )
                )

            self.dataframe = dataframe

            self.speed['Time'] = current_time
            self.speed['Message'] = velocity

            # Check monotonicity of time
            time_diff = np.diff(current_time)

            if not np.all(time_diff >= 0):
                raise ValueError(
                    "Time is not monotonically increasing "
                    "in the provided dataset"
                )

        # CAN input support was already commented out in the ROS1 version.

        self.current_time = None
        self.next_time = None

    def publish(self):
        """
        Publish velocity from the CSV data.
        """

        if self.speed.empty:
            self.next_time = -1
            return

        new_vel = self.speed.iloc[0]['Message']
        self.current_time = self.speed.iloc[0]['Time']

        if self.speed.shape[0] == 1:

            self.next_time = -1

        else:

            self.next_time = self.speed.iloc[1]['Time']

        # Remove the row just read
        self.speed = self.speed.iloc[1:]

        new_vel_msg = Twist()

        new_vel_msg.linear.x = float(new_vel)
        new_vel_msg.linear.y = 0.0
        new_vel_msg.linear.z = 0.0

        new_vel_msg.angular.x = 0.0
        new_vel_msg.angular.y = 0.0
        new_vel_msg.angular.z = float(self.str_angle)

        self.vel_pub.publish(new_vel_msg)


def main(argv):

    rclpy.init(args=None)

    # Retrieve namespace
    node_temp = Node('velinjector_namespace')

    ns = node_temp.get_namespace()

    if ns.endswith('/'):
        ns = ns[:-1]

    node_temp.destroy_node()

    if len(argv) < 5:
        print(
            "Usage: velinjector.py "
            "<csvfile> <time_col> <vel_col> "
            "<steering_angle> <input_type>"
        )
        rclpy.shutdown()
        return

    csvfile = argv[0]
    time_col = argv[1]
    vel_col = argv[2]
    str_angle = float(argv[3])
    input_type = argv[4]

    node = VelInjector(
        ns,
        csvfile,
        time_col,
        vel_col,
        str_angle,
        input_type
    )

    try:

        while rclpy.ok():

            # Process ROS2 callbacks/parameters
            rclpy.spin_once(
                node,
                timeout_sec=0.0
            )

            # ROS2 parameter equivalent of /execute
            if node.has_parameter('execute'):
                execute = node.get_parameter(
                    'execute'
                ).value
            else:
                execute = False

            if execute:

                node.publish()

                if node.next_time == -1:
                    break

                deltaT = node.next_time - node.current_time

                if deltaT > 0:
                    time.sleep(deltaT)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
