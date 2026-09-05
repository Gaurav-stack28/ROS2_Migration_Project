#!/usr/bin/env python3
#
# Author: Jonathan Sprinkle
# Copyright (c) 2015-2016 Arizona Board of Regents
# All rights reserved.
#
# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that the
# above copyright notice and the following two paragraphs appear in
# all copies of this software.
#
# IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY
# FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES
# ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN
# IF THE ARIZONA BOARD OF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#
# THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
# IS ON AN "AS IS" BASIS, AND THE ARIZONA BOARD OF REGENTS HAS NO OBLIGATION
# TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

# This node will publish the path in which you've traveled. The publishing
# happens at 10Hz, unless no connection to odometry exists.
# The published path is appended whenever the odometry differs by at least
# 1m in the L1 norm---i.e., x or y changes by 1m---from the last point in the
# published path.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

import sys
import getopt


class Odom2Path(Node):

    def __init__(self, ns):
        super().__init__('odom2path')

        self.ns = ns

        # Set so that whenever we receive on the odom topic,
        # the callback method is called
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.callback,
            10
        )

        # Setup the state data for the publisher
        self.pub_path = self.create_publisher(
            Path,
            'path',
            10
        )

        # We want to publish when a new odometry data point arrives
        self.publishNow = True

        # Initialize the path message and its header
        self.pathMsg = Path()
        self.pathMsg.header = Header()

        # Initial values are not provided
        self.x = None
        self.y = None

        # Publish at 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.publish
        )

    # This method is called whenever we receive a message
    def callback(self, data):
        # We always publish right away
        self.publishNow = True

        # ROS 2 Header does not contain a sequence number

        # Use ROS 2 clock so simulation time is handled correctly
        self.pathMsg.header.stamp = self.get_clock().now().to_msg()

        # The odometry frame is set here
        self.pathMsg.header.frame_id = '{0}/odom'.format(self.ns)

        # Append a new pose ONLY if the position has moved
        # more than 1 m from its previous position
        if (
            self.x is None
            or abs(self.x - data.pose.pose.position.x) > 1
            or abs(self.y - data.pose.pose.position.y) > 1
        ):
            pose = PoseStamped()

            pose.header.frame_id = '{0}/odom'.format(self.ns)
            pose.header.stamp = self.get_clock().now().to_msg()

            pose.pose.position.x = float(data.pose.pose.position.x)
            pose.pose.position.y = float(data.pose.pose.position.y)

            pose.pose.orientation.x = float(
                data.pose.pose.orientation.x
            )
            pose.pose.orientation.y = float(
                data.pose.pose.orientation.y
            )
            pose.pose.orientation.z = float(
                data.pose.pose.orientation.z
            )
            pose.pose.orientation.w = float(
                data.pose.pose.orientation.w
            )

            self.pathMsg.poses.append(pose)

            self.x = float(data.pose.pose.position.x)
            self.y = float(data.pose.pose.position.y)

    # Publish when a new odometry point has arrived
    def publish(self):
        if self.publishNow:
            self.get_logger().debug(
                'Publishing new path with {} elements.'.format(
                    len(self.pathMsg.poses)
                )
            )

            self.pub_path.publish(self.pathMsg)

            # Wait until a new odometry point arrives
            self.publishNow = False


def usage():
    print('odom2path -n catvehicle')


def main(argv):
    # Acquire the namespace from the command line
    ns = ''

    try:
        opts, args = getopt.getopt(
            argv,
            "hn:",
            ["help", "namespace="]
        )
    except getopt.GetoptError:
        usage()
        return

    for o, a in opts:
        if o in ("-h", "--help"):
            usage()
            return
        elif o in ("-n", "--namespace"):
            ns = a
        else:
            usage()
            return

    rclpy.init(args=None)

    node = Odom2Path(ns)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])

