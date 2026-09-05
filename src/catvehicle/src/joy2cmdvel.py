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



import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class Joy2CmdVel(Node):

    def __init__(self):
        super().__init__('joy2cmdvel')

        self.declare_parameter('namespace', 'catvehicle')
        self.declare_parameter('velmax', 2.0)

        self.ns = self.get_parameter('namespace').value
        self.velmax = self.get_parameter('velmax').value

        self.get_logger().info(
            'Startup in namespace {} with max velocity {}'.format(
                self.ns, self.velmax
            )
        )

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.callback,
            10
        )

        self.pub_cmdvel = self.create_publisher(
            Twist,
            '{0}/cmd_vel'.format(self.ns),
            1
        )

        self.x = 0.0
        self.z = 0.0

        # Publish at 100 Hz
        self.timer = self.create_timer(
            0.01,
            self.publish
        )

    def callback(self, data):
        self.x = data.axes[3] * self.velmax
        self.z = -1.0 * data.axes[0] * 0.05

    def publish(self):
        msg_twist = Twist()
        msg_twist.linear.x = self.x
        msg_twist.angular.z = self.z

        self.pub_cmdvel.publish(msg_twist)


def main(args=None):
    rclpy.init(args=args)

    node = Joy2CmdVel()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


