#!/usr/bin/env python3
#
# Author: Jonathan Sprinkle
# Copyright (c) 2015 Arizona Board of Regents
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

# This node generates cmd_vel inputs to the vehicle in order to make it move
# around. Use the arrow keys to make the vehicle turn its wheels, move forward,
# or backward. If you want to move forward (but turn left), first press the left
# arrow key, then press up.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import sys
import getopt
import curses


class PrimitiveCmdVel(Node):

    def __init__(self, ns):
        super().__init__('primitiveCmdVel')

        self.ns = ns

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '{0}/cmd_vel'.format(ns),
            1
        )

        self.x = 0.0
        self.z = 0.0

    def publish(self):
        msg = Twist()
        msg.linear.x = self.x
        msg.angular.z = self.z

        self.pub_cmd_vel.publish(msg)


def usage():
    print('primitiveCmdVel -n catvehicle')


def main(argv):
    rclpy.init(args=None)

    ns = 'catvehicle'

    try:
        opts, args = getopt.getopt(
            argv,
            "hn:",
            ["help", "namespace="]
        )
    except getopt.GetoptError:
        usage()
        rclpy.shutdown()
        return

    for opt, arg in opts:
        if opt in ("-h", "--help"):
            usage()
            rclpy.shutdown()
            return
        elif opt in ("-n", "--namespace"):
            ns = arg

    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(False)

    node = PrimitiveCmdVel(ns)

    try:
        while rclpy.ok():

            ch = stdscr.getch()

            node.x = 4.0
            node.z = 0.0

            # Left arrow
            if ch == curses.KEY_LEFT:
                node.z = 1.0

            # Up arrow
            elif ch == curses.KEY_UP:
                node.x = node.x

            # Right arrow
            elif ch == curses.KEY_RIGHT:
                node.z = -1.0

            # Down arrow
            elif ch == curses.KEY_DOWN:
                node.x = -node.x

            else:
                break

            node.publish()

    except KeyboardInterrupt:
        pass

    finally:
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])


