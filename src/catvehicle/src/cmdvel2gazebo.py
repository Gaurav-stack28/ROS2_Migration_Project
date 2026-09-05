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

# This node converts cmd_vel inputs to the vehicle to the ROS topics that
# are exposed in Gazebo for moving the vehicle in simulation. Notably, the
# inputs to Gazebo are to joints on the wheel, so there is a multiplier of
# 2.8101 that is applied to the joint's velocity whenever we try to move
# so that the output in Gazebo will match the desired input velocity.

#!/usr/bin/env python3

#
# ROS 2 migration of the original ROS 1 cmdvel2gazebo.py
#
# Original behavior preserved:
#   - cmd_vel input
#   - 2.6101 velocity gain
#   - Ackermann steering geometry
#   - wheelbase = 2.62 m
#   - tread = 1.29 m
#   - maximum steering calculation
#   - 0.2 second command timeout
#   - 100 Hz output
#
# ROS 2 ros2_control interface:
#
#   Velocity controllers:
#       /joint1_velocity_controller/commands
#       /joint2_velocity_controller/commands
#
#   Steering controllers:
#       /front_left_steering_position_controller/commands
#       /front_right_steering_position_controller/commands
#

import sys
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class Cmdvel2Gazebo(Node):

    def __init__(self, ns=''):

        super().__init__('cmdvel2gazebo')

        # ------------------------------------------------------------
        # Namespace
        # ------------------------------------------------------------

        self.ns = ns.strip('/')

        if self.ns:
            prefix = '/' + self.ns
        else:
            prefix = ''

        # ------------------------------------------------------------
        # Topics
        # ------------------------------------------------------------

        cmd_vel_topic = prefix + '/cmd_vel'

        rear_left_topic = (
             '/joint1_velocity_controller/commands'
        )

        rear_right_topic = (
             '/joint2_velocity_controller/commands'
        )

        steer_left_topic = (
             '/front_left_steering_position_controller/commands'
        )

        steer_right_topic = (
             '/front_right_steering_position_controller/commands'
        )

        # ------------------------------------------------------------
        # Subscriber
        # ------------------------------------------------------------

        self.subscription = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.callback,
            10
        )

        # ------------------------------------------------------------
        # Publishers
        #
        # ROS 2 JointGroupVelocityController and
        # JointGroupPositionController expect:
        #
        # std_msgs/msg/Float64MultiArray
        #
        # and the topic is /commands rather than /command.
        # ------------------------------------------------------------

        self.pub_steerL = self.create_publisher(
            Float64MultiArray,
            steer_left_topic,
            10
        )

        self.pub_steerR = self.create_publisher(
            Float64MultiArray,
            steer_right_topic,
            10
        )

        self.pub_rearL = self.create_publisher(
            Float64MultiArray,
            rear_left_topic,
            10
        )

        self.pub_rearR = self.create_publisher(
            Float64MultiArray,
            rear_right_topic,
            10
        )

        # ------------------------------------------------------------
        # Initial velocity and steering angle
        # ------------------------------------------------------------

        self.x = 0.0
        self.z = 0.0

        # ------------------------------------------------------------
        # Vehicle geometry
        # ------------------------------------------------------------

        # Car wheelbase (m)
        self.L = 2.62

        # Car tread (m)
        self.T = 1.29

        # ------------------------------------------------------------
        # Dead-man timeout
        # ------------------------------------------------------------

        self.timeout = 0.2

        self.lastMsg = self.get_clock().now()

        # ------------------------------------------------------------
        # Maximum steering calculation
        #
        # Gazebo inside tire maximum steering angle = 0.6 rad
        # ------------------------------------------------------------

        self.maxsteerInside = 0.6

        # tan(maxsteerInside) = wheelbase / radius
        rMax = self.L / math.tan(self.maxsteerInside)

        # Radius of ideal middle tire
        rIdeal = rMax + (self.T / 2.0)

        # tan(angle) = wheelbase / radius
        self.maxsteer = math.atan2(
            self.L,
            rIdeal
        )

        # ------------------------------------------------------------
        # Information messages
        # ------------------------------------------------------------

        self.get_logger().info(
            'Maximum ideal steering angle set to {:.6f} rad'.format(
                self.maxsteer
            )
        )

        self.get_logger().info(
            'cmd_vel input: {}'.format(cmd_vel_topic)
        )

        self.get_logger().info(
            'Rear-left output: {}'.format(rear_left_topic)
        )

        self.get_logger().info(
            'Rear-right output: {}'.format(rear_right_topic)
        )

        self.get_logger().info(
            'Steering-left output: {}'.format(steer_left_topic)
        )

        self.get_logger().info(
            'Steering-right output: {}'.format(steer_right_topic)
        )

        # ------------------------------------------------------------
        # 100 Hz publishing
        #
        # ROS 1:
        #     rospy.Rate(100)
        #
        # ROS 2:
        #     timer period = 0.01 seconds
        # ------------------------------------------------------------

        self.timer = self.create_timer(
            0.01,
            self.publish
        )

    # ================================================================
    # cmd_vel callback
    # ================================================================

    def callback(self, data):

        # ------------------------------------------------------------
        # Original ROS 1 gain factor
        #
        # 2.6101 is the mechanical reduction compensation.
        # ------------------------------------------------------------

        self.x = 2.6101 * data.linear.x

        # ------------------------------------------------------------
        # Constrain ideal steering angle
        # ------------------------------------------------------------

        self.z = max(
            -self.maxsteer,
            min(self.maxsteer, data.angular.z)
        )

        # Update command timestamp
        self.lastMsg = self.get_clock().now()

    # ================================================================
    # Publish commands to Gazebo controllers
    # ================================================================

    def publish(self):

        now = self.get_clock().now()

        elapsed = (
            now - self.lastMsg
        ).nanoseconds / 1e9

        # ------------------------------------------------------------
        # Dead-man timeout
        # ------------------------------------------------------------

        if elapsed > self.timeout:

            self.x = 0.0

            # Stop rear-left wheel
            msgRearL = Float64MultiArray()
            msgRearL.data = [0.0]

            # Stop rear-right wheel
            msgRearR = Float64MultiArray()
            msgRearR.data = [0.0]

            # Keep steering at the last commanded position.
            #
            # This matches the original ROS 1 behavior where
            # self.z was not reset after timeout.
            msgSteerL = Float64MultiArray()
            msgSteerL.data = [self.z]

            msgSteerR = Float64MultiArray()
            msgSteerR.data = [self.z]

            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)

            self.pub_steerL.publish(msgSteerL)
            self.pub_steerR.publish(msgSteerR)

            return

        # ------------------------------------------------------------
        # Turning
        # ------------------------------------------------------------

        if self.z != 0.0:

            T = self.T
            L = self.L

            # Radius of ideal middle tire
            r = L / math.fabs(
                math.tan(self.z)
            )

            # Left and right tire radii
            rL = r - (
                math.copysign(1, self.z) *
                (T / 2.0)
            )

            rR = r + (
                math.copysign(1, self.z) *
                (T / 2.0)
            )

            # --------------------------------------------------------
            # Rear-right wheel
            # --------------------------------------------------------

            msgRearR = Float64MultiArray()

            msgRearR.data = [
                self.x * rR / r
            ]

            # --------------------------------------------------------
            # Rear-left wheel
            # --------------------------------------------------------

            msgRearL = Float64MultiArray()

            msgRearL.data = [
                self.x * rL / r
            ]

            self.pub_rearL.publish(msgRearL)
            self.pub_rearR.publish(msgRearR)

            # --------------------------------------------------------
            # Front-left steering
            # --------------------------------------------------------

            msgSteerL = Float64MultiArray()

            msgSteerL.data = [
                math.atan2(L, rL) *
                math.copysign(1, self.z)
            ]

            self.pub_steerL.publish(msgSteerL)

            # --------------------------------------------------------
            # Front-right steering
            # --------------------------------------------------------

            msgSteerR = Float64MultiArray()

            msgSteerR.data = [
                math.atan2(L, rR) *
                math.copysign(1, self.z)
            ]

            self.pub_steerR.publish(msgSteerR)

        # ------------------------------------------------------------
        # Straight driving
        # ------------------------------------------------------------

        else:

            # Both rear wheels receive the same velocity
            msgRear = Float64MultiArray()

            msgRear.data = [self.x]

            self.pub_rearL.publish(msgRear)
            self.pub_rearR.publish(msgRear)

            # Both steering wheels remain straight
            msgSteer = Float64MultiArray()

            msgSteer.data = [self.z]

            self.pub_steerL.publish(msgSteer)
            self.pub_steerR.publish(msgSteer)


# ====================================================================
# Main
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    # ------------------------------------------------------------
    # Read namespace from command-line argument
    #
    # Example:
    #
    # ros2 run catvehicle cmdvel2gazebo.py -n catvehicle
    # ------------------------------------------------------------

    ns = ''

    if args is None:
        args = sys.argv[1:]

    i = 0

    while i < len(args):

        if args[i] == '-n' and i + 1 < len(args):

            ns = args[i + 1]
            i += 2

        else:

            i += 1

    node = Cmdvel2Gazebo(ns)

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



