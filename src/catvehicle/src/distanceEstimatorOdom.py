#!/usr/bin/env python3

# Taken from http://answers.ros.org/question/203270/how-do-i-get-the-distancies-between-frames-in-openni_tracker/
# Otherwise, author Jonathan Sprinkle
# Copyright (c) 2016 Arizona Board of Regents

import rclpy
from rclpy.node import Node

import tf2_ros

from std_msgs.msg import Float32
import geometry_msgs.msg

from numpy import (array, dot, arccos, arctan2)
from numpy.linalg import norm


class DistanceEstimatorOdom(Node):

    def __init__(self):
        super().__init__('distanceEstimatorOdom')

        # Get the name of the follower vehicle
        self.follower = self.declare_parameter(
            'follower', 'catvehicle'
        ).value

        self.leader = self.declare_parameter(
            'leader', 'car1'
        ).value

        self.disttopicname = self.declare_parameter(
            'dist_topic', '/distanceEstimator/dist'
        ).value

        self.angletopicname = self.declare_parameter(
            'angle_topic', '/distanceEstimator/angle'
        ).value

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        # Publishers
        self.publisherDist = self.create_publisher(
            Float32,
            self.disttopicname,
            1
        )

        self.publisherAngle = self.create_publisher(
            Float32,
            self.angletopicname,
            1
        )

        # Run at 75 Hz
        self.timer = self.create_timer(
            1.0 / 75.0,
            self.update
        )

    def update(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                '{0}/front_laser_link'.format(self.follower),
                '{0}/back_left_marker_link'.format(self.leader),
                rclpy.time.Time()
            )

            trans = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ]

            rot = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]

            # Keep original distance calculation
            dist = norm(trans[0:1])

            self.get_logger().debug(
                "Translation: {0}, Rotation: {1}".format(
                    trans,
                    rot
                )
            )

            # Use arctan
            angle = arctan2(
                trans[1],
                trans[0]
            )

            self.get_logger().debug(
                "Distance, angle between the points is = ({0:f},{1:f})".format(
                    dist,
                    angle
                )
            )

            # Publish distance
            dist_msg = Float32()
            dist_msg.data = float(dist)

            # Publish angle
            angle_msg = Float32()
            angle_msg.data = float(angle)

            self.publisherDist.publish(dist_msg)
            self.publisherAngle.publish(angle_msg)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            # Same behavior as the ROS 1 version:
            # simply wait for TF to become available.
            return


def main(args=None):

    rclpy.init(args=args)

    node = DistanceEstimatorOdom()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()