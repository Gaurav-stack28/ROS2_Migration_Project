#!/usr/bin/env python3
import rclpy
import message_filters
import numpy as np
from collections import deque
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64, Float64MultiArray
class DataProcess(Node):
    def __init__(self):
        super().__init__('data_process')
        self.tyre_radius = 0.3671951254
        self.acc_y_deque = deque(maxlen=20)
        self.yaw_old = 0.0
        # Subscribers
        imu_data = message_filters.Subscriber( self, Imu, '/catvehicle/imu' )
        joint_data = message_filters.Subscriber( self, JointState, '/catvehicle/joint_states' )
        steering_data = message_filters.Subscriber( self, Float64, '/catvehicle/ste_angle' )
        # Synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer( [ imu_data, joint_data, steering_data ], queue_size=10, slop=1.0, allow_headerless=True )
        self.sync.registerCallback(self.data_callback)
        # Publishers
        self.pub1 = self.create_publisher( Float64MultiArray, '/SSAE/processed_data', 10 )
        self.pub2 = self.create_publisher( Float64, '/SSAE/deltav', 10 )
        self.pub3 = self.create_publisher( Float64, '/SSAE/yawrate', 10 )
        self.get_logger().info( 'Start data process node ...' )
    def data_callback( self, imu_data, joint_states, steering_data ):
        try:
            # Get sensor data
            yaw_rate = imu_data.angular_velocity.z
            self.yaw_old = yaw_rate
            acc_x = imu_data.linear_acceleration.x
            acc_y_t = imu_data.linear_acceleration.y
            self.acc_y_deque.append(acc_y_t)
            acc_y = np.mean(self.acc_y_deque)
            # Calculate longitudinal velocity
            vel_x = ( joint_states.velocity[0] * self.tyre_radius )
            # Get steering angle
            steering_angle = steering_data.data
            # Create processed output
            processed_data = Float64MultiArray()
            processed_data.data = [ acc_x, acc_y, yaw_rate, vel_x, steering_angle ]
            # Publish
            deltav_msg = Float64()
            deltav_msg.data = steering_angle
            yawrate_msg = Float64()
            yawrate_msg.data = yaw_rate
            self.pub2.publish(deltav_msg)
            self.pub3.publish(yawrate_msg)
            self.pub1.publish(processed_data)
            self.get_logger().info( f'yaw rate: {yaw_rate:.6f}' )
        except IndexError:
            self.get_logger().warning( 'IndexError while processing sensor data' )
def main(args=None):
    rclpy.init(args=args)
    node = DataProcess()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()