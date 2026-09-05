#!/usr/bin/env python

import math
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64, Bool, Float64MultiArray
import matplotlib.pyplot as plt
from collections import deque


class Side_Slip_Angle_Estimation(object):
    def __init__(self):
        # init node
        # ROS 1:
        # rospy.init_node(self.node_name)
        #
        # ROS 2:
        # Node is initialized by rclpy and this class inherits Nodes
        self.node_name = 'SideSlipAngle_Estimation'
        super().__init__(self.node_name)
        self.get_logger().info('start side slip angle estimation node ...')
        # Set process and sensor noise for EKF of dynamics model
        self.process_noise_v_kd = np.array([[0.001],[0.001]])
        self.sensor_noise_v_kd = np.array([[0.0001],[0.0001]])
        self.Q_kd = np.array([[0.01, 0],[0, 0.01]])
        self.R_kd = np.array([[0.005, 0],[0, 0.005]])
        # Set process and sensor noise for EKF of kinematics model
        self.process_noise_v_kk = np.array([[0.001],[0.001]])
        self.sensor_noise_v_kk = np.array([[0.0001],[0.0001]])
        self.Q_kk = np.array([[0.01, 0],[0, 0.01]])
        self.R_kk = np.array([[0.005, 0],[0, 0.005]])
        # Set Constant Parameter of vehicle
        self.C_f = 169265.0
        self.C_r = 249962.5
        self.L_f = 1.55
        self.L_r = 1.05
        self.m_v = 1883.239
        self.I_z = 2529.4827
        # initialisation of EKF
        self.beta = []
        self.v_y = 0
        self.v_x = 0
        self.state_estimate_kd = np.array([[0],[0]])
        self.P_kd = np.array([[0.1, 0],[0, 0.1]])
        self.state_estimate_dd = np.array([[0],[0]])
        self.P_dd = np.array([[0.1, 0],[0, 0.1]])
        # beta smoothing
        self.last_beta_smooth = 0
        self.beta_smooth_deque = deque(maxlen=1)
        # trajectory calculation
        self.last_beta = 0
        self.x0 = 0
        self.y0 = 0
        self.x_buffer = [0]
        self.y_buffer = [0]
        self.psi = 0
        self.last_time = self.get_clock().now()
        # ROS 2 Publishers
        #ROS 1;
        # rospy.Publisher("/SSAE/beta", Float64, queue_size=10)
        # ROS 2:
        # self.create_publisher(Float64, "/SSAE/beta", 10)
        self.pub1 = self.create_publisher(Float64,"/SSAE/beta",10)
        self.pub2 = self.create_publisher(Float64,"/SSAE/x_dist",10)
        self.pub3 = self.create_publisher(Float64,"/SSAE/y_dist",10)
        self.pub4 = self.create_publisher(Float64,"/SSAE/psi",10)
        # ROS 2 Subscriber
        self.sub1 = self.create_subscription(Float64MultiArray,"/SSAE/processed_data",self.EstCallback,10)
        self.get_logger().info('SSAE EKF node initialized successfully.')
    # KINEMATIC EKF
    def ekf_update_k(self,A_k,B_k,C_k,z_k_observation_vector,state_estimate_k,control_vector_k,P_k):
        """ EKF update function of kinematics model State: [v_x, v_y]^T """
        # Predicted state estimate
        state_estimate_k = (np.matmul(A_k, state_estimate_k) + np.matmul(B_k, control_vector_k) + self.process_noise_v_kk)
        # Predicted covarience estimate
        P_k = (np.matmul(np.matmul(A_k, P_k),A_k.T)+ self.Q_kk)
        # reset model if velocity is too small
        if self.v_x < 0.05:
            P_k = np.array([[0.1, 0],[0, 0.1]])
        # Measurement residual
        measurement_residual_y_k = (z_k_observation_vector - (np.matmul(C_k, state_estimate_k) + self.sensor_noise_w_kk))
        # Residual covariance
        S_k = (np.matmul(np.matmul(C_k, P_k),C_k.T) + self.R_kk)
        # Kalman gain
        K_k = np.matmul(np.matmul(P_k, C_k.T),np.linalg.pinv(S_k))
        # update state estimate
        state_estimate_k = (state_estimate_k + np.matmul(K_k,measurement_residual_y_k))
        # update covariance
        P_k = P_k - np.matmul(np.matmul(K_k, C_k),P_k)
        return state_estimate_k, P_k
    # DYNAMIC EKF
    





