#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64, Bool, Float64MultiArray
import matplotlib.pyplot as plt
from collections import deque


class Side_Slip_Angle_Estimation(Node):
    def __init__(self):
        # init node
        self.node_name = 'SideSlipAngle_Estimation'
        super().__init__(self.node_name)

        self.get_logger().info(
            'start side slip angle estimation node ...'
        )

        # ============================================================
        # Set process and sensor noise for EKF of dynamics model
        # ============================================================

        self.process_noise_v_kd = np.array([[0.001],
                                            [0.001]])

        self.sensor_noise_w_kd = np.array([[0.0001],
                                           [0.0001]])

        self.Q_kd = np.array([[0.01, 0],
                              [0, 0.01]])

        self.R_kd = np.array([[0.005, 0],
                              [0, 0.005]])

        # ============================================================
        # Set process and sensor noise for EKF of kinematics model
        # ============================================================

        self.process_noise_v_kk = np.array([[0.1],
                                            [0.1]])

        self.sensor_noise_w_kk = np.array([[0.25],
                                           [0.25]])

        self.Q_kk = np.array([[0.02, 0],
                              [0, 0.02]])

        self.R_kk = np.array([[0.001, 0],
                              [0, 0.001]])

        # ============================================================
        # Set Constant Parameter of vehicle
        # ============================================================

        self.C_f = 169265.0
        self.C_r = 249962.5
        self.L_f = 1.55
        self.L_r = 1.05
        self.m_v = 1883.239
        self.I_z = 2529.4827

        # ============================================================
        # Initialisation of EKF
        # ============================================================

        self.beta = []
        self.v_y = 0
        self.v_x = 0

        self.state_estimate_kd = np.array([[0],
                                           [0]])

        self.P_kd = np.array([[0.1, 0],
                              [0, 0.1]])

        self.state_estimate_dd = np.array([[0],
                                           [0]])

        self.P_dd = np.array([[0.1, 0],
                              [0, 0.1]])

        # ============================================================
        # Beta smoothing
        # ============================================================

        self.last_beta_smooth = 0
        self.beta_smooth_deque = deque(maxlen=1)

        # ============================================================
        # Trajectory calculation
        # ============================================================

        self.last_beta = 0
        self.x0 = 0
        self.y0 = 0

        self.x_buffer = [0]
        self.y_buffer = [0]

        self.psi = 0

        self.last_time = self.get_clock().now()

        # ============================================================
        # ROS2 Publishers
        # ============================================================

        self.pub1 = self.create_publisher(
            Float64,
            "/SSAE/beta",
            10
        )

        self.pub2 = self.create_publisher(
            Float64,
            "/SSAE/x_dist",
            10
        )

        self.pub3 = self.create_publisher(
            Float64,
            "/SSAE/y_dist",
            10
        )

        self.pub4 = self.create_publisher(
            Float64,
            "/SSAE/psi",
            10
        )

        # ============================================================
        # ROS2 Subscriber
        # ============================================================

        self.sub1 = self.create_subscription(
            Float64MultiArray,
            "/SSAE/processed_data",
            self.EstCallback,
            10
        )

    # ================================================================
    # EKF update function of kinematics model
    # ================================================================

    def ekf_update_k(
        self,
        A_k,
        B_k,
        C_k,
        z_k_observation_vector,
        state_estimate_k,
        control_vector_k,
        P_k
    ):
        """
        EKF update function of kinematics model

        :param A_k: A_d or A_k
        :param B_k: B_d or B_k
        :param C_k: measurement matrix
        :param z_k_observation_vector: measurement data directly
                                       from sensor, y_d
        :param state_estimate_k: x_d, state estimation at time k
        :param control_vector_k: u_d, control vector at time k
        :param P_k: covariance estimation at time k

        :return: state estimation [v_x, v_y]^T, covariance
        """

        # Predicted state estimate
        state_estimate_k = (
            np.matmul(A_k, state_estimate_k)
            + np.matmul(B_k, control_vector_k)
            + self.process_noise_v_kk
        )

        # Predicted covariance estimate
        P_k = (
            np.matmul(
                np.matmul(A_k, P_k),
                A_k.T
            )
            + self.Q_kk
        )

        # Reset model if velocity is too small
        if self.v_x < 0.05:
            P_k = np.array([
                [0.1, 0],
                [0, 0.1]
            ])

        # Measurement residual
        measurement_residual_y_k = (
            z_k_observation_vector
            - (
                np.matmul(C_k, state_estimate_k)
                + self.sensor_noise_w_kk
            )
        )

        # Residual covariance
        S_k = (
            np.matmul(
                np.matmul(C_k, P_k),
                C_k.T
            )
            + self.R_kk
        )

        # Kalman gain
        K_k = (
            np.matmul(
                np.matmul(P_k, C_k.T),
                np.linalg.pinv(S_k)
            )
        )

        # Update state estimate
        state_estimate_k = (
            state_estimate_k
            + np.matmul(
                K_k,
                measurement_residual_y_k
            )
        )

        # Update covariance of state estimate
        P_k = (
            P_k
            - np.matmul(
                np.matmul(K_k, C_k),
                P_k
            )
        )

        return state_estimate_k, P_k

    # ================================================================
    # EKF update function of dynamics model
    # ================================================================

    def ekf_update_d(
        self,
        A_k,
        B_k,
        C_k,
        z_k_observation_vector,
        state_estimate_k,
        control_vector_k,
        P_k
    ):
        """
        EKF update function of Dynamics model

        :param A_k: A_dd
        :param B_k: B_dd
        :param C_k: C_dd
        :param z_k_observation_vector: measurement data directly
                                       from sensor, y_d
        :param state_estimate_k: state estimation at time k
        :param control_vector_k: control vector at time k
        :param P_k: covariance estimation at time k

        :return: state estimation [v_y, yaw_rate]^T, covariance
        """

        # Predicted state estimate
        state_estimate_k = (
            np.matmul(A_k, state_estimate_k)
            + (B_k * control_vector_k)
            + self.process_noise_v_kd
        )

        # Predicted covariance estimate
        P_k = (
            np.matmul(
                np.matmul(A_k, P_k),
                A_k.T
            )
            + self.Q_kd
        )

        # Reset model if velocity is too small
        if self.v_x < 0.05:
            P_k = np.array([
                [0.1, 0],
                [0, 0.1]
            ])

        # Measurement residual
        measurement_residual_y_k = (
            z_k_observation_vector
            - (
                np.matmul(C_k, state_estimate_k)
                + self.sensor_noise_w_kd
            )
        )

        # Residual covariance
        S_k = (
            np.matmul(
                np.matmul(C_k, P_k),
                C_k.T
            )
            + self.R_kd
        )

        # Kalman gain
        K_k = (
            np.matmul(
                np.matmul(P_k, C_k.T),
                np.linalg.pinv(S_k)
            )
        )

        # Update state estimate
        state_estimate_k = (
            state_estimate_k
            + np.matmul(
                K_k,
                measurement_residual_y_k
            )
        )

        # Update covariance of state estimate
        P_k = (
            P_k
            - np.matmul(
                np.matmul(K_k, C_k),
                P_k
            )
        )

        return state_estimate_k, P_k

    # ================================================================
    # Kinematics parameters
    # ================================================================

    def get_kinematics_param(self, data, Delta_T):
        """
        Calculate discretized kinematics system matrix,
        which is used in EKF.

        :param data: input data,
                     acc_x, acc_y, steering_angle, yaw_rate, vel_x
        :param Delta_T: time interval

        :return: system matrix
        """

        # Get the required data
        acc_x, acc_y, steering_angle, yaw_rate, vel_x = data

        # Calculate discretized transformation matrix A_d

        A_k11 = (
            (Delta_T ** 4 * yaw_rate ** 4) / 24
            - (Delta_T ** 2 * yaw_rate ** 2) / 2
            + 1
        )

        A_k12 = (
            (Delta_T ** 3 * yaw_rate ** 3) / 6
            - Delta_T * yaw_rate
        )

        A_k21 = (
            Delta_T * yaw_rate
            - (Delta_T ** 3 * yaw_rate ** 3) / 6
        )

        A_k22 = (
            (Delta_T ** 4 * yaw_rate ** 4) / 24
            - (Delta_T ** 2 * yaw_rate ** 2) / 2
            + 1
        )

        A_kd = np.array([
            [A_k11, A_k12],
            [A_k21, A_k22]
        ])

        # Calculate discretized B_d matrix

        B_k11 = np.sin(Delta_T * yaw_rate) / yaw_rate
        B_k12 = np.cos(Delta_T * yaw_rate) / yaw_rate
        B_k21 = -np.cos(Delta_T * yaw_rate) / yaw_rate
        B_k22 = np.sin(Delta_T * yaw_rate) / yaw_rate

        B_kd = np.array([
            [B_k11, B_k12],
            [B_k21, B_k22]
        ])

        # Calculate measurement matrix

        C_kd = np.array([
            [1, 0],
            [0, 1]
        ])

        # Set measurement vector

        y_kd = np.array([
            [vel_x],
            [self.v_y]
        ])

        # Set control vector

        u_kd = np.array([
            [acc_x],
            [acc_y]
        ])

        return A_kd, B_kd, C_kd, y_kd, u_kd

    # ================================================================
    # Dynamics parameters
    # ================================================================

    def get_dynamics_param(self, processed_data, dt):
        """
        Calculate discretized dynamics system matrix,
        which is used in EKF.

        :param processed_data: input data,
                             acc_x, acc_y, steering_angle,
                             yaw_rate, vel_x
        :param dt: time interval

        :return: system matrix
        """

        # Get data from topic
        acc_x, acc_y, steering_angle, yaw_rate, vel_x = processed_data

        # Prevent vel_x equal to 0
        if vel_x < 0.01:
            vel_x = 0.01

        # Calculate transformation matrix

        A_d = np.array([
            [
                (-self.C_f - self.C_r) /
                (self.m_v * vel_x),

                (
                    -vel_x
                    - (
                        self.L_f * self.C_f
                        - self.L_r * self.C_r
                    ) /
                    (self.m_v * vel_x)
                )
            ],

            [
                (
                    -self.L_f * self.C_f
                    + self.L_r * self.C_r
                ) /
                (self.I_z * vel_x),

                (
                    -(self.L_f ** 2) * self.C_f
                    - (self.L_r ** 2) * self.C_r
                ) /
                (self.I_z * vel_x)
            ]
        ])

        C_d = np.array([
            [
                (-self.C_f - self.C_r) /
                (self.m_v * vel_x),

                -(
                    self.L_f * self.C_f
                    - self.L_r * self.C_r
                ) /
                (self.m_v * vel_x)
            ],

            [0, 1]
        ])

        D_d = np.array([
            [self.C_f / self.m_v],
            [0]
        ])

        # Get the measurement output vector

        y_d = (
            np.array([
                [acc_y, yaw_rate]
            ]).T
            - D_d * steering_angle
        )

        # Get discretized linear continuous dynamics model

        A_dd = (
            np.eye(2)
            + A_d * dt
            + 0.5 * A_d ** 2 * dt ** 2
            + 1 / 6 * A_d ** 3 * dt ** 3
            + 1 / 24 * A_d ** 4 * dt ** 4
        )

        # ============================================================
        # B_dd_0
        # Keep the original mathematical expression intact.
        # ============================================================

        B_dd_0 = (
            (
                207319096137250381803524174900331
                * vel_x
                * np.exp(
                    (
                        512 * dt *
                        (
                            (
                                1122942022011136646878644803407220262924012636719516516972657
                                - 79924283481264903055262051827173378911198266909432741888
                                * vel_x ** 2
                            ) ** (1 / 2)
                            - 11075153569676092564949274424089
                        )
                    )
                    /
                    (
                        23035455126361153533724908322259
                        * vel_x
                    )
                )
                *
                (
                    179903239029935360
                    *
                    (
                        (
                            1122942022011136646878644803407220262924012636719516516972657
                            - 79924283481264903055262051827173378911198266909432741888
                            * vel_x ** 2
                        ) ** (1 / 2)
                        - 9340524050321149179789466627555870280588317023
                        * vel_x ** 2
                        + 191136591265856894831378235498059356636140569856
                    )
                )
            )
            /
            (
                18446744073709551616
                *
                (
                    (
                        1122942022011136646878644803407220262924012636719516516972657
                        - 79924283481264903055262051827173378911198266909432741888
                        * vel_x ** 2
                    ) ** (1 / 2)
                )
                *
                (
                    (
                        1122942022011136646878644803407220262924012636719516516972657
                        - 79924283481264903055262051827173378911198266909432741888
                        * vel_x ** 2
                    ) ** (1 / 2)
                    - 11075153569676092564949274424089
                )
            )
            -
            (
                207319096137250381803524174900331
                * vel_x
                * np.exp(
                    -(
                        512 * dt *
                        (
                            (
                                1122942022011136646878644803407220262924012636719516516972657
                                - 79924283481264903055262051827173378911198266909432741888
                                * vel_x ** 2
                            ) ** (1 / 2)
                            + 11075153569676092564949274424089
                        )
                    )
                    /
                    (
                        23035455126361153533724908322259
                        * vel_x
                    )
                )
                *
                (
                    179903239029935360
                    *
                    (
                        (
                            1122942022011136646878644803407220262924012636719516516972657
                            - 79924283481264903055262051827173378911198266909432741888
                            * vel_x ** 2
                        ) ** (1 / 2)
                        + 9340524050321149179789466627555870280588317023
                        * vel_x ** 2
                        - 191136591265856894831378235498059356636140569856
                    )
                )
            )
            /
            (
                18446744073709551616
                *
                (
                    (
                        1122942022011136646878644803407220262924012636719516516972657
                        - 79924283481264903055262051827173378911198266909432741888
                        * vel_x ** 2
                    ) ** (1 / 2)
                )
                *
                (
                    (
                        1122942022011136646878644803407220262924012636719516516972657
                        - 79924283481264903055262051827173378911198266909432741888
                        * vel_x ** 2
                    ) ** (1 / 2)
                    + 11075153569676092564949274424089
                )
            )
        )

        # ============================================================
        # B_dd_1
        # ============================================================

        B_dd_1 = (
            (
                207319096137250381803524174900331
                * vel_x
                * np.exp(
                    (
                        512 * dt *
                        (
                            (
                                1122942022011136646878644803407220262924012636719516516972657
                                - 79924283481264903055262051827173378911198266909432741888
                                * vel_x ** 2
                            ) ** (1 / 2)
                            - 11075153569676092564949274424089
                        )
                    )
                    /
                    (
                        23035455126361153533724908322259
                        * vel_x
                    )
                )
                *
                (
                    405484675648197
                    *
                    (
                        (
                            1122942022011136646878644803407220262924012636719516516972657
                            - 79924283481264903055262051827173378911198266909432741888
                            * vel_x ** 2
                        ) ** (1 / 2)
                        - 429062881814690501191506705002666862454131773
                    )
                )
            )
            /
            (
                36028797018963968
                *
                (
                    (
                        1122942022011136646878644803407220262924012636719516516972657
                        - 79924283481264903055262051827173378911198266909432741888
                        * vel_x ** 2
                    ) ** (1 / 2)
                )
                *
                (
                    (
                        1122942022011136646878644803407220262924012636719516516972657
                        - 79924283481264903055262051827173378911198266909432741888
                        * vel_x ** 2
                    ) ** (1 / 2)
                    - 11075153569676092564949274424089
                )
            )
            -
            (
                207319096137250381803524174900331
                * vel_x
                * np.exp(
                    -(
                        512 * dt *
                        (
                            (
                                1122942022011136646878644803407220262924012636719516516972657
                                - 79924283481264903055262051827173378911198266909432741888
                                * vel_x ** 2
                            ) ** (1 / 2)
                            + 11075153569676092564949274424089
                        )
                    )
                    /
                    (
                        23035455126361153533724908322259
                        * vel_x
                    )
                )
                *
                (
                    405484675648197
                    *
                    (
                        (
                            1122942022011136646878644803407220262924012636719516516972657
                            - 79924283481264903055262051827173378911198266909432741888
                            * vel_x ** 2
                        ) ** (1 / 2)
                        + 429062881814690501191506705002666862454131773
                    )
                )
            )
            /
            (
                36028797018963968
                *
                (
                    (
                        1122942022011136646878644803407220262924012636719516516972657
                        - 79924283481264903055262051827173378911198266909432741888
                        * vel_x ** 2
                    ) ** (1 / 2)
                )
                *
                (
                    (
                        1122942022011136646878644803407220262924012636719516516972657
                        - 79924283481264903055262051827173378911198266909432741888
                        * vel_x ** 2
                    ) ** (1 / 2)
                    + 11075153569676092564949274424089
                )
            )
        )

        B_dd = np.array([
            [B_dd_0],
            [B_dd_1]
        ])

        # Control vector
        u_d = steering_angle

        return A_dd, B_dd, C_d, D_d, y_d, u_d

    # ================================================================
    # Estimation callback
    # ================================================================

    def EstCallback(self, processed_data):

        # Get data from topic
        accx, accy, yaw_rate, self.v_x, deltav = processed_data.data

        data = np.array([
            accx,
            accy,
            deltav,
            yaw_rate,
            self.v_x
        ])

        Delta_T = 0.1

        # ============================================================
        # Reset model when yaw rate is too small
        # ============================================================

        if np.abs(yaw_rate) < 0.01:

            Beta = self.last_beta

            self.state_estimate_kd = np.array([
                [self.v_x],
                [0]
            ])

            self.state_estimate_dd = np.array([
                [0],
                [0]
            ])

            self.P_kd = np.array([
                [0.1, 0],
                [0, 0.1]
            ])

            self.P_dd = np.array([
                [0.1, 0],
                [0, 0.1]
            ])

            print('beta', Beta)

            msg_beta = Float64()
            msg_beta.data = float(Beta)
            self.pub1.publish(msg_beta)

        # ============================================================
        # EKF estimation
        # ============================================================

        else:

            # Get kinematics model param
            A_kd, B_kd, C_kd, y_kd, u_kd = (
                self.get_kinematics_param(
                    data,
                    Delta_T
                )
            )

            # EKF update function of kinematics model

            optimal_state_estimate_kd, self.P_kd = (
                self.ekf_update_k(
                    A_kd,
                    B_kd,
                    C_kd,
                    y_kd,
                    self.state_estimate_kd,
                    u_kd,
                    self.P_kd
                )
            )

            self.state_estimate_dd[0, 0] = (
                optimal_state_estimate_kd[1]
            )

            # Get system matrix of dynamics model

            A_dd, B_dd, C_dd, D_dd, y_dd, u_dd = (
                self.get_dynamics_param(
                    data,
                    Delta_T
                )
            )

            # EKF update function of dynamics model

            optimal_state_estimate_dd, self.P_dd = (
                self.ekf_update_d(
                    A_dd,
                    B_dd,
                    C_dd,
                    y_dd,
                    self.state_estimate_dd,
                    u_dd,
                    self.P_dd
                )
            )

            # Update v_y for kinematics model in next time step

            self.v_y = optimal_state_estimate_dd[0, 0]

            # State update

            self.state_estimate_kd[0, 0] = (
                optimal_state_estimate_kd[0, 0]
            )

            self.state_estimate_kd[1, 0] = (
                optimal_state_estimate_dd[0, 0]
            )

            self.state_estimate_dd[1, 0] = (
                optimal_state_estimate_dd[1, 0]
            )

            self.v_y = optimal_state_estimate_dd[0, 0]

            # ========================================================
            # Calculate side slip angle
            # ========================================================

            Beta = np.arctan(
                optimal_state_estimate_dd[0, 0]
                /
                optimal_state_estimate_kd[0, 0]
            )

            # Delete over range noise

            if abs(Beta) > 0.6:
                Beta = self.last_beta

            # Record values

            self.beta.append(Beta)

            msg_beta = Float64()
            msg_beta.data = float(Beta)
            self.pub1.publish(msg_beta)

            self.get_logger().info(
                'Beta = {}'.format(Beta)
            )

            self.last_beta = Beta

            # ========================================================
            # Beta smoothing
            # ========================================================
            #
            # alpha = 0.2
            # Beta_t = alpha * Beta + \
            #          (1-alpha) * self.last_beta_smooth
            #
            # self.beta_smooth_deque.append(Beta_t)
            #
            # Beta = np.mean(self.beta_smooth_deque)
            #
            # msg_beta = Float64()
            # msg_beta.data = float(Beta)
            # self.pub1.publish(msg_beta)
            #
            # self.get_logger().info(
            #     'Beta = {}'.format(Beta)
            # )
            #
            # self.last_beta = Beta

        # ============================================================
        # Calculate position and trajectory
        # ============================================================

        self.psi = (
            self.psi
            + data[3] * Delta_T
        )

        self.x0 = (
            self.x0
            + Delta_T
            * data[4]
            * np.cos(Beta + self.psi)
        )

        self.y0 = (
            self.y0
            + Delta_T
            * data[4]
            * np.sin(Beta + self.psi)
        )

        self.x_buffer.append(self.x0)
        self.y_buffer.append(self.y0)

        # ============================================================
        # Publish trajectory data
        # ============================================================

        msg_x = Float64()
        msg_x.data = float(self.x0)
        self.pub2.publish(msg_x)

        msg_y = Float64()
        msg_y.data = float(self.y0)
        self.pub3.publish(msg_y)

        msg_psi = Float64()
        msg_psi.data = float(self.psi)
        self.pub4.publish(msg_psi)

        # ============================================================
        # Store test data as txt file for Matlab
        # ============================================================

        with open("x0.txt", "a") as f:
            f.write('\r')
            f.write(str(self.x0))

        with open("y0.txt", "a") as f:
            f.write('\r')
            f.write(str(self.y0))

        with open("beta.txt", "a") as f:
            f.write('\r')
            f.write(str(Beta))

        with open("vy.txt", "a") as f:
            f.write('\r')
            f.write(str(self.v_y))

        with open("deltav.txt", "a") as f:
            f.write('\r')
            f.write(str(deltav))


# ====================================================================
# ROS2 main
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = Side_Slip_Angle_Estimation()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()