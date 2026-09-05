#!/usr/bin/env python3
# rosrun catvehicle lane_detection_kp.py

import roslib
import cv2
import numpy
import scipy.signal
import scipy.optimize as spo
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from scipy.ndimage import gaussian_filter1d
from matplotlib import pyplot
import matplotlib.pyplot as plt
from pyclothoids import Clothoid
import camera_transformation as ct
import message_filters
from message_filters import TimeSynchronizer, Subscriber
from collections import deque
import sys
import os

print('here')

class lane_detection_node(object):
    #lateral_distance = 0
    def __init__(self):
        self.node_name = 'vehicle_control'
        rospy.init_node(self.node_name)

        self.r = rospy.Rate(150)
        self.lateral_distance = 0
        self.w_lateral = 0
        self.beta_old = 0
        self.yaw_rate_old = 0
        self.steering_angle_last = 0
        self.steering_angle_2ndlast = 0
        self.steering_angle_deque = deque(maxlen=3)
        #self.steering_angle_smoothing = 0
        self.integrator = 0
        self.yaw_angular_acc_normal = 0
        self.img_width = 800
        self.img_height = 800  # kameraaufloesung
        self.cc = 1.3962634  # bildebene groesse
        # creating class object
        self.Cam = ct.Camera(self.img_width, self.img_height, 0, 1.75, 0.75, self.cc, self.cc, 50, 50)
        # Subscribers
        self.image_get = rospy.Subscriber("/catvehicle/camera_left/image_raw_left", Image, self.camera_callback)
        beta_get = message_filters.Subscriber("/SSAE/beta", Float64)
        yawrate_get = message_filters.Subscriber("/SSAE/yawrate", Float64)
        control_data = message_filters.ApproximateTimeSynchronizer([beta_get, yawrate_get], 10, 1, allow_headerless=True)
        control_data.registerCallback(self.test_callback)

        self.cmd_vel = rospy.Publisher('/catvehicle/cmd_vel', Twist, queue_size=10)
        self.move_cmd = Twist()
        # Publishers
        self.pub_co = rospy.Publisher('/catvehicle/curvature_opposing', Float64, queue_size=10)
        self.pub_cl = rospy.Publisher('/catvehicle/curvature_left', Float64, queue_size=10)
        self.pub_cr = rospy.Publisher('/catvehicle/curvature_right', Float64, queue_size=10)
        self.pub3 = rospy.Publisher("/catvehicle/ste_angle", Float64, queue_size=10)
        # rospy.spin()

    def display_image(self, img):
        #cv2.line(img, (400,477),(400,800), (0, 0, 255), 7)
        #cv2.rectangle(img, (450, 540), (650,550), (155, 255, 200), 4)
        #cv2.rectangle(img, (220, 540), (420, 550), (155, 255, 200), 4)
        #cv2.rectangle(img, (0, 540), (220, 550), (155, 255, 200), 4)
        cv2.imshow('Kamera', img)
        cv2.waitKey(2)

    def find_lane_start(self, img):
        # the image is divided into 3 sections in which the start of the markings is searched for
        box_r = img[540:650, 380:630].sum(axis=0)
        box_l = img[540:650, 220:420].sum(axis=0)
        box_o = img[540:650, 0:250].sum(axis=0)
        sum_axisr = box_r.sum(axis=0) + 0.1
        sum_axisl = box_l.sum(axis=0) + 0.1
        sum_axiso = box_o.sum(axis=0) + 0.1

        b = numpy.arange(0, 250)

        # the focal point in the area is the beginning of the markings
        a = numpy.arange(0, 200)

        weight_r = (box_r * b).sum() / sum_axisr + 380
        weight_l = (box_l * a).sum() / sum_axiso + 220
        weight_o = (box_o * b).sum() / sum_axisl

        return (round(weight_o), round(weight_l), round(weight_r), 545)

    def sliding_windows(self, img, lstx, starty, box_height, box_width):
        # anfangsparameter
        pos_x = lstx
        pos_y = starty
        a = numpy.arange(0, box_width * 2)
        lane_x = []
        lane_y = []

        # erste box
        box = img[pos_y - box_height:pos_y, pos_x - box_width:pos_x + box_width].sum(axis=0)
        # taking the first box weight to starting box
        if box.sum() > 500:
            weight = (box * a).sum() / box.sum(axis=0)
            diff = round(weight - box_width)
        else:
            diff = 0

        a = numpy.arange(0, box_width * 2)
        # second box starts by diff in position x
        # for height it increases by height value
        pos_x += diff
        pos_y -= box_height
        lane_x.append(pos_x)
        lane_y.append(800 - pos_y)

        box = img[pos_y - box_height:pos_y, pos_x - box_width:pos_x + box_width].sum(axis=0)

        while box.sum() > 500:
            # for the second box
            weight = (box * a).sum() / box.sum(axis=0)
            diff = round(weight - box_width)

            if pos_y > 390:
                box_height = 3
                box_width = 9
                a = numpy.arange(0, box_width * 2)

            # center point for second box
            pos_x += diff
            box = img[pos_y - box_height:pos_y, pos_x - box_width:pos_x + box_width].sum(axis=0)

            # appending new points in lane as x and y coordinates
            lane_x.append(pos_x)
            lane_y.append(800 - pos_y)

            # position for new box
            pos_x += (pos_x - lane_x[-2])
            pos_y -= box_height

            # putting second box in new position
            box = img[pos_y - box_height:pos_y, pos_x - box_width:pos_x + box_width].sum(axis=0)


        lane_x = numpy.array(lane_x)
        lane_y = numpy.array(lane_y)

        return lane_x, lane_y

    def interpolate_mid_lane(self, lx, ly, rx, ry, ox, oy):
        # cut the right lane and the opposite lane to the same length
        diff = len(oy) - len(ry)
        if diff > 0:
            oy = oy[0:-diff - 1]
            ox = ox[0:-diff - 1]
        elif diff < 0:
            ry = ry[0:diff - 1]
            rx = rx[0:diff - 1]
        # if the center line (ly) is shorter than the other marks
        diff2 = len(ry) - len(ly)
        # then fill in the missing points of the ml with the average value of the other two lanes
        if diff2 > 0:
            ly = numpy.append(ly, ry[-diff2:])
            lx = numpy.append(lx, (ox[-diff2:] + rx[-diff2:]) / 2)

        return lx, ly, rx, ry, ox, oy

    def lsq_error(self, param, lx, ly):
        ast, aend, ex = param
        lx = numpy.array(lx)
        ly = numpy.array(ly)
        # creating clothoids for new points
        clothoid0 = Clothoid.G1Hermite(lx[0], ly[0], ast, ex, ly[-1], aend)
        # determining the values of clothoids
        cl = [clothoid0.X((i - ly[0]) * (clothoid0.length / (ly[-1] - ly[0]))) for i in ly]

        # least square distance between points annd clothoids
        diff = numpy.power(cl - lx, 2)
        score = diff.sum()

        return score

    def fit_clothoid(self, lx, ly):
        # estimation the marking angles between starting and last points
        dy = ly[-1] - ly[-5]
        dx = lx[-5] - lx[-1]
        ang_end_guess = numpy.arctan(dx / dy) + numpy.pi / 2
        dy = ly[5] - ly[0]
        dx = lx[0] - lx[5]
        ang_st_guess = numpy.arctan(dx / dy) / 2 + numpy.pi / 2
        # parameters to be adjuste like start angle, end angle, end point
        parameters = numpy.array([ang_st_guess, ang_end_guess, lx[-1]])
        # minimizing the lsq_error function
        result = spo.minimize(self.lsq_error, parameters, args=(lx, ly), tol=0.05, options={'maxiter': 20})
        # parameters for fitted clothoids
        ang_start, ang_end, x_end = result.x
        clothoid0 = Clothoid.G1Hermite(lx[0], ly[0], ang_start, x_end, ly[-1], ang_end)

        # curvature of the clothoid
        curvature = []
        for s in ly:
            curvature.append(clothoid0.ThetaD(s))
        # returns curvature and x and y coordinates of the clothoid
        return numpy.array(curvature), clothoid0.SampleXY(ly.size)[0], clothoid0.SampleXY(ly.size)[1]

    def draw_lines_on_input(self, img, lx, ly):
        for e in range(len(ly) - 1):
            img = cv2.line(img, (int(lx[e]), int(ly[e])), (int(lx[e + 1]), int(ly[e + 1])), (0, 255, 255), 7)

        return img

    def draw_lines_on_input2(self, img, lx, ly):
        for e in range(len(ly) - 1):
            img = cv2.line(img, (int(lx[e]), int(ly[e])), (int(lx[e + 1]), int(ly[e + 1])), (255, 255, 255), 7)

        return img

    # Implementing lateral control by using estimated ssbeta and yawrate

    def vehicle_control(self):
        # defining the required parameters for lateral control
        C_f = 169265.0
        C_r = 249962.5
        L_f = 1.55
        L_r = 1.05
        m_v = 1883.239
        kp = 73.93
        ki = 15
        kd = 2
        vel = 10
        del_t = 0.1
        I_z = 2529.4827
        llo= 5.3333
        wltr = 0

        Integrator = 0

        # condition for yawrate

        print('lateral dist', self.lateral_distance)

        # the lateral distance error (difference between central line and vehicle orientation line)
        lateral_control_err = numpy.arctan(wltr / llo) - numpy.arctan(self.lateral_distance / llo)

        # defining the proportional gain effect in algorithm
        Proportional = kp * lateral_control_err

        # defining the integral gain effect in algorithm
        Integrator = Integrator + ki * lateral_control_err * del_t
        #self.integrator = Integrator

        # defining the derivative gain effect in algorithm
        Derivative = (-1) * kd * self.yaw_rate_old / del_t

        # defining the combine effect
        yaw_angular_acc_normal = Proportional + Integrator + Derivative
        #self.integrator = Integrator

        # Linearizing the lateral control dynamics
        # Linearizing Rear Lateral Force
        S_h = C_r * (self.beta_old + (L_r / vel) * self.yaw_rate_old)


        # Linearizing Front Lateral Force
        S_v = (yaw_angular_acc_normal * I_z + S_h * L_r) / L_f

        # Finally the steering angle by algorithm
        Steering_angle = (S_v / C_f) - self.beta_old + (L_f * self.yaw_rate_old / vel)



        if Steering_angle > 0.05:
            Steering_angle = 0.05


        elif Steering_angle < -0.05:
            Steering_angle = -0.05

        # smoothing the steering value for smooth curve
        alpha = 0.2
        steering_val = 0.2 * Steering_angle + 0.4 * self.steering_angle_last + 0.4 * self.steering_angle_2ndlast
        self.steering_angle_deque.append(steering_val)
        steering_angle_smoothing = steering_val
        self.steering_angle_last = steering_angle_smoothing
        self.steering_angle_2ndlast = Steering_angle
        print('dque= ', steering_val, self.steering_angle_deque, steering_angle_smoothing)


        return steering_angle_smoothing

    def move_vehicle(self, lateral_distance, vel):
        # obtaining the steering angle from vehicle control function
        stervalue = self.vehicle_control()
        # publishing steering value
        self.move_cmd.angular.z = stervalue
        if numpy.absolute(stervalue) > 0.02:
            self.move_cmd.linear.x = 10
        else:
            self.move_cmd.linear.x = vel

        print('velo', self.move_cmd.linear.x)
        self.cmd_vel.publish(self.move_cmd)
        print('stervalue', stervalue)
        self.pub3.publish(stervalue)

    def camera_callback(self, image):

        try:
            # calling image data
            image_data = image
            input_image = numpy.frombuffer(image_data.data, dtype=numpy.uint8).reshape(image_data.height,
                                                                                       image_data.width,
                                                                                       -1)

            # changing image from color to gray
            image = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY)

            # Region of interest
            image[0:-1][555:800] = 0
            image[0:-1][0:395] = 0

            # thresholding for edge detection
            ret, threshold_image = cv2.threshold(image, 220, 255, cv2.THRESH_BINARY)

            # using filter for edge detection
            edge_image = cv2.GaussianBlur(image, (5, 5), 0)

            # canny edge detection
            edge_image = cv2.Canny(edge_image, 200, 250)

            # combining the image
            combined_image = edge_image + threshold_image

            # removing the edges caused by region of interest
            combined_image[0:-1][390:412] = 0
            combined_image[0:-1][550:560] = 0

            # copying image for bird's eye transformation
            combined_image_bv = combined_image

            # find lane starting points
            lane_opposing_start, lane_left_start, lane_right_start, start_y = self.find_lane_start(combined_image_bv)

            # sliding window algorithm for left lanes
            lane_points_left_x_px, lane_points_left_y_px = self.sliding_windows(combined_image_bv, lane_left_start,
                                                                                start_y, 4, 10)
            # sliding window algorithm for right lane
            lane_points_right_x_px, lane_points_right_y_px = self.sliding_windows(combined_image_bv, lane_right_start,
                                                                                  start_y, 4, 10)
            # sliding window algorithm for extreme left lane
            lane_points_opposing_x_px, lane_points_opposing_y_px = self.sliding_windows(combined_image_bv,
                                                                                        lane_opposing_start,
                                                                                        start_y, 4, 10)

            # changing above points in meter from bird's eye transformation
            lane_points_left_x_m, lane_points_left_y_m = self.Cam.cam_coordinates_to_plane_m(lane_points_left_x_px,
                                                                                             lane_points_left_y_px)
            lane_points_right_x_m, lane_points_right_y_m = self.Cam.cam_coordinates_to_plane_m(lane_points_right_x_px,
                                                                                               lane_points_right_y_px)
            lane_points_opposing_x_m, lane_points_opposing_y_m = self.Cam.cam_coordinates_to_plane_m(
                lane_points_opposing_x_px,
                lane_points_opposing_y_px)

            # interpolation for mid points
            lane_points_left_x_m, lane_points_left_y_m, lane_points_right_x_m, lane_points_right_y_m, lane_points_opposing_x_m, \
            lane_points_opposing_y_m = self.interpolate_mid_lane(lane_points_left_x_m, lane_points_left_y_m,
                                                                 lane_points_right_x_m,
                                                                 lane_points_right_y_m, lane_points_opposing_x_m,
                                                                 lane_points_opposing_y_m)
            lane_cx = (lane_points_right_x_m + lane_points_left_x_m) / 2
            lane_cy = (lane_points_left_y_m + lane_points_right_y_m) / 2

            if lane_points_opposing_y_m.size > 20 and lane_points_left_y_m.size > 20 and lane_points_right_y_m.size > 20:
                # calling clothoid function
                curvature_left, clothoid_x_left, clothoid_y_left = self.fit_clothoid(lane_points_left_x_m[10:],
                                                                                     lane_points_left_y_m[10:])
                curvature_right, clothoid_x_right, clothoid_y_right = self.fit_clothoid(lane_points_right_x_m[10:],
                                                                                        lane_points_right_y_m[10:])
                curvature_opposing, clothoid_x_opposing, clothoid_y_opposing = self.fit_clothoid(
                    lane_points_opposing_x_m[10:],
                    lane_points_opposing_y_m[10:])
                curvature_center, clothoid_x_center, clothoid_y_center = self.fit_clothoid(lane_cx[10:], lane_cy[10:])

                # putting detected lines in image
                cl_x_left_kp, cl_y_left_kp = self.Cam.plane_m_to_cam_px(clothoid_x_left, clothoid_y_left)
                cl_x_right_kp, cl_y_right_kp = self.Cam.plane_m_to_cam_px(clothoid_x_right, clothoid_y_right)
                cl_x_opposing_kp, cl_y_opposing_kp = self.Cam.plane_m_to_cam_px(clothoid_x_opposing,
                                                                                clothoid_y_opposing)
                cl_x_center_kp, cl_y_center_kp = self.Cam.plane_m_to_cam_px(clothoid_x_center, clothoid_y_center)

                cl_kp_img = numpy.zeros((800, 800, 3), numpy.uint8)

                cl_kp_img = self.draw_lines_on_input(cl_kp_img, cl_x_left_kp, cl_y_left_kp)
                cl_kp_img = self.draw_lines_on_input(cl_kp_img, cl_x_right_kp, cl_y_right_kp)
                cl_kp_img = self.draw_lines_on_input(cl_kp_img, cl_x_opposing_kp, cl_y_opposing_kp)
                cl_kp_img = self.draw_lines_on_input(cl_kp_img, cl_x_center_kp, cl_y_center_kp)
                cl_kp_img = cv2.line(cl_kp_img, (int(cl_x_center_kp[0]), int(cl_y_center_kp[0])), (400, 800),
                                     (255, 0, 255),
                                     4)
                cl_kp_img = cv2.line(cl_kp_img, (400, int(cl_y_center_kp[0])), (400, 800), (0, 255, 0), 4)


                cl_kp_img = cv2.addWeighted(input_image, 1, cl_kp_img, 0.9, 0)

                self.display_image(cl_kp_img)

                # publishing the curvatures detected by clothoid and interpolation methods
                self.pub_co.publish(curvature_opposing[0])
                self.pub_cl.publish(curvature_left[0])
                self.pub_cr.publish(curvature_right[0])

                print(curvature_right[0], " ", rospy.get_time())

                self.lateral_distance = (lane_points_left_x_m[12] + lane_points_right_x_m[12]) / 2
                print('ydist', lane_cy[5])
                self.move_vehicle(self.lateral_distance, 10)

            elif lane_points_left_y_m.size > 0 and lane_points_right_y_m.size > 0:
                self.display_image(input_image)

                self.lateral_distance = (lane_points_left_x_m[12] + lane_points_right_x_m[12])/2
                self.move_vehicle(self.lateral_distance, 10)

            else:
                self.display_image(input_image)

            self.r.sleep()

        except(IndexError):
            pass

    def test_callback(self, beta, yawrate):
        # subscribing the side slip angle and yaw rate data from subscribers method
        self.beta_old = beta.data
        self.yaw_rate_old = yawrate.data

if __name__ == '__main__':
    print('please')
    try:
        lane_detection_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
