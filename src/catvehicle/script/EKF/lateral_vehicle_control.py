#!/usr/bin/env python3
import cv2
import numpy
import scipy.optimize as spo
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import message_filters
from pyclothoids import Clothoid
import camera_transformation as ct
from collections import deque
class LateralVehicleControl(Node):
    def __init__(self):
        super().__init__('vehicle_control')
        # Parameters / internal variables
        self.lateral_distance = 0.0
        self.w_lateral = 0.0
        self.beta_old = 0.0
        self.yaw_rate_old = 0.0
        self.steering_angle_last = 0.0
        self.steering_angle_2ndlast = 0.0
        self.steering_angle_deque = deque(maxlen=3)
        self.integrator = 0.0
        self.yaw_angular_acc_normal = 0.0
        self.img_width = 800
        self.img_height = 800
        self.cc = 1.3962634
        # Camera transformation
        self.Cam = ct.Camera(self.img_width,self.img_height,0,1.75,0.75,self.cc,self.cc,50,50)
        # QoS
        qos_sensor = QoSProfile( history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT )
        qos_default = QoSProfile( history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE )
        # Camera subscriber
        self.image_get = self.create_subscription( Image, '/catvehicle/camera_left/image_raw_left', self.camera_callback, qos_sensor )
        # message_filters subscribers
        #
        # ROS 2 message_filters requires a node as the first argument.
        beta_get = message_filters.Subscriber( self, Float64, '/SSAE/beta', qos_profile=qos_default )
        yawrate_get = message_filters.Subscriber( self, Float64, '/SSAE/yawrate', qos_profile=qos_default )
        self.control_data = message_filters.ApproximateTimeSynchronizer( [beta_get, yawrate_get], queue_size=10, slop=1.0, allow_headerless=True )
        self.control_data.registerCallback( self.test_callback )
        # Command velocity publisher
        self.cmd_vel = self.create_publisher( Twist, '/catvehicle/cmd_vel', 10 )
        self.move_cmd = Twist()
        # Curvature publishers
        self.pub_co = self.create_publisher( Float64, '/catvehicle/curvature_opposing', 10 )
        self.pub_cl = self.create_publisher( Float64, '/catvehicle/curvature_left', 10 )
        self.pub_cr = self.create_publisher( Float64, '/catvehicle/curvature_right', 10 )
        self.pub3 = self.create_publisher( Float64, '/catvehicle/ste_angle', 10 )
        self.get_logger().info( 'Lateral vehicle control node started.' )
    # Display image
    def display_image(self, img):
        cv2.imshow('Kamera', img)
        cv2.waitKey(2)
    # Find lane starting points
    def find_lane_start(self, img):
        box_r = img[540:650, 380:630].sum(axis=0)
        box_l = img[540:650, 220:420].sum(axis=0)
        box_o = img[540:650, 0:250].sum(axis=0)
        sum_axisr = box_r.sum(axis=0) + 0.1
        sum_axisl = box_l.sum(axis=0) + 0.1
        sum_axiso = box_o.sum(axis=0) + 0.1
        b = numpy.arange(0, 250)
        a = numpy.arange(0, 200)
        weight_r = (box_r * b).sum() / sum_axisr + 380
        weight_l = (box_l * a).sum() / sum_axiso + 220
        weight_o = (box_o * b).sum() / sum_axisl
        return ( round(weight_o), round(weight_l), round(weight_r), 545 )
    # Sliding window lane detection
    def sliding_windows( self, img, lstx, starty, box_height, box_width ):
        pos_x = lstx
        pos_y = starty
        a = numpy.arange(0, box_width * 2)
        lane_x = []
        lane_y = []
        box = img[ pos_y - box_height:pos_y, pos_x - box_width:pos_x + box_width ].sum(axis=0)
        if box.sum() > 500:
            weight = ( (box * a).sum() / box.sum(axis=0) )
            diff = round(weight - box_width)
        else:
            diff = 0
        a = numpy.arange(0, box_width * 2)
        pos_x += diff
        pos_y -= box_height
        lane_x.append(pos_x)
        lane_y.append(800 - pos_y)
        box = img[ pos_y - box_height:pos_y, pos_x - box_width:pos_x + box_width ].sum(axis=0)
        while box.sum() > 500:
            weight = ( (box * a).sum() / box.sum(axis=0) )
            diff = round(weight - box_width)
            if pos_y > 390:
                box_height = 3
                box_width = 9
                a = numpy.arange( 0, box_width * 2 )
            pos_x += diff
            box = img[ pos_y - box_height:pos_y, pos_x - box_width:pos_x + box_width ].sum(axis=0)
            lane_x.append(pos_x)
            lane_y.append(800 - pos_y)
            pos_x += ( pos_x - lane_x[-2] )
            pos_y -= box_height
            box = img[ pos_y - box_height:pos_y, pos_x - box_width:pos_x + box_width ].sum(axis=0)
        lane_x = numpy.array(lane_x)
        lane_y = numpy.array(lane_y)
        return lane_x, lane_y
    # Interpolate middle lane
    def interpolate_mid_lane( self, lx, ly, rx, ry, ox, oy ):
        diff = len(oy) - len(ry)
        if diff > 0:
            oy = oy[0:-diff - 1]
            ox = ox[0:-diff - 1]
        elif diff < 0:
            ry = ry[0:diff - 1]
            rx = rx[0:diff - 1]
        diff2 = len(ry) - len(ly)
        if diff2 > 0:
            ly = numpy.append( ly, ry[-diff2:] )
            lx = numpy.append( lx, (ox[-diff2:] + rx[-diff2:]) / 2 )
        return ( lx, ly, rx, ry, ox, oy )
    # Clothoid least-square error
    def lsq_error( self, param, lx, ly ):
        ast, aend, ex = param
        lx = numpy.array(lx)
        ly = numpy.array(ly)
        clothoid0 = Clothoid.G1Hermite( lx[0], ly[0], ast, ex, ly[-1], aend )
        cl = [ clothoid0.X( (i - ly[0]) * ( clothoid0.length / (ly[-1] - ly[0]) ) ) for i in ly ]
        diff = numpy.power( cl - lx, 2 )
        score = diff.sum()
        return score
    # Fit clothoid
    def fit_clothoid(self, lx, ly):
        dy = ly[-1] - ly[-5]
        dx = lx[-5] - lx[-1]
        ang_end_guess = ( numpy.arctan(dx / dy) + numpy.pi / 2 )
        dy = ly[5] - ly[0]
        dx = lx[0] - lx[5]
        ang_st_guess = ( numpy.arctan(dx / dy) / 2 + numpy.pi / 2 )
        parameters = numpy.array( [ ang_st_guess, ang_end_guess, lx[-1] ] )
        result = spo.minimize( self.lsq_error, parameters, args=(lx, ly), tol=0.05, options={'maxiter': 20} )
        ang_start, ang_end, x_end = result.x
        clothoid0 = Clothoid.G1Hermite( lx[0], ly[0], ang_start, x_end, ly[-1], ang_end )
        curvature = []
        for s in ly: curvature.append( clothoid0.ThetaD(s) ) 
        sample_x, sample_y = clothoid0.SampleXY( ly.size )
        return ( numpy.array(curvature), sample_x, sample_y )
    # Draw detected lane
    def draw_lines_on_input( self, img, lx, ly ):
        for e in range(len(ly) - 1):
            img = cv2.line( img, ( int(lx[e]), int(ly[e]) ), ( int(lx[e + 1]), int(ly[e + 1]) ), (0, 255, 255), 7 )
        return img
    # Draw detected lane
    def draw_lines_on_input2( self, img, lx, ly ):
        for e in range(len(ly) - 1):
            img = cv2.line( img, ( int(lx[e]), int(ly[e]) ), ( int(lx[e + 1]), int(ly[e + 1]) ), (255, 255, 255), 7 )
        return img
    # Lateral vehicle control
    def vehicle_control(self):
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
        llo = 5.3333
        wltr = 0
        Integrator = 0
        self.get_logger().info( f'lateral dist = {self.lateral_distance}' )
        # Lateral distance error
        lateral_control_err = ( numpy.arctan(wltr / llo) - numpy.arctan( self.lateral_distance / llo ) )
        # Proportional term
        Proportional = ( kp * lateral_control_err )
        # Integral term
        Integrator = ( Integrator + ki * lateral_control_err * del_t )
        # Derivative term
        Derivative = ( -1 * kd * self.yaw_rate_old / del_t )
        # Combined yaw angular acceleration
        yaw_angular_acc_normal = ( Proportional + Integrator + Derivative )
        # Rear lateral force
        S_h = ( C_r * ( self.beta_old + (L_r / vel) * self.yaw_rate_old ) )
        # Front lateral force
        S_v = ( yaw_angular_acc_normal * I_z + S_h * L_r ) / L_f
        # Steering angle
        Steering_angle = ( (S_v / C_f) - self.beta_old + (L_f * self.yaw_rate_old / vel) )
        # Steering angle limits
        if Steering_angle > 0.05:
            Steering_angle = 0.05
        elif Steering_angle < -0.05:
            Steering_angle = -0.05
        # Steering smoothing
        steering_val = ( 0.2 * Steering_angle + 0.4 * self.steering_angle_last + 0.4 * self.steering_angle_2ndlast )
        self.steering_angle_deque.append( steering_val )
        steering_angle_smoothing = steering_val
        self.steering_angle_last = ( steering_angle_smoothing )
        self.steering_angle_2ndlast = ( Steering_angle )
        self.get_logger().info( f'steering = {steering_val}' )
        return steering_angle_smoothing
    # Move vehicle
    def move_vehicle( self, lateral_distance, vel ):
        stervalue = self.vehicle_control()
        # Steering
        self.move_cmd.angular.z = float( stervalue )
        # Vehicle velocity
        if numpy.absolute(stervalue) > 0.02:
            self.move_cmd.linear.x = 10.0
        else:
            self.move_cmd.linear.x = float( vel )
        self.cmd_vel.publish( self.move_cmd )
        # Publish steering angle
        steering_msg = Float64()
        steering_msg.data = float( stervalue )
        self.pub3.publish( steering_msg )
        self.get_logger().info( f'velocity = {self.move_cmd.linear.x}, ' f'steering = {stervalue}' )
    # Camera callback
    def camera_callback(self, image):
        try:
            # Convert ROS Image to NumPy image
            input_image = numpy.frombuffer( image.data, dtype=numpy.uint8 ).reshape( image.height, image.width, -1 )
            # Convert BGR image to grayscale
            gray_image = cv2.cvtColor( input_image, cv2.COLOR_BGR2GRAY )
            # Region of interest
            gray_image[0:-1][555:800] = 0
            gray_image[0:-1][0:395] = 0
            # Thresholding
            ret, threshold_image = cv2.threshold( gray_image, 220, 255, cv2.THRESH_BINARY )
            # Gaussian filtering
            edge_image = cv2.GaussianBlur( gray_image, (5, 5), 0 )
            # Canny edge detection
            edge_image = cv2.Canny( edge_image, 200, 250 )
            # Combine threshold + edges
            combined_image = ( edge_image + threshold_image )
            # Remove unwanted edges
            combined_image[0:-1][390:412] = 0
            combined_image[0:-1][550:560] = 0
            combined_image_bv = combined_image
            # Find lane starting points
            ( lane_opposing_start, lane_left_start, lane_right_start, start_y ) = self.find_lane_start( combined_image_bv )
            # Left lane
            ( lane_points_left_x_px, lane_points_left_y_px ) = self.sliding_windows( combined_image_bv, lane_left_start, start_y, 4, 10 )
            # Right lane
            ( lane_points_right_x_px, lane_points_right_y_px ) = self.sliding_windows( combined_image_bv, lane_right_start, start_y, 4, 10 )
            # Opposing lane
            ( lane_points_opposing_x_px, lane_points_opposing_y_px ) = self.sliding_windows( combined_image_bv, lane_opposing_start, start_y, 4, 10 )
            # Convert pixels to meters
            ( lane_points_left_x_m, lane_points_left_y_m ) = self.Cam.cam_coordinates_to_plane_m( lane_points_left_x_px, lane_points_left_y_px )
            ( lane_points_right_x_m, lane_points_right_y_m ) = self.Cam.cam_coordinates_to_plane_m( lane_points_right_x_px, lane_points_right_y_px )
            ( lane_points_opposing_x_m, lane_points_opposing_y_m ) = self.Cam.cam_coordinates_to_plane_m( lane_points_opposing_x_px, lane_points_opposing_y_px )
            # Interpolate middle lane
            ( lane_points_left_x_m, lane_points_left_y_m, lane_points_right_x_m, lane_points_right_y_m, lane_points_opposing_x_m, lane_points_opposing_y_m ) = self.interpolate_mid_lane( lane_points_left_x_m, lane_points_left_y_m, lane_points_right_x_m, lane_points_right_y_m, lane_points_opposing_x_m, lane_points_opposing_y_m )
            lane_cx = ( lane_points_right_x_m + lane_points_left_x_m ) / 2
            lane_cy = ( lane_points_left_y_m + lane_points_right_y_m ) / 2
            # Full lane detection
            if ( lane_points_opposing_y_m.size > 20 and lane_points_left_y_m.size > 20 and lane_points_right_y_m.size > 20 ):
                # Fit clothoids
                ( curvature_left, clothoid_x_left, clothoid_y_left ) = self.fit_clothoid( lane_points_left_x_m[10:], lane_points_left_y_m[10:] )
                ( curvature_right, clothoid_x_right, clothoid_y_right ) = self.fit_clothoid( lane_points_right_x_m[10:], lane_points_right_y_m[10:] )
                ( curvature_opposing, clothoid_x_opposing, clothoid_y_opposing ) = self.fit_clothoid( lane_points_opposing_x_m[10:], lane_points_opposing_y_m[10:] )
                ( curvature_center, clothoid_x_center, clothoid_y_center ) = self.fit_clothoid( lane_cx[10:], lane_cy[10:] )
                # Convert clothoid coordinates back to pixels
                ( cl_x_left_kp, cl_y_left_kp ) = self.Cam.plane_m_to_cam_px( clothoid_x_left, clothoid_y_left )
                ( cl_x_right_kp, cl_y_right_kp ) = self.Cam.plane_m_to_cam_px( clothoid_x_right, clothoid_y_right )
                ( cl_x_opposing_kp, cl_y_opposing_kp ) = self.Cam.plane_m_to_cam_px( clothoid_x_opposing, clothoid_y_opposing )
                ( cl_x_center_kp, cl_y_center_kp ) = self.Cam.plane_m_to_cam_px( clothoid_x_center, clothoid_y_center )
                # Create visualization image
                cl_kp_img = numpy.zeros( (800, 800, 3), numpy.uint8 )
                cl_kp_img = self.draw_lines_on_input( cl_kp_img, cl_x_left_kp, cl_y_left_kp )
                cl_kp_img = self.draw_lines_on_input( cl_kp_img, cl_x_right_kp, cl_y_right_kp )
                cl_kp_img = self.draw_lines_on_input( cl_kp_img, cl_x_opposing_kp, cl_y_opposing_kp )
                cl_kp_img = self.draw_lines_on_input( cl_kp_img, cl_x_center_kp, cl_y_center_kp )
                # Center line
                cl_kp_img = cv2.line( cl_kp_img, ( int(cl_x_center_kp[0]), int(cl_y_center_kp[0]) ), (400, 800), (255, 0, 255), 4 )
                # Vehicle orientation line
                cl_kp_img = cv2.line( cl_kp_img, ( 400, int(cl_y_center_kp[0]) ), (400, 800), (0, 255, 0), 4 )
                # Overlay
                cl_kp_img = cv2.addWeighted( input_image, 1, cl_kp_img, 0.9, 0 )
                self.display_image( cl_kp_img )
                # Publish curvature values
                msg_opposing = Float64()
                msg_opposing.data = float( curvature_opposing[0] )
                msg_left = Float64()
                msg_left.data = float( curvature_left[0] )
                msg_right = Float64()
                msg_right.data = float( curvature_right[0] )
                self.pub_co.publish( msg_opposing )
                self.pub_cl.publish( msg_left )
                self.pub_cr.publish( msg_right )
                # ROS 2 timestamp
                current_time = ( self.get_clock() .now() .nanoseconds / 1e9 )
                self.get_logger().info( f'curvature_right = ' f'{curvature_right[0]:.6f}, ' f'time = {current_time:.3f}' )
                # Lateral distance
                self.lateral_distance = ( lane_points_left_x_m[12] + lane_points_right_x_m[12] ) / 2
                self.get_logger().info( f'ydist = {lane_cy[5]}' )
                # Vehicle control
                self.move_vehicle( self.lateral_distance, 10 )
            # Only left + right lanes detected
            elif ( lane_points_left_y_m.size > 0 and lane_points_right_y_m.size > 0 ):
                self.display_image( input_image )
                self.lateral_distance = ( lane_points_left_x_m[12] + lane_points_right_x_m[12] ) / 2
                self.move_vehicle( self.lateral_distance, 10 )
            # No usable lane
            else:
                self.display_image( input_image )
        except IndexError:
            # Keep original behavior:
            # ignore insufficient lane-point cases.
            pass
        except Exception as e:
            self.get_logger().error( f'Camera callback error: {e}' )
    # SSAE beta + yaw rate callback
    def test_callback( self, beta, yawrate ):
        self.beta_old = float( beta.data )
        self.yaw_rate_old = float( yawrate.data )
# Main
def main(args=None):
    rclpy.init(args=args)
    node = LateralVehicleControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
