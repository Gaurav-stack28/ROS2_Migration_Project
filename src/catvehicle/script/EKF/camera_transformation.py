import cv2
import numpy


class Camera:
    def __init__(self, image_height, image_width, angle, height, focal_length, image_plane_width, image_plane_height,
                 ground_plane_length, ground_plane_width):
        self.img_height = image_height
        self.img_width = image_width
        self.ang = numpy.tan(numpy.pi / 2 - angle)
        self.h = height
        self.f = focal_length
        self.iw = image_plane_width
        self.ih = image_plane_height
        self.gl = ground_plane_length
        self.gw = ground_plane_width

    def camera_to_birdseye(self, img):
        # process of finding IPM from camera perspective
        y_kpx = numpy.arange(self.img_height)
        x_kpx = numpy.arange(self.img_width).reshape(self.img_width, 1)
        # converting image pixel points to the image plane
        x_km = (x_kpx / self.img_width - 0.5) * self.ih
        y_km = -(y_kpx / self.img_height - 0.5) * self.iw
        # converting points in image plane to 2d coordinates
        y_vm = -((self.f * self.ang - y_km) / (y_km * self.ang + self.f)) * self.h
        x_vm = x_km * (1 + y_vm / self.f)
        # converting 2d coordinates to the IPM view/ Bird's eye view
        x_vpx = numpy.around(x_vm * self.img_height / self.gl) + self.img_width / 2
        y_vpx = self.img_height - numpy.around(y_vm * self.img_height / self.gw)
        # removing unwanted pixels coordinates
        above_img_size = y_vpx >= self.img_height
        y_vpx[above_img_size] = 0
        below_img_size = y_vpx < 0
        y_vpx[below_img_size] = 0

        above_img_size = x_vpx >= self.img_width
        x_vpx[above_img_size] = 0
        below_img_size = x_vpx < 0
        x_vpx[below_img_size] = 0
        # changing types of image coordinate value to integer
        x_vpx = x_vpx.astype(int)
        y_vpx = y_vpx.astype(int)
        # changing coordinates to array matrix
        y_vpx = numpy.tile(y_vpx, (self.img_width, 1))

        y_kpx = numpy.tile(y_kpx, (self.img_height, 1))
        x_kpx = numpy.tile(x_kpx, (1, self.img_width))
        # creating blank image
        bv_img = numpy.zeros((self.img_width, self.img_height), numpy.uint8)
        # input image filling
        bv_img[y_vpx, x_vpx] = img[y_kpx, x_kpx]
        # using inpaint function to fill the derived pixels
        mask = cv2.inRange(bv_img, 0, 0)
        bv_img = cv2.inpaint(bv_img, mask, 1, cv2.INPAINT_TELEA)


        return bv_img

    def pixel_to_meter(self, x, y):
        # changing pixel values of bird's eye image to x and y coordinates in meter
        x_m = self.gl / self.img_height * (x - self.img_width / 2)
        y_m = self.gw / self.img_width * (self.img_height - y)
        return x_m, y_m

    def cam_coordinates_to_plane_m(self, x_kpx, y_kpx):
        # finding coordinates from camera image to bird#s eye view
        x_km = (x_kpx / self.img_width - 0.5) * self.ih
        y_km = -(y_kpx / self.img_height - 0.5) * self.iw

        y_vm = ((self.f * self.ang - y_km) / (y_km * self.ang + self.f)) * self.h
        x_vm = x_km * (1 + y_vm / self.f)

        return x_vm, y_vm

    def plane_m_to_cam_px(self, x_vm, y_vm):
        # changing coordinates from Bird's eye view to camera image coordinate/ changing coordinates to real camera coordinate
        x_vm = numpy.array(x_vm)
        y_vm = numpy.array(y_vm)

        x_km = x_vm / (1 + y_vm / self.f)
        y_km = self.f * (self.ang - y_vm / self.h) / (self.ang * y_vm / self.h + 1)

        x_kpx = self.img_width / self.iw * x_km + self.img_width / 2
        y_kpx = self.img_height / self.ih * y_km + self.img_height / 2

        return numpy.around(x_kpx, 0).astype(int), numpy.around(y_kpx, 0).astype(int)