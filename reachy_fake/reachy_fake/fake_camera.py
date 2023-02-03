#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
#  File Name	: fake_camera.py
#  Author	: Steve NGUYEN
#  Contact      : steve.nguyen@pollen-robotics.com
#  Created	: Friday, February  3 2023
#  Revised	:
#  Version	:
#  Target MCU	:
#
#  Notes:	notes
#


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from cv_bridge import CvBridge
import cv2
import numpy as np

from reachy_msgs.srv import GetCameraZoomLevel, GetCameraZoomSpeed
from reachy_msgs.srv import SetCameraZoomLevel, SetCameraZoomSpeed
from reachy_msgs.srv import GetCameraZoomFocus, SetCameraZoomFocus
from reachy_msgs.srv import SetFocusState

from sensor_msgs.msg._compressed_image import CompressedImage


class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera')

        self.logger = self.get_logger()

        # Dummy camera service
        self.get_zoom_level_service = self.create_service(GetCameraZoomLevel, 'get_camera_zoom_level', self.dummy_service_cb)
        self.get_zoom_speed_service = self.create_service(GetCameraZoomSpeed, 'get_camera_zoom_speed', self.dummy_service_cb)
        self.set_zoom_level_service = self.create_service(SetCameraZoomLevel, 'set_camera_zoom_level', self.dummy_service_cb)
        self.set_zoom_speed_service = self.create_service(SetCameraZoomSpeed, 'set_camera_zoom_speed', self.dummy_service_cb)
        self.get_zoom_focus_service = self.create_service(GetCameraZoomFocus, 'get_camera_zoom_focus', self.dummy_service_cb)
        self.set_zoom_focus_service = self.create_service(SetCameraZoomFocus, 'set_camera_zoom_focus', self.dummy_service_cb)
        self.set_focus_state_service = self.create_service(SetFocusState, 'set_focus_state', self.dummy_service_cb)

        self.left_pub = self.create_publisher(CompressedImage, '/left_image/image_raw/compressed', 10)
        self.right_pub = self.create_publisher(CompressedImage, '/right_image/image_raw/compressed', 10)

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        self.img = np.zeros((480, 640, 3), np.uint8)

        self.bridge = CvBridge()
        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (30, 250)
        fontScale = 1
        fontColor = (255, 255, 255)
        thickness = 1
        lineType = 2

        cv2.putText(self.img, 'Cameras unavailable in fake mode',
                    bottomLeftCornerOfText,
                    font,
                    fontScale,
                    fontColor,
                    thickness,
                    lineType)
        self.img_msg = CompressedImage()

        self.img_msg = self.bridge.cv2_to_compressed_imgmsg(np.array(self.img), "jpeg")

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.logger.info(f'Fake camera ready')

    def dummy_service_cb(self, request, response):
        '''Just make the camera services exist, used for Reachy camera server'''
        return response

    def timer_callback(self):
        self.left_pub.publish(self.img_msg)
        self.right_pub.publish(self.img_msg)


def main(args=None):
    rclpy.init(args=args)

    fakecam = FakeCamera()

    rclpy.spin(fakecam)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fakecam.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
