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
from cv_bridge import CvBridge
import cv2
import numpy as np

from reachy_msgs.srv import GetCameraZoomLevel, GetCameraZoomSpeed
from reachy_msgs.srv import SetCameraZoomLevel, SetCameraZoomSpeed
from reachy_msgs.srv import GetCameraZoomFocus, SetCameraZoomFocus
from reachy_msgs.srv import SetFocusState


class FakeZoom(Node):
    def __init__(self):
        super().__init__('fake_zoom')

        self.logger = self.get_logger()

        # Dummy camera service
        self.get_zoom_level_service = self.create_service(GetCameraZoomLevel, 'get_camera_zoom_level', self.dummy_zoom_service_cb)
        self.get_zoom_speed_service = self.create_service(GetCameraZoomSpeed, 'get_camera_zoom_speed', self.dummy_service_cb)
        self.set_zoom_level_service = self.create_service(SetCameraZoomLevel, 'set_camera_zoom_level', self.dummy_service_cb)
        self.set_zoom_speed_service = self.create_service(SetCameraZoomSpeed, 'set_camera_zoom_speed', self.dummy_service_cb)
        self.get_zoom_focus_service = self.create_service(GetCameraZoomFocus, 'get_camera_zoom_focus', self.dummy_service_cb)
        self.set_zoom_focus_service = self.create_service(SetCameraZoomFocus, 'set_camera_zoom_focus', self.dummy_service_cb)
        self.set_focus_state_service = self.create_service(SetFocusState, 'set_focus_state', self.dummy_service_cb)

        self.logger.info(f'Fake zoom ready')

    def dummy_service_cb(self, request, response):
        '''Just make the camera services exist, used for Reachy camera server'''
        return response

    def dummy_zoom_service_cb(self, request, response):
        '''Just make the zoom level service exist, used for Reachy camera server'''
        response.zoom_level = 'inter'
        return response


def main(args=None):
    rclpy.init(args=args)

    fake_zoom = FakeZoom()

    rclpy.spin(fake_zoom)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_zoom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
