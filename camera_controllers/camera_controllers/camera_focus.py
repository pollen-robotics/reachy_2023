"""Node to perform autofocus on Reachy's cameras."""
import cv2 as cv
from cv_bridge import CvBridge

from functools import partial

import threading
import time
from typing import Dict

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg._compressed_image import CompressedImage

from reachy_msgs.msg import ZoomCommand
from reachy_msgs.srv import GetCameraZoomFocus, SetCameraZoomFocus
from reachy_msgs.srv import SetFocusState


class CameraFocus(Node):
    """Handle the autofocus of both reachy cameras in real time."""

    def __init__(self):
        """Set up variables shared between threads, publishers and clients."""
        super().__init__('camera_focus')

        self.eyes_info = {
            'left_eye': {
                'pos': 0,
                'final_pos': 0,
                'min_pos': 0,
                'max_pos': 0,
                'init': True,
                'current_zoom': -1,
                'compressed_img': None,
                'focus_flag': False,
            },
            'right_eye': {
                'pos': 0,
                'final_pos': 0,
                'min_pos': 0,
                'max_pos': 0,
                'init': True,
                'current_zoom': -1,
                'compressed_img': None,
                'focus_flag': False,
            },
        }

        self.logger = self.get_logger()
        self.bridge = CvBridge()

        self.camera_subscriber_left = self.create_subscription(
            CompressedImage, 'left_image',
            partial(self.on_image_update, side='left'),
            1,
            )

        self.camera_subscriber_right = self.create_subscription(
            CompressedImage, 'right_image',
            partial(self.on_image_update, side='right'),
            1,
            )

        self.set_focus_state_service = self.create_service(
            SetFocusState,
            'set_focus_state',
            self._set_focus_state_callback,
        )

        self.set_camera_zoom_focus_client = self.create_client(
            SetCameraZoomFocus,
            'set_camera_zoom_focus',
        )

        self.get_camera_zoom_focus_client = self.create_client(
            GetCameraZoomFocus,
            'get_camera_zoom_focus',
        )

        self.right_eye_thread = threading.Thread(
            target=self.focusing_algorithm,
            args=('right_eye',),
            daemon=True)
        self.left_eye_thread = threading.Thread(
            target=self.focusing_algorithm,
            args=('left_eye',),
            daemon=True)

        self.right_eye_thread.start()
        self.left_eye_thread.start()

    def _wait_for(self, future):
        for _ in range(10000):
            if future.done():
                return future.result()
            time.sleep(0.001)

    def compute_poses_maxima(self, eye):
        """Return range limitation regarding current zoom position.

        Args:
            eye: either 'left_eye' or 'right_eye'.
        """
        self.eyes_info[eye]['min_pos'] = max(int(500 - (np.math.exp(0.01*self.eyes_info[eye]['current_zoom'])+25)*5), 0)
        self.eyes_info[eye]['max_pos'] = min(int(500 - (np.math.exp(0.05*self.eyes_info[eye]['current_zoom']/6)+5)*5), 500)

    def compute_next_pose(self, eye: str, step: int):
        """Compute the next position to reach regarding range limitations.

        Args:
            eye: either 'left_eye' or 'right_eye'
            step:step between the current position and the next desired,
            can be positive as negative value
        """
        if self.eyes_info[eye]['min_pos'] < self.eyes_info[eye]['pos'] + step < self.eyes_info[eye]['max_pos']:
            self.eyes_info[eye]['pos'] += step
        elif self.eyes_info[eye]['pos'] + step >= self.eyes_info[eye]['max_pos']:
            self.eyes_info[eye]['pos'] = self.eyes_info[eye]['max_pos']
        elif self.eyes_info[eye]['pos'] + step <= self.eyes_info[eye]['min_pos']:
            self.eyes_info[eye]['pos'] = self.eyes_info[eye]['min_pos']
        return self.eyes_info[eye]['pos']

    def canny_sharpness_function(self, im):
        """Return the shaprness of im through canny edge dectection algorithm.

        Args:
            im: image used in canny edge detection algorithm
        """
        im = self.bridge.compressed_imgmsg_to_cv2(im)
        im = cv.cvtColor(im, cv.COLOR_BGR2GRAY)
        im = cv.Canny(im, 50, 100)
        im_sum = cv.integral(im)
        return im_sum[-1][-1]/(im.shape[0]*im.shape[1])

    def on_image_update(self, msg, side):
        """Get data from image. Callback for "/'side'_image "subscriber."""
        self.eyes_info[side+'_eye']['compressed_img'] = msg

    def _set_focus_state_callback(self,
                                  request: SetFocusState.Request,
                                  response: SetFocusState.Response
                                  ) -> SetFocusState.Response:
        for eye, state in zip(request.eye, request.state):
            if eye not in ['left_eye', 'right_eye']:
                self.logger.warning("Invalid name sent to focus controller (must be in ('left_eye', 'right_eye')).")
                response.success = False
                return response
            self.eyes_info[eye]['focus_flag'] = state
            if state:
                self.logger.info(f'Starting autofocus on {eye}.')
                self.eyes_info[eye]['current_zoom'] = -1
                self.eyes_info[eye]['init'] = True
            else:
                self.logger.info(f'Stopping autofocus on {eye}.')
        response.success = True
        return response

    def send_request_set_camera_zoom_focus(self, command: Dict):
        """Set the focus and/or zoom of a given camera using SetCameraZoomFocus service."""
        req = SetCameraZoomFocus.Request()

        for side, cmd in command.items():
            for cmd_name, value in cmd.items():
                zoom_cmd_msg = ZoomCommand()
                zoom_cmd_msg.flag = True
                zoom_cmd_msg.value = value
                setattr(req, side+'_'+cmd_name, zoom_cmd_msg)
        result = self._wait_for(self.set_camera_zoom_focus_client.call_async(req))
        return result

    def send_request_get_camera_zoom_focus(self):
        """Get the focus and zoom of both cameras."""
        req = GetCameraZoomFocus.Request()
        result = self._wait_for(self.get_camera_zoom_focus_client.call_async(req))
        return result

    def focusing_algorithm(self, eye):
        """Perform autofocus on a given camera.

        Args:
            eye: either 'left_eye' or 'right_eye'.
        """
        max_res = 0  # Best canny sharpness function result obtained
        p_max = 0  # focus position link to max_res
        low_thresh = 0  # lower noise tolerance threshold
        up_thresh = 0  # upper noise tolerance threshold
        step = 1  # moving step
        self.eyes_info[eye]['init'] = True
        first = True
        stop = 0
        zoom = self.eyes_info[eye]['current_zoom']
        noise = 0.4
        step = 1
        eye_side = eye.split('_')[0]

        time.sleep(15.0)

        while not self.eyes_info[eye]['compressed_img']:
            self.logger.info(f"Waiting for an image from /{eye_side}_image...")
            time.sleep(5.0)
            continue

        self.logger.info(f'Autofocus node for {eye} ready!')

        while(1):
            if self.eyes_info[eye]['focus_flag']:
                res = self.canny_sharpness_function(self.eyes_info[eye]['compressed_img'])

                if self.eyes_info[eye]['init']:
                    while self.eyes_info[eye]['current_zoom'] == -1:
                        self.eyes_info[eye]['current_zoom'] = getattr(self.send_request_get_camera_zoom_focus(), eye_side+'_zoom')

                    zoom = self.eyes_info[eye]['current_zoom']

                    if zoom < 100:
                        noise = 5

                    first = True
                    stop = 0
                    self.compute_poses_maxima(eye)
                    self.eyes_info[eye]['pos'] = self.eyes_info[eye]['min_pos']
                    max_res = 0
                    self.eyes_info[eye]['init'] = False

                    self.send_request_set_camera_zoom_focus({eye_side: {'focus': self.eyes_info[eye]['min_pos']}})
                    time.sleep(2)

                elif stop == 0:
                    if res > max_res:
                        max_res = res
                        p_max = self.eyes_info[eye]['pos']

                    if first:
                        first = False
                        low_thresh = res - noise
                        up_thresh = res + noise
                        self.compute_next_pose(eye, step)
                    elif res < low_thresh or self.eyes_info[eye]['pos'] == self.eyes_info[eye]['max_pos']:
                        self.eyes_info[eye]['final_pos'] = p_max
                        stop = 1
                        temp_pose = self.compute_next_pose(eye, step=-30)
                        self.send_request_set_camera_zoom_focus({eye_side: {'focus': temp_pose}})

                        time.sleep(0.5)
                        self.send_request_set_camera_zoom_focus({eye_side: {'focus': self.eyes_info[eye]['final_pos']}})
                        time.sleep(0.5)
                        self.eyes_info[eye]['pos'] = self.eyes_info[eye]['final_pos']
                        self.eyes_info[eye]['final_pos'] = -1
                        self.eyes_info[eye]['focus_flag'] = False
                        self.logger.info(f'Finished autofocus on {eye}.')

                    elif res > up_thresh:
                        low_thresh = res - noise
                        up_thresh = res + noise
                        step = 1
                        self.compute_next_pose(eye, step)

                    else:
                        if step == 1:
                            step = 5
                        self.eyes_info[eye]['pos'] = self.compute_next_pose(eye, step)
                    self.send_request_set_camera_zoom_focus({eye_side: {'zoom': zoom, 'focus': self.eyes_info[eye]['pos']}})
                    time.sleep(0.15)

            else:
                time.sleep(0.04)


def main(args=None):
    """Create and launch CameraFocus Node.

    If ctrl+c is pressed, node is destroyed.
    """
    rclpy.init(args=args)

    camera_focus = CameraFocus()

    try:
        rclpy.spin(camera_focus)
    except KeyboardInterrupt:

        camera_focus.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
