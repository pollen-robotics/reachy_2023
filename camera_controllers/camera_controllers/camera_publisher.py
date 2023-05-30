"""
Camera Node.

- publish /left_image and /right_image at the specified rate (default: 30Hz)

Supported resolutions are: (1920,1080) at 15FPS, (1280,720) at 30FPS, (640,480) at 30FPS. <-- TODO update for luxonis FFC if needed
"""
from functools import partial
from threading import Thread
from typing import Dict
from subprocess import Popen, PIPE
from ffc_wrapper import FFCWrapper
import cv2
from cv_bridge import CvBridge
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg._compressed_image import CompressedImage


class CameraPublisher(Node):
    """Camera Publisher class."""

    def __init__(self,
                 left_port: str = '/dev/left_camera',
                 right_port: str = '/dev/right_camera',
                 resolution: tuple = (1920, 1080),
                 fps: int = 30) -> None:
        """Connect to both cameras, initialize the publishers."""
        super().__init__('camera_publisher')
        self.logger = self.get_logger()
        self.clock = self.get_clock()

        self.ffcw = FFCWrapper("/home/reachy/dev/Reachy_stereo/luxonis/MY_MODULAR_CONFIG.json")

        self.camera_publisher_left = self.create_publisher(CompressedImage, 'left_image', 1)
        self.logger.info(f'Launching "{self.camera_publisher_left.topic_name}" publisher.')

        self.camera_publisher_right = self.create_publisher(CompressedImage, 'right_image', 1)
        self.logger.info(f'Launching "{self.camera_publisher_right.topic_name}" publisher.')

        self.publisher = {
            'left': self.camera_publisher_left,
            'right': self.camera_publisher_right
        }

        self.compr_img: Dict[str, CompressedImage] = {}

        self.bridge = CvBridge()

        def publisher():
            self.logger.info('Left and Right cameras ready to publish!')
            while True:
                imgList = self.ffcw.get_sync_images(rectify=True, invert_h=True, invert_v=True)
                if imgList is not None:
                    self.logger.info('PUBLISHING')
                    leftIm = imgList["left"]
                    rightIm = imgList["right"]
                    
                    leftIm = cv2.resize(leftIm, (0, 0), fx=0.5, fy=0.5)
                    rightIm = cv2.resize(rightIm, (0, 0), fx=0.5, fy=0.5)

                    # im = np.random.rand(1920, 1080, 3)*255

                    # leftIm = self.bridge.cv2_to_compressed_imgmsg(im, "jpeg")
                    # rightIm = self.bridge.cv2_to_compressed_imgmsg(im, "jpeg")

                    leftIm = self.bridge.cv2_to_compressed_imgmsg(leftIm, "jpeg")
                    rightIm = self.bridge.cv2_to_compressed_imgmsg(rightIm, "jpeg")

                    self.publish_img("left", leftIm)
                    self.publish_img("right", rightIm)

        for side in ('left', 'right'):
            compr_img = CompressedImage()
            compr_img.format = 'jpeg'
            self.compr_img[side] = compr_img

        t = Thread(target=publisher)
        t.daemon = True
        t.start()

        self.logger.info('Node ready!')

    def publish_img(self, side: str, frame: bytes) -> None:
        """Read image from the requested side and publishes it."""
        # compr_img = self.compr_img[side]
        # compr_img.header.stamp = self.clock.now().to_msg()
        # compr_img.data = frame
        frame.header.stamp = self.clock.now().to_msg()
        self.publisher[side].publish(frame)

    def _rotate(self, frame, angle):
        process = Popen([
            'jpegtran', '-rotate', angle,
        ], stdin=PIPE, stdout=PIPE, bufsize=-1)
        out, err = process.communicate(frame)
        process.wait()
        return out


def main() -> None:
    """Run Camera publisher main loop."""
    rclpy.init()

    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
