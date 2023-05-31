"""
Camera Node.

- publish /left_image and /right_image at the specified rate (default: 30Hz)

Supported resolutions are: (1920,1080) at 15FPS, (1280,720) at 30FPS, (640,480) at 30FPS. <-- TODO update for luxonis FFC if needed
"""
from typing import Dict
from subprocess import Popen, PIPE
from ffc_wrapper import FFCWrapper
from cv_bridge import CvBridge

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

        self.ffcw = FFCWrapper("/home/reachy/dev/Reachy_stereo/luxonis/MY_MODULAR_CONFIG.json", resize=True, imsize=(888, 500))

        self.camera_publisher_left = self.create_publisher(CompressedImage, 'left_image/image_raw/compressed', 1)
        self.logger.info(f'Launching "{self.camera_publisher_left.topic_name}" publisher.')

        self.camera_publisher_right = self.create_publisher(CompressedImage, 'right_image/image_raw/compressed', 1)
        self.logger.info(f'Launching "{self.camera_publisher_right.topic_name}" publisher.')

        self.publisher = {
            'left': self.camera_publisher_left,
            'right': self.camera_publisher_right
        }

        self.compr_img: Dict[str, CompressedImage] = {}

        self.bridge = CvBridge()

        self.last_images = {"left": None, "right": None}

        def publisher():
            if self.last_images is not None:
                if self.last_images["left"] is not None:
                    self.publish_img("left", self.last_images["left"])
                if self.last_images["right"] is not None:
                    self.publish_img("right", self.last_images["right"])

        for side in ('left', 'right'):
            compr_img = CompressedImage()
            compr_img.format = 'jpeg'
            self.compr_img[side] = compr_img

        self.publisher_timer = self.create_timer(
            timer_period_sec=1/30,
            callback=publisher,
        )
        self.get_images_timer = self.create_timer(
            timer_period_sec=1/30,
            callback=self.get_images,
        )

        self.logger.info('Node ready!')

    def get_images(self):
        imgList = self.ffcw.get_images(rectify=True)

        if imgList is None:
            return

        if imgList["left"] is not None:
            leftIm = imgList["left"]
            leftIm = self.bridge.cv2_to_compressed_imgmsg(leftIm, "jpg")
            self.last_images["left"] = leftIm
        if imgList["right"] is not None:
            rightIm = imgList["right"]
            rightIm = self.bridge.cv2_to_compressed_imgmsg(rightIm, "jpg")
            self.last_images["right"] = rightIm

    def publish_img(self, side: str, frame: bytes) -> None:
        """Read image from the requested side and publishes it."""
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
