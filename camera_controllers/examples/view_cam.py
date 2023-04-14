"""Test Reachy's cameras, either by subscribing to ROS topics or by using OpenCV.

If you're usig ROS, make sure that the camera_publisher.launch.py has been launched so that
the topics /left_image and /right_image are actually published.

On the contrary, if you're using OpenCV, camera_publisher.launch.py must NOT be launched,
or you won't get acces to the cameras.
"""

import time
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg._compressed_image import CompressedImage
import cv2 as cv


class RosCameraSubscriber(
                Node,
                ):
    """ROS node subscribing to the image topics."""

    def __init__(self, node_name: str, side: str) -> None:
        """Set up the node.

        Subscribe to the requested image topic (either /left_image/image_raw/compressed or /right_image/image_raw/compressed).
        """
        super().__init__(node_name=node_name)

        self.camera_sub = self.create_subscription(
            CompressedImage,
            side+'_image/image_raw/compressed',
            self.on_image_update,
            1,
        )

        self.cam_img = None

    def on_image_update(self, msg):
        """Get data from image. Callback for "/'side'_image "subscriber."""
        data = np.frombuffer(msg.data.tobytes(), dtype=np.uint8)
        self.cam_img = cv.imdecode(data, cv.IMREAD_COLOR)

    def update_image(self):
        """Get the last image by spinning the node."""
        rclpy.spin_once(self)


class OpenCvCameraViewer:
    """CameraViewer using OpenCV."""

    rot = {
        'left': 3,  # 3 * 90 = 270
        'right': 1,  # 1 * 90 = 90
    }

    def __init__(self, side: str) -> None:
        """Initialize cap by connecting to the required video port and configuring it."""
        self.cap = cv.VideoCapture(f'/dev/{side}_camera', apiPreference=cv.CAP_V4L2)
        self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('m', 'j', 'p', 'g'))
        self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv.CAP_PROP_FPS, 30)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)

        self.cam_img = None
        self.rot_value = self.rot[side]

    def update_image(self):
        """Get the last image by reading cap."""
        _, im = self.cap.read()
        self.cam_img = np.rot90(im, self.rot_value)


def main():
    """Instanciate the correct CameraViewer object for the requested side."""
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('side')
    parser.add_argument('source')

    args = parser.parse_args()

    requested_side = args.side

    if requested_side not in ['left', 'right']:
        raise ValueError("side argument must either be 'left' or 'right'")

    viewing_source = args.source
    if viewing_source not in ['ros', 'opencv']:
        raise ValueError("source must either be 'ros' or 'opencv'")

    if viewing_source == 'ros':
        rclpy.init()
        time.sleep(1)
        image_getter = RosCameraSubscriber(node_name='image_viewer', side=requested_side)
    else:
        image_getter = OpenCvCameraViewer(side=requested_side)

    while True:
        image_getter.update_image()
        cv.imshow(args.side + ' camera', image_getter.cam_img)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    main()
