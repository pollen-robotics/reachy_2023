import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np

from sensor_msgs.msg._compressed_image import CompressedImage


class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera')

        self.logger = self.get_logger()

        self.left_pub = self.create_publisher(CompressedImage, '/left_image/image_raw/compressed', 10)
        self.right_pub = self.create_publisher(CompressedImage, '/right_image/image_raw/compressed', 10)

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
