import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
import traceback
from pyvesc.VESC import MultiVESC
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
import sys

# The cv2 import is only needed in the verbose mode of the follow_me
# where images are created based on the LIDAR scan
# import cv2


def sign(x):
    if x >= 0:
        return 1
    else:
        return -1


class FollowMe(Node):
    def __init__(self):
        super().__init__('follow_me')
        self.get_logger().info("Starting follow me behavior!")
        self.laser_scan = None

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription  # prevent unused variable warning... JESUS WHAT HAVE WE BECOME

        self.cmd_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)

        self.lin_max_speed = 0.5
        self.rot_max_speed = 1.2
        self.t0 = time.time()
        self.get_logger().info(
            "Follow me started.")
        self.create_timer(0.01, self.main_tick)

    def emergency_shutdown(self):
        self.get_logger().warn("Follow me emergency shutdown! Spamming a Twist of 0s!")
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        for i in range(4):
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

    def scan_callback(self, msg):
        # self.get_logger().warn("SCAN:\n{}\n".format(msg))
        self.laser_scan = msg

    def angle_diff(self, a, b):
        # Returns the smallest distance between 2 angles
        d = a - b
        d = ((d + math.pi) % (2 * math.pi)) - math.pi
        return d

    def get_barycenter_offset(self, range_min, range_max, detection_angle, verbose=False):
        """Listen to the /scan LaserScan topic and outputs dlin_percent and dang_percent
        which represent how far away from the center of the chosen shape the barycenter of points is.
        """

        """
        sensor_msgs.msg.LaserScan(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1643673771
         nanosec=826557127)
         frame_id='laser')
         angle_min=-3.1241390705108643
         angle_max=3.1415927410125732
         angle_increment=0.0019344649044796824
         time_increment=3.075326094403863e-05
         scan_time=0.09960980713367462
         range_min=0.15000000596046448
         range_max=30.0
        """
        if self.laser_scan is None:
            return 0, 0

        absolute_range_max = 3.5
        half_angle = detection_angle/2

        angle_increment = self.laser_scan.angle_increment
        pixel_per_meter = 250
        image_size = int(absolute_range_max * pixel_per_meter)
        height = image_size
        width = image_size
        # self.get_logger().info("Image will be {}x{}".format(width, height))
        image = np.zeros((height, width, 3), np.uint8)
        index = -1
        center_x = width / 2
        center_y = 0.5*(range_max + range_min) * pixel_per_meter
        sum_x = 0
        sum_y = 0

        nb_points = 0
        if verbose:
            for i in range(width):
                for j in range(height):
                    angle = math.atan2(i-(width/2), j)
                    dist = math.sqrt(((width/2)-i)**2 + j**2)/pixel_per_meter
                    if dist >= range_min and dist <= range_max and abs(self.angle_diff(angle, 0)) < half_angle:
                        image[j, i] = (50, 50, 100)  # y, x as always

        # the 0, 0 point of the laserscan will be drawn on the point x = width/2 and y = 0 (so the top of the image)
        for r in self.laser_scan.ranges:
            index += 1
            d = 0
            try:
                d = float(r)
                if np.isnan(d) or math.isinf(d):
                    continue
            except ValueError as e:
                # inf?
                continue
            if d < range_min or d >= range_max:
                continue
            angle = self.laser_scan.angle_min + index * angle_increment
            if abs(self.angle_diff(angle, 0)) > half_angle:
                continue
            x = int(round(width / 2 + d * pixel_per_meter * math.sin(angle)))
            y = int(round(d * pixel_per_meter * math.cos(angle)))
            sum_x += x
            sum_y += y
            nb_points += 1
            if x >= 0 and x < width and y >= 0 and y < height:
                image[y, x] = (255, 255, 255)  # y, x as always
            # print("Adding point {}".format(round(d*math.cos(angle)), round(width/2 + d*math.sin(angle))))
        if nb_points > 0:
            sum_x = sum_x/nb_points
            sum_y = sum_y/nb_points
            image[int(sum_y), int(sum_x)] = (255, 0, 0)  # y, x as always
            # Careful, x in image frame is -y lidar frame, and y in image frame is x in lidar frame
            avg_angle = math.atan2(sum_x - width/2, sum_y)
            # dist = math.sqrt((sum_x-center_x)**2 + (sum_y-center_y)**2)
            dist = abs(sum_y-center_y)
            if center_y < sum_y:
                dist *= -1
            dlin_percent = max(-1, min(1, 2*dist /
                               (pixel_per_meter*(range_max - range_min))))
            dang_percent = max(-1, min(1, avg_angle/half_angle))
            # self.get_logger().info("avg_x_pixels={}, avg_y_pixels){}, avg_angle={}, dist_pixels={}, dlin_percent={},
            #  dang_percent={}, center_y={}".format(sum_x, sum_y, avg_angle,
            #  dist, dlin_percent, dang_percent, center_y))
        else:
            dlin_percent = 0
            dang_percent = 0

        if verbose:
            # Disabling the pylint warnings for these 2 lines because forcing the cv2 import is worse.
            cv2.imshow("image", image)  # type: ignore
            # cv2.imwrite("raw" + str(datetime.utcnow())+".png", image)
            cv2.waitKey(1)  # type: ignore
        return dlin_percent, dang_percent

    def main_tick(self, verbose=False):
        if verbose:
            t = time.time()
            dt = t - self.t0
            if dt == 0:
                f = 0
            else:
                f = 1/dt
            self.get_logger().info("Potential freq: {:.0f}Hz".format(f))

        # dlin_percent, dang_percent = self.get_barycenter_offset(0.75, 2.25, math.pi/4)
        dlin_percent, dang_percent = self.get_barycenter_offset(
            0.3, 1.0, math.pi/4)

        lin_speed = -dlin_percent*self.lin_max_speed
        rot_speed = dang_percent*self.rot_max_speed
        self.get_logger().info("lin_speed={:.2f}, rot_speed={:.2f}".format(
            lin_speed, rot_speed))
        twist = Twist()
        twist.linear.x = lin_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = rot_speed
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = FollowMe()
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
        rclpy.logging._root_logger.error('Failed to init FollowMe')
        rclpy.shutdown()
        sys.exit(1)

    try:
        rclpy.spin(node)
    except Exception:
        rclpy.logging._root_logger.error(traceback.format_exc())
    finally:
        node.emergency_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
