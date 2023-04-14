import rclpy
from rclpy.node import Node
import time
import math
import traceback
import sys
from example_interfaces.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.constants import S_TO_NS
from collections import deque
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import copy

LASER_UPPER_ANGLE = math.pi*2 - math.pi/4
LASER_LOWER_ANGLE = math.pi/4

class LaserFilter(Node):
    def __init__(self):
        super().__init__('zuuu_laser_filter')
        self.get_logger().info("Starting zuuu_laser_filter!")

        # TODO Temporary fix since https://github.com/ros-perception/laser_filters doesn't work on Foxy aparently
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_filter_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.scan_sub  # prevent unused variable warning... JESUS WHAT HAVE WE BECOME
        self.scan_pub = self.create_publisher(
            LaserScan, 'scan_filtered', 10)

        self.get_logger().info(
            "zuuu_laser_filter started")


    def scan_filter_callback(self, msg) :
        filtered_scan = LaserScan() 
        filtered_scan.header = copy.deepcopy(msg.header)
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max
        ranges = []
        intensities = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i*msg.angle_increment
            if angle > LASER_UPPER_ANGLE or angle < LASER_LOWER_ANGLE :
                ranges.append(0.0)
                intensities.append(0.0)
            else :
                ranges.append(r)
                intensities.append(msg.intensities[i])
        filtered_scan.ranges = ranges
        filtered_scan.intensities = intensities
        self.scan_pub.publish(filtered_scan)

    def angle_diff(self, a, b):
        # Returns the smallest distance between 2 angles
        d = a - b
        d = ((d + math.pi) % (2 * math.pi)) - math.pi
        return d



def main(args=None):
    rclpy.init(args=args)
    node = LaserFilter()

    try:
        rclpy.spin(node)
    except Exception as e:
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
