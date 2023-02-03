import sys

import geometry_msgs.msg
import rclpy
import time


def main():

    rclpy.init()

    node = rclpy.create_node('zuuu_speed_calibration')
    pub = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)

    try:
        for i in range(8):
            t0 = time.time()
            theta = i*0.05
            node.get_logger().info(
                "setting rot speed to {:.2f}% PWM".format(theta))
            while (time.time() - t0) < 2.0:
                twist = geometry_msgs.msg.Twist()
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = theta
                pub.publish(twist)
                time.sleep(0.05)

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)


if __name__ == '__main__':
    main()
