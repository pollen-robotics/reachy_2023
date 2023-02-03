import sys

import geometry_msgs.msg
import rclpy

from rclpy.node import Node
import pygame
import time
import math
import numpy as np
import traceback
import sys
from rclpy.qos import ReliabilityPolicy, QoSProfile

import os

# To be able to use pygame in "headless" mode
# set SDL to use the dummy NULL video driver, so it doesn't need a windowing system.
os.environ["SDL_VIDEODRIVER"] = "dummy"

msg = """
This node takes inputs from a controller and publishes them
as Twist messages in SI units. Tested on a SONY Dual shock 4 controller
and an XBOX controller.

Left joy: holonomic translations
Right joy: rotation

L2/L1 : increase/decrease only linear speed (additive) +-0.05m/s
R2/R1 : increase/decrease only angular speed (additive) +-0.2rad/s

CTRL-C  or press CIRCLE on the controller to quit
"""

# PS4 controller:
# Button  0 = X
# Button  1 = O
# Button  2 = Triangle
# Button  3 = Square
# Button  4 = l1
# Button  5 = r1
# Button  6 = l2
# Button  7 = r2
# Button  8 = share
# Button  9 = options
# Button 10 = ps_button
# Button 11 = joy_left
# Button 12 = joy_right

# XBOX controller:
# Button  0 = A
# Button  1 = B
# Button  2 = X
# Button  3 = Y
# Button  4 = LB
# Button  5 = RB
# Button  6 = back
# Button  7 = start
# Button  8 = big central button
# LT and RT are axis (like a joy)

# When using the XBOX controller, most of it is the same,
# except that you must use Start and Back to increase the max speeds.


def sign(x):
    if x >= 0:
        return 1
    else:
        return -1


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('zuuu_teleop_joy')
        self.get_logger().info("Starting zuuu_teleop_joy!")

        pygame.init()
        pygame.display.init()
        pygame.joystick.init()

        self.nb_joy = pygame.joystick.get_count()
        if self.nb_joy < 1:
            self.get_logger().error("No controller detected.")
            self.emergency_shutdown()
        self.get_logger().info("nb joysticks: {}".format(self.nb_joy))
        self.j = pygame.joystick.Joystick(0)
        self.lin_speed_ratio = 0.5
        self.rot_speed_ratio = 2.0
        # The joyticks dont come back at a perfect 0 position when released.
        # Any abs(value) below min_joy_position will be assumed to be 0
        self.min_joy_position = 0.03
        self.pub = self.create_publisher(
            geometry_msgs.msg.Twist, 'cmd_vel', 10)
        self.create_timer(0.01, self.main_tick)
        self.get_logger().info(msg)

    def emergency_shutdown(self):
        self.get_logger().warn("Emergency shutdown! Spamming a Twist of 0s!")
        while True:
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            time.sleep(0.01)

    def tick_controller(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.emergency_shutdown()
            elif event.type == pygame.JOYBUTTONDOWN:
                if self.j.get_button(1):
                    self.get_logger().warn("Pressed emergency stop!")
                    self.emergency_shutdown()
                if self.j.get_button(6):  # l2
                    self.lin_speed_ratio = min(3.0, self.lin_speed_ratio+0.05)
                    self.get_logger().info("max translational speed: {:.1f}m/s, max rotational speed: {:.1f}rad/s"
                                           .format(self.lin_speed_ratio*100, self.rot_speed_ratio*100))
                if self.j.get_button(7):  # r2
                    self.rot_speed_ratio = min(12.0, self.rot_speed_ratio+0.2)
                    self.get_logger().info("max translational speed: {:.1f}m/s, max rotational speed: {:.1f}rad/s"
                                           .format(self.lin_speed_ratio*100, self.rot_speed_ratio*100))
                if self.j.get_button(4):  # l1
                    self.lin_speed_ratio = max(0.0, self.lin_speed_ratio-0.05)
                    self.get_logger().info("max translational speed: {:.1f}m/s, max rotational speed: {:.1f}rad/s"
                                           .format(self.lin_speed_ratio*100, self.rot_speed_ratio*100))
                if self.j.get_button(5):  # r1
                    self.rot_speed_ratio = max(0.0, self.rot_speed_ratio-0.2)
                    self.get_logger().info("max translational speed: {:.1f}m/s, max rotational speed: {:.1f}rad/s"
                                           .format(self.lin_speed_ratio*100, self.rot_speed_ratio*100))
            elif event.type == pygame.JOYBUTTONUP:
                pass

        if self.nb_joy != pygame.joystick.get_count():
            self.get_logger().warn("Controller disconnected!")
            self.emergency_shutdown()

    def rumble(self, duration):
        self.rumble_start = time.time()
        self.is_rumble = True
        self.rumble_duration = duration
        # Duration doesn't work, have to do it ourselves
        self.j.rumble(1, 1, 1000)

    def print_controller(self):
        # Get the name from the OS for the controller/joystick.
        name = self.j.get_name()
        self.get_logger().info("Joystick name: {}".format(name))

        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = self.j.get_numaxes()
        self.get_logger().info("Number of axes: {}".format(axes))

        for i in range(axes):
            axis = self.j.get_axis(i)
            self.get_logger().info("Axis {} value: {:>6.3f}".format(i, axis))

        buttons = self.j.get_numbuttons()
        self.get_logger().info("Number of buttons: {}".format(buttons))

        for i in range(buttons):
            button = self.j.get_button(i)
            self.get_logger().info("Button {:>2} value: {}".format(i, button))

    def speeds_from_joystick(self):
        cycle_max_t = self.lin_speed_ratio  # 0.2*factor
        cycle_max_r = self.rot_speed_ratio  # 0.1*factor

        if abs(self.j.get_axis(1)) < self.min_joy_position:
            x = 0.0
        else:
            x = -self.j.get_axis(1) * cycle_max_t

        if abs(self.j.get_axis(0)) < self.min_joy_position:
            y = 0.0
        else:
            y = -self.j.get_axis(0) * cycle_max_t

        if abs(self.j.get_axis(3)) < self.min_joy_position:
            rot = 0.0
        else:
            rot = -self.j.get_axis(3) * cycle_max_r

        return x, y, rot

    def main_tick(self):
        self.get_logger().info("Tick!!")
        self.tick_controller()
        x, y, theta = self.speeds_from_joystick()
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = theta
        self.pub.publish(twist)
        self.get_logger().info("\nx_vel: {:.2f}m/s, y_vel: {:.2f}m/s, theta_vel: {:.2f}rad/s.\n"
                               "Max lin_vel: {:.2f}m/s, max rot_vel: {:.2f}rad/s".format(
                                   x, y, theta, self.lin_speed_ratio, self.rot_speed_ratio))
        # self.print_controller()
        # time.sleep(1.0)


def main():
    rclpy.init()
    node = JoyTeleop()

    try:
        rclpy.spin(node)
    except Exception as e:
        traceback.print_exc()
    finally:
        node.emergency_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
