# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys

import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
Modified version of the default ROS2 keyboard teleop node.
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with an AZERTY keyboard with a
pad num layout :)

Holonomic mode (translations only):
---------------------------
   7    8    9
   4    5    6
   1    2    3

a  : counter-clock wise rotation
e  : clock wise rotation

+- : increase/decrease only linear speed (additive) +-0.05m/s
*/ : increase/decrease only angular speed (additive) +-0.2rad/s

Anything else : stop

CTRL-C to quit
"""

moveBindings = {

    'a': (0, 0, 0, 1),
    'e': (0, 0, 0, -1),
    '9': (1, -1, 0, 0),
    '8': (1, 0, 0, 0),
    '4': (0, 1, 0, 0),
    '6': (0, -1, 0, 0),
    '7': (1, 1, 0, 0),
    '2': (-1, 0, 0, 0),
    '3': (-1, -1, 0, 0),
    '1': (-1, 1, 0, 0),
}

linSpeedBindings = {
    '+': 0.05,
    '-': -0.05,
}

rotSpeedBindings = {
    '*': 0.2,
    '/': -0.2,
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)

    lin_speed = 0.5
    rot_speed = 2.0
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0

    try:
        while True:
            if (status == 0):
                print(msg)
            status = (status + 1) % 15
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = 0.0
                th = moveBindings[key][3]
            elif key in linSpeedBindings.keys():
                lin_speed += linSpeedBindings[key]
            elif key in rotSpeedBindings.keys():
                rot_speed += rotSpeedBindings[key]
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break
            x = x * lin_speed
            y = y * lin_speed
            th = th * rot_speed
            print("\nx_vel: {:.2f}m/s, y_vel: {:.2f}m/s, theta_vel: {:.2f}rad/s.\n"
                  "Max lin_vel: {:.2f}m/s, max rot_vel: {:.2f}rad/s".format(
                      x, y, th, lin_speed, rot_speed))

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = x
            twist.linear.y = y
            twist.linear.z = z
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th
            pub.publish(twist)

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

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
