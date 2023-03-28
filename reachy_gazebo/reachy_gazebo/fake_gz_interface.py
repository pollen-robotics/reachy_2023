#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
#  File Name	: fake_gz_interface.py
#  Author	: Steve NGUYEN
#  Contact      : steve.nguyen@pollen-robotics.com
#  Created	: lundi, janvier 23 2023
#  Revised	:
#  Version	:
#  Target MCU	:
#
#  Notes:	notes
#

import time
from functools import partial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from rcl_interfaces.msg import ParameterDescriptor

from control_msgs.msg import DynamicJointState, InterfaceValue
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

from reachy_msgs.srv import GetCameraZoomLevel, GetCameraZoomSpeed
from reachy_msgs.srv import SetCameraZoomLevel, SetCameraZoomSpeed
from reachy_msgs.srv import GetCameraZoomFocus, SetCameraZoomFocus
from reachy_msgs.srv import SetFocusState

DUMMY_JOINT_INTERFACE_NAMES = [
    'torque',
    'p_gain', 'i_gain', 'd_gain',
    'temperature',
    'speed_limit',
    'torque_limit',
]

DUMMY_SPECIAL_INTERFACES = {
    'full_kit': {
        'l_shoulder_fan': 'state',
        'l_elbow_fan': 'state',
        'l_wrist_fan': 'state',
        'l_antenna_fan': 'state',
        'l_force_gripper': 'force',
        'r_shoulder_fan': 'state',
        'r_elbow_fan': 'state',
        'r_wrist_fan': 'state',
        'r_antenna_fan': 'state',
        'r_force_gripper': 'force',
    },
    'headless': {
        'l_shoulder_fan': 'state',
        'l_elbow_fan': 'state',
        'l_wrist_fan': 'state',
        'l_force_gripper': 'force',
        'r_shoulder_fan': 'state',
        'r_elbow_fan': 'state',
        'r_wrist_fan': 'state',
        'r_force_gripper': 'force',
    },
    'starter_kit_right': {
        'l_antenna_fan': 'state',
        'r_elbow_fan': 'state',
        'r_wrist_fan': 'state',
        'r_antenna_fan': 'state',
        'r_force_gripper': 'force',
    },
    'starter_kit_left': {
        'l_shoulder_fan': 'state',
        'l_elbow_fan': 'state',
        'l_wrist_fan': 'state',
        'l_antenna_fan': 'state',
        'l_force_gripper': 'force',
        'r_antenna_fan': 'state',
    }
}


class FakeGzInterface(Node):
    def __init__(self):
        super().__init__('fake_gz_interface')
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.dyn_publisher = self.create_publisher(
            DynamicJointState, '/dynamic_joint_states', qos_profile=latching_qos)

        self.js_publisher = self.create_publisher(
            JointState, '/joint_states', qos_profile=latching_qos)
        param_descriptor = ParameterDescriptor(
            description='The robot configuration.')

        self.declare_parameter('robot_config', 'full_kit', param_descriptor)

        self.logger = self.get_logger()
        self.robot_config = self.get_parameter('robot_config').value
        self.logger.warning(f'Robot config: {self.robot_config}')

        self.joints_list = []
        self.dyn_subscription = self.create_subscription(
            DynamicJointState,
            '/joint_state_broadcaster/dynamic_joint_states',
            self.dyn_state_callback,
            10)

        self.js_subscription = self.create_subscription(
            JointState,
            '/joint_state_broadcaster/joint_states',
            self.joint_state_callback,
            10)

        if self.robot_config in ['full_kit', 'starter_kit_left']:
            self.l_gripper_force_sub = self.create_subscription(
                msg_type=WrenchStamped,
                topic='/l_force_sensor/l_gripper_ft_sensor',
                qos_profile=5,
                callback=partial(self.force_cb, side='l'))
        if self.robot_config in ['full_kit', 'starter_kit_right']:
            self.r_gripper_force_sub = self.create_subscription(
                msg_type=WrenchStamped,
                topic='/r_force_sensor/r_gripper_ft_sensor',
                qos_profile=5,
                callback=partial(self.force_cb, side='r'))

        self._curr_l_force = 0.0
        self._curr_r_force = 0.0
        self.logger.info(f'Fake Gazebo interface for /dynamic_joint_states and /joint_states and camera services')

    def force_cb(self, msg, side):
        # We simulate the gripper force sensor using the Gazebo plugin ft_sensor (force sensor plugin seems broken...)
        if side == 'l':
            self._curr_l_force = msg.wrench.force.z
        elif side == 'r':
            self._curr_r_force = msg.wrench.force.z

    def joint_state_callback(self, msg):
        # Nothing to do here (for now), just forward the message
        self.js_publisher.publish(msg)

    def dyn_state_callback(self, msg):

        # Here we just add the extra fake joints (like fans) and interfaces not handleled by Gazebo (like pid)
        joints = []
        joint_interface = {}
        dummy_joint_interface_present = False

        for j, interface in zip(msg.joint_names, msg.interface_values):
            joints.append(j)
            interfaces = {}
            if any(i in interface.interface_names for i in DUMMY_JOINT_INTERFACE_NAMES):
                dummy_joint_interface_present = True  # we assume that if there is one special interface, they are all present
            for interface_name, value in zip(interface.interface_names, interface.values):
                interfaces[interface_name] = value
            joint_interface[j] = interfaces

        fake = DynamicJointState()
        should_publish = False
        if not any(i in joints for i in list(DUMMY_SPECIAL_INTERFACES[self.robot_config].keys())):  # there is none of the special interface
            # add dummy special interfaces
            for k, v in DUMMY_SPECIAL_INTERFACES[self.robot_config].items():
                fake.joint_names.append(k)
                inter = InterfaceValue()
                inter.interface_names.append(v)
                if k == 'r_force_gripper':
                    inter.values.append(self._curr_r_force)
                elif k == 'l_force_gripper':
                    inter.values.append(self._curr_l_force)
                else:
                    inter.values.append(0.0)
                fake.interface_values.append(inter)
            should_publish = True
        if not dummy_joint_interface_present:  # there is non of the special interface for the joint
            # add dummy joint interfaces
            for j in joints:
                fake.joint_names.append(j)
                inter = InterfaceValue()
                for it_name in DUMMY_JOINT_INTERFACE_NAMES:
                    inter.interface_names.append(it_name)
                    inter.values.append(0.0)
                # add standard joint interfaces
                for it_name in joint_interface[j].keys():
                    inter.interface_names.append(it_name)
                    inter.values.append(joint_interface[j][it_name])

                fake.interface_values.append(inter)
            should_publish = True

        if should_publish:
            fake.header.stamp = self.get_clock().now().to_msg()
            self.dyn_publisher.publish(fake)


def main(args=None):
    rclpy.init(args=args)

    publisher = FakeGzInterface()

    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
