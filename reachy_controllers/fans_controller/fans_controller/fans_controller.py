# Reachy Fan Controller
# This script implements a ROS 2 node that controls Reachy's fans based on motor temperatures.

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState, InterfaceValue

from reachy_utils.config import get_fan_trigger_temperature, get_reachy_model
from reachy_utils.discovery import robot_config_to_parts


fans_per_part = {
    'head': ['l_antenna_fan', 'r_antenna_fan'],
    'left_arm': ['l_shoulder_fan', 'l_elbow_fan', 'l_wrist_fan'],
    'right_arm': ['r_shoulder_fan', 'r_elbow_fan', 'r_wrist_fan'],
}

motors_to_monitor_per_part = {
    'head': ['l_antenna', 'r_antenna'],
    'left_arm': ['l_shoulder_pitch', 'l_elbow_pitch', 'l_wrist_pitch'],
    'right_arm': ['r_shoulder_pitch', 'r_elbow_pitch', 'r_wrist_pitch'],
}


class FansController(Node):
    """
    Fans controller class.
    Inherits from the rclpy.node.Node class to create a ROS 2 node.
    """
    def __init__(self, node_name):
        super().__init__(node_name)
        self._logger = self.get_logger()

        robot_parts = robot_config_to_parts[get_reachy_model()]

        self._fan_trigger_temperature = get_fan_trigger_temperature()
        if self._fan_trigger_temperature == -1:
            self.get_logger().error('Fan trigger temperature is not defined.')
            exit()

        self._logger.info(f'Fan trigger temperature is {self._fan_trigger_temperature}°C.')

        self._fan_states = {}
        self._fan_states_to_publish = []
        self._motor_temperatures = {}

        for part in robot_parts:
            if part == 'orbita':
                continue

            for motor in motors_to_monitor_per_part[part]:
                self._motor_temperatures[motor] = 0.0

            for fan in fans_per_part[part]:
                self._fan_states[fan] = False

        self._joint_state_sub = self.create_subscription(
            DynamicJointState,
            'dynamic_joint_states',
            self._on_joint_state,
            5)

        self._joint_state_pub = self.create_publisher(
            DynamicJointState,
            'dynamic_joint_commands',
            5)

        self._logger.info('Fans controller node started.')

        self._handle_first_command()
        self._timer = self.create_timer(1, self._on_timer)

    def _on_joint_state(self, msg):
        """
        Callback function for the 'dynamic_joint_states' subscription.
        Updates the motor temperatures based on the received joint states.
        """
        for (joint, interface) in zip(msg.joint_names, msg.interface_values):
            if joint in self._motor_temperatures:
                for (interface_name, value) in zip(interface.interface_names, interface.values):
                    if interface_name == 'temperature':
                        self._motor_temperatures[joint] = value

    def _on_timer(self):
        """
        Timer callback function called periodically.
        Checks the motor temperatures and updates the fan states accordingly.
        Publishes the fan states if there are changes.
        """
        for motor, temperature in self._motor_temperatures.items():
            motor = motor.split('_pitch')[0]

            if temperature >= self._fan_trigger_temperature and not self._fan_states[f'{motor}_fan']:
                self._fan_states[f'{motor}_fan'] = True
                self._fan_states_to_publish.append(f'{motor}_fan')

            elif temperature < self._fan_trigger_temperature - 2 and self._fan_states[f'{motor}_fan']:
                self._fan_states[f'{motor}_fan'] = False
                self._fan_states_to_publish.append(f'{motor}_fan')

        if self._fan_states_to_publish:
            msg = DynamicJointState()
            msg.joint_names = self._fan_states_to_publish
            msg.interface_values = [
                InterfaceValue(
                    interface_names=['state'],
                    values=[float(self._fan_states[fan])],
                )
                for fan in self._fan_states_to_publish
            ]

            self._joint_state_pub.publish(msg)

            for fan in self._fan_states_to_publish:
                state = 'ON' if self._fan_states[fan] else 'OFF'
                self._logger.info(f'Turning {state} {fan}.')

            self._fan_states_to_publish = []

    def _handle_first_command(self):
        """
        Handles the first command upon initialization.
        Sets the initial fan states based on motor temperatures and publishes the fan states.
        """
        for motor, temperature in self._motor_temperatures.items():
            motor = motor.split('_pitch')[0]

            if temperature >= self._fan_trigger_temperature:
                self._fan_states[f'{motor}_fan'] = True

            elif temperature < self._fan_trigger_temperature - 2 and self._fan_states[f'{motor}_fan']:
                self._fan_states[f'{motor}_fan'] = False

        msg = DynamicJointState()
        msg.joint_names = list(self._fan_states.keys())
        msg.interface_values = [
            InterfaceValue(
                interface_names=['state'],
                values=[float(state)],
            )
            for state in self._fan_states.values()
        ]
        self._joint_state_pub.publish(msg)
        self._logger.info('Sent initial fan command:')
        for (fan, state) in self._fan_states.items():
            state = 'ON' if state else 'OFF'
            self._logger.info(f'    {fan} is {state}')


def main(args=None):
    """
    Main function to initialize the ROS 2 node and start the fans controller.
    """
    rclpy.init(args=args)
    node = FansController('fans_controller')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
