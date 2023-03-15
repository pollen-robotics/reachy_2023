from typing import Dict

import time
import yaml
from threading import Event, Thread

import numpy as np

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState, InterfaceValue
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from reachy_msgs.msg import Gripper


from .gripper_state import GripperState, DT

# Gripper OPEN/CLOSE position (in rads)
# Defined for the direct orientation
POSITION_LIMIT = (-0.87, 0.35)


class GripperSafeController(Node):
    """Grippers High-level controller node."""

    def __init__(self, controllers_file):
        """Prepare the GripperSafeController node.

        Listen to topics:
        - "/grippers/commands" for new opening/force commands.
        - "/joint_state" for gripper current position.

        Publish to topics:
        - "/gripper_forward_position_controller/commands"

        """
        super().__init__('grippers_controller')
        self.logger = self.get_logger()

        # Topic subscriptions
        self.grippers_sub = self.create_subscription(
            msg_type=Gripper,
            topic='/grippers/commands',
            callback=self.grippers_command_callback,
            qos_profile=5,
        )
        self.logger.info(f'Subscribe to "{self.grippers_sub.topic_name}".')

        self.joint_states_sub = self.create_subscription(
            msg_type=JointState,
            topic='/joint_states',
            callback=self.joint_states_callback,
            qos_profile=5,
        )
        self.logger.info(f'Subscribe to "{self.joint_states_sub.topic_name}".')

        # Gripper state cache setup
        self.grippers: Dict[str, Dict[str, float]] = {}
        self.first_init = Event()

        self.gripper_forward_order = self._parse_controller(controllers_file)

        self.wait_for_setup()
        self.gripper_states = {
            name: GripperState(
                name, is_direct=name.startswith('r'),
                present_position=value['present_position'], 
                user_requested_goal_position=value['user_requested_goal_position'],
            )
            for name, value in self.grippers.items()
        }
        self.limits = {}
        lower, upper = POSITION_LIMIT
        for name, state in self.gripper_states.items():
            open_pos = lower if state.is_direct else -upper
            close_pos = upper if state.is_direct else -lower
            self.limits[name] = (open_pos, close_pos)
        self.logger.info(f'Setup done, basic state: {self.gripper_states} with limits {self.limits}')

        self.last_grippers_pid = {
            name: (np.nan, np.nan, np.nan)
            for name, state in self.gripper_states.items()
        }

        # Gripper command publisher
        self.gripper_forward_publisher = self.create_publisher(
            msg_type=Float64MultiArray,
            topic='/gripper_forward_position_controller/commands',
            qos_profile=5,
        )
        self.logger.info(f'Publish to "{self.gripper_forward_publisher.topic_name}".')

        # PID command publisher
        self.pid_publisher = self.create_publisher(
            msg_type=DynamicJointState,
            topic='/dynamic_joint_commands',
            qos_profile=5,
        )

        # Update thread loop
        def gripper_state_update_thread():
            self.publish_goals()
            self.publish_pids()

            while rclpy.ok():
                self.gripper_state_update()
                time.sleep(DT)

        self.gripper_state_thread = Thread(target=gripper_state_update_thread)
        self.gripper_state_thread.daemon = True
        self.gripper_state_thread.start()

        self.logger.info('Node ready!')

    # Subscription callback
    def grippers_command_callback(self, msg: Gripper):
        """Get latest Gripper msg from /grippers."""
        # TODO: deals with force?

        for name, opening in zip(msg.name, msg.opening):
            if name not in self.grippers:
                self.logger.warning(f'Unhandled gripper "{name}"!')
                continue

            open_pos, close_pos = self.limits[name]
            goal_pos = np.clip(opening, open_pos, close_pos)

            self.grippers[name]['user_requested_goal_position'] = goal_pos

    def joint_states_callback(self, msg: JointState):
        """Get latest JointState from /joint_states."""
        if not self.first_init.is_set():
            self.setup_grippers(msg)
            self.first_init.set()

        for name, position in zip(msg.name, msg.position):
            if name not in self.grippers:
                continue

            self.grippers[name]['present_position'] = position

    # Gripper update loop
    def setup_grippers(self, msg: JointState):
        for name, position in zip(msg.name, msg.position):
            if 'gripper' in name:
                self.grippers[name] = {
                    'present_position': position,
                    'user_requested_goal_position': position,
                }
                self.logger.info(f'Found gripper "{name}". Setup done.')

    def gripper_state_update(self):
        """Update grippers state machine."""
        for name, gripper_state in self.gripper_states.items():
            gripper_state.update(
                new_present_position=self.grippers[name]['present_position'],
                new_user_requested_goal_position=self.grippers[name]['user_requested_goal_position'],
            )

        self.publish_goals()
        self.publish_pids()

    def publish_goals(self):
        """Publish new /*_gripper_forward_position_controller/commands for grippers."""

        data = [0.0] * len(self.gripper_forward_order)
        for name, state in self.gripper_states.items():
            data[self.gripper_forward_order[name]] = state.safe_computed_goal_position

        msg = Float64MultiArray()
        msg.data = data
        self.gripper_forward_publisher.publish(msg)

    def publish_pids(self):
        """Publish new PID requests for grippers."""
        msg = DynamicJointState()

        for name, gripper_state in self.gripper_states.items():
            if gripper_state.pid != self.last_grippers_pid[name]:
                msg.joint_names.append(name)

                iv = InterfaceValue()
                iv.interface_names = ['p_gain', 'i_gain', 'd_gain']
                iv.values = list(gripper_state.pid)

                msg.interface_values.append(iv)

                self.last_grippers_pid[name] = gripper_state.pid

        if msg.joint_names:
            self.pid_publisher.publish(msg)

    def wait_for_setup(self):
        while True:
            rclpy.spin_once(self)
            if self.first_init.wait(timeout=0.1):
                break

    def _parse_controller(self, controllers_file):
        d = {}

        with open(controllers_file, 'r') as f:
            config = yaml.safe_load(f)

            controller_config = config['controller_manager']['ros__parameters']
            forward_controllers = []
            for k, v in controller_config.items():
                if 'gripper' not in k:
                    continue
                try:
                    if v['type'] == 'forward_command_controller/ForwardCommandController':
                        forward_controllers.append(k)
                except (KeyError, TypeError):
                    pass

            for c in forward_controllers:
                joints = config[c]['ros__parameters']['joints']
                d[c] = {
                    j: i for i, j in enumerate(joints)
                }
                
        return d['gripper_forward_position_controller']

def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--ros-args', action='store_true')
    parser.add_argument('--controllers-file')
    args = parser.parse_args()

    """Run gripper controller main loop."""
    rclpy.init()

    gripper = GripperSafeController(controllers_file=args.controllers_file)
    rclpy.spin(gripper)

    rclpy.shutdown()


if __name__ == '__main__':
    main()