""" Easy access of any joint, sensor, gpio state/command interfaces.

High-level ROS interface on top of ROS2 control.
It lets you easily set a command interfaces for a single joint of a forward controller. 

Service:
- /get_dynamic_state (GetDynamicState) - retrieve any state(s) interface(s) for a specific joint/sensor/gpio

Publication:
- /joint_commands (JointState) for each joints (100Hz)

Subscription:
- /dynamic_joint_commands (DynamicJointState) - set any command(s) interface(s) for one or many joint/sensor/gpio


In more details, it:
- Subscribes to /dynamic_joint_states to get current values
- Publishes to corresponding /*_forward_controller (by automatically computing the diff)
- Specifically handles grippers commands (due to SafeGripperController)

"""
from collections import defaultdict
from functools import partial
from threading import Event, Lock, Thread

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from reachy_msgs.msg import Gripper
from reachy_msgs.srv import GetDynamicState


from .forward_controller import ForwardControllersPool


class DynamicStateRouterNode(Node):
    def __init__(self, node_name, controllers_file):
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()

        self.forward_controllers = ForwardControllersPool.parse(self.logger, controllers_file)

        # Subscribe to /dynamic_joint_states
        self.joint_state_ready = Event()
        self.joint_state = {}
        self.joint_state_sub = self.create_subscription(
            msg_type=DynamicJointState,
            topic='/dynamic_joint_states',
            qos_profile=5,
            callback=self.on_dynamic_joint_states,
        )

        self.joint_command = {}

        # We wait to retrieve all setup info
        self.wait_for_setup()

        # Subscribe to each /forward_position_controller so we got target position updates
        self.forward_position_controller_sub = {
            fc.name: self.create_subscription(
                msg_type=Float64MultiArray,
                topic=f'/{fc.name}/commands',
                qos_profile=5,
                callback=partial(self.on_forward_position_controller_update, controller_name=fc.name),
            )
            for fc in self.forward_controllers.get_controllers_for_interface('position') 
        }

        # Now we start our own service, publication and subscription

        # SERVICE: /get_dynamic_state (GetDynamicState)
        self.get_dyn_state_service = self.create_service(
            srv_type=GetDynamicState, 
            srv_name='/get_dynamic_state',
            callback=self.get_dyn_state_cb,
        )

        # PUBLICATION: /joint_commands (JointState)
        self.joint_commands_pub = self.create_publisher(
            msg_type=JointState,
            topic='/joint_commands',
            qos_profile=5,
        )
        self.joint_commands_timer = self.create_timer(
            timer_period_sec=0.01, 
            callback=self.publish_joint_commands,
        )

        # SUBSCRIPTION: /dynamic_joint_commands (DynamicJointState)
        # We need to get publisher to all forward controllers and the gripper safe controller
        self.joint_command_request_pub = Event()
        self.requested_commands = defaultdict(dict)
        self.pub_lock = Lock()
        self.publish_command_t = Thread(target=self.publish_command_loop)
        self.publish_command_t.daemon = True
        self.publish_command_t.start()

        self.fc_publisher = {
            fc.name: self.create_publisher(
                msg_type=Float64MultiArray,
                topic=f'/{fc.name}/commands',
                qos_profile=5,
            )
            
            for fc in self.forward_controllers.all()
        }
        self.gripper_pub = self.create_publisher(
            msg_type=Gripper,
            topic='/grippers/commands',
            qos_profile=5,
        )

        self.joint_command_sub = self.create_subscription(
            msg_type=DynamicJointState,
            topic='/dynamic_joint_commands',
            qos_profile=5,
            callback=self.on_dynamic_joint_commands,
        )

    # Service: GetDynamicState
    def get_dyn_state_cb(self, request: GetDynamicState.Request, response: GetDynamicState.Response) -> GetDynamicState.Response:
        """ Conveniant service to get interface values of a single joint/sensor/gpio. """
        if request.name not in self.joint_state.keys():
            self.logger.warning(f"Name should be one of {list(self.joint_state.keys())} (got '{request.name}' instead)")
            return response

        response.name = request.name
        for interface in request.interfaces:
            joint = self.joint_state[request.name]

            if interface not in joint:
                possible_interfaces = list(joint.keys())
                self.logger.warning(f"Interface should be one of {possible_interfaces} (got '{interface}' instead)")
                continue

            response.interfaces.append(interface)
            response.values.append(joint[interface])

        return response

    # Subscription: dynamic_joint_commands (DynamicJointState)
    def on_dynamic_joint_commands(self, command: DynamicJointState):
        """ Retreive the joint commands from /dynamic_joint_commands."""
        with self.pub_lock:
            for name, iv in zip(command.joint_names, command.interface_values):
                if name not in self.joint_state:
                    self.logger.warning(f'Unknown joint "{name}" ({list(self.joint_state.keys())})')
                    continue

                for k, v in zip(iv.interface_names, iv.values):
                    if k not in self.joint_state[name]:
                        self.logger.warning(f'Unknown interface for joint "{k}" ({list(self.joint_state[name].keys())})')
                        continue

                    self.requested_commands[name][k] = v

        self.joint_command_request_pub.set()

    def publish_command_loop(self):
        while rclpy.ok():
            self.joint_command_request_pub.wait()

            with self.pub_lock:
                self.handle_commands(self.requested_commands)
                self.requested_commands.clear()
                self.joint_command_request_pub.clear()

    def publish_joint_commands(self):
        msg = JointState()

        for j in self.joint_state.values():
            if 'target_position' in j:
                msg.name.append(j['name'])
                msg.position.append(j['target_position'])

        self.joint_commands_pub.publish(msg)

    def handle_commands(self, commands):
        gripper_commands = defaultdict(dict)
        regular_commands = defaultdict(dict)
        pid_commands = defaultdict(dict)

        commands = self.patch_orbita_commands(commands)

        for joint, iv in commands.items():
            for interface, value in iv.items():
                if joint.endswith('gripper') and interface == 'position':
                    gripper_commands[joint].update({interface: value})
                elif interface in ('p_gain', 'i_gain', 'd_gain'):
                    pid_commands[joint].update({interface: value})
                else:
                    regular_commands[joint].update({interface: value})

        self.handle_gripper_commands(gripper_commands)
        self.handle_pid_commands(pid_commands)
        self.handle_regular_commands(regular_commands)

    def handle_gripper_commands(self, commands):
        msg = Gripper()

        for joint, iv in commands.items():
            for interface, value in iv.items():
                if interface == 'position':
                    msg.name.append(joint)
                    msg.opening.append(value)

        self.gripper_pub.publish(msg)  

    def handle_pid_commands(self, commands):
        pid_fc = self.forward_controllers['forward_pid_controller']
        msg = Float64MultiArray()

        for j in pid_fc.joints:
            for gain in ('p_gain', 'i_gain', 'd_gain'):
                if j in commands and gain in commands[j]:
                    msg.data.append(commands[j][gain])
                elif j in self.joint_command and gain in self.joint_command[j]:
                    msg.data.append(self.joint_command[j][gain])
                else:
                    msg.data.append(self.joint_state[j][gain])

        for j, gains in commands.items():
            for g, val in gains.items():
                self.joint_command[j][g] = val

        self.fc_publisher['forward_pid_controller'].publish(msg)

    def handle_regular_commands(self, commands): 
        # Group commands by forward controller
        to_pub = defaultdict(dict)
        for joint, iv in commands.items():
            for interface, value in iv.items():
                fc = self.forward_controllers.get_corresponding_controller(joint, interface)
                to_pub[fc][joint] = value

        for fc, new_cmd in to_pub.items():
            msg = Float64MultiArray()
            
            pub_interface = fc.interface if fc.interface != 'position' else 'target_position'

            msg.data = []
            for j in fc.joints:
                if j in new_cmd:
                    msg.data.append(new_cmd[j])
                elif pub_interface in self.joint_command[j]:
                    msg.data.append(self.joint_command[j][pub_interface])
                else:
                    msg.data.append(self.joint_state[j][pub_interface])

            for j, val in new_cmd.items():
                self.joint_command[j][pub_interface] = val

            self.fc_publisher[fc.name].publish(msg)
                

    # Internal ROS2 subscription
    # Subscription: DynamicJointState cb
    def on_dynamic_joint_states(self, state: DynamicJointState):
        """ Retreive the joint state from /dynamic_joint_states."""
        if not self.joint_state_ready.is_set():
            for uid, name in enumerate(state.joint_names):
                self.joint_state[name] = {}
                self.joint_state[name]['name'] = name
                self.joint_state[name]['uid'] = uid

                self.joint_command[name] = {}

        for uid, (name, kv) in enumerate(zip(state.joint_names, state.interface_values)):
            for k, v in zip(kv.interface_names, kv.values):
                self.joint_state[name][k] = v

        if not self.joint_state_ready.is_set():
            for name, state in self.joint_state.items():
                if 'position' in state:
                    state['target_position'] = state['position']

            self.joint_state_ready.set()
        
    def on_forward_position_controller_update(self, msg: Float64MultiArray, controller_name: str):
        fc = self.forward_controllers[controller_name]

        for j, v in zip(fc.joints, msg.data):
            self.joint_state[j]['target_position'] = v

    def wait_for_setup(self):
        while not self.joint_state_ready.is_set():
            self.logger.info('Waiting for /dynamic_joint_states...')
            rclpy.spin_once(self)
        self.logger.info('Setup done!')

    def patch_orbita_commands(self, commands):
        patched_commands = defaultdict(dict)

        for joint, iv in commands.items():
            if _is_orbita_joint(joint):
                other = _other_orbita_joint(joint)

                for reg, val in iv.items():
                    if reg in ('torque', 'p_gain', 'i_gain', 'd_gain', 'speed_limit'):
                        for o in other:
                            if o not in commands or reg not in commands[o]:
                                patched_commands[o][reg] = val

            patched_commands[joint] = iv

        return patched_commands


def _is_orbita_joint(joint):
    return joint.startswith('neck_')

def _other_orbita_joint(joint):
    prefix, suffix = joint.split('_')

    other = []
    for s in ('roll', 'pitch', 'yaw'):
        if suffix != s:
            other.append(f'{prefix}_{s}')

    return other

def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--ros-args', action='store_true')
    parser.add_argument('controllers_file')
    args = parser.parse_args()

    rclpy.init()
    node = DynamicStateRouterNode(
        node_name='dynanic_state_router_node',
        controllers_file=args.controllers_file,
    )
    rclpy.spin(node)


if __name__ == '__main__':
    main()