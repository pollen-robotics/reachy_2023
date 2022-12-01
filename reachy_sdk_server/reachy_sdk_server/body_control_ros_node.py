import time
from threading import Event, Lock, Thread
from typing import List

import yaml
import numpy as np

from google.protobuf.wrappers_pb2 import FloatValue, UInt32Value
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

from reachy_msgs.msg import Gripper

from reachy_sdk_api.arm_kinematics_pb2 import ArmIKRequest, ArmSide
from reachy_sdk_api.fullbody_cartesian_command_pb2 import FullBodyCartesianCommand
from reachy_sdk_api.joint_pb2 import JointId, JointsCommand, JointState, JointsState
from reachy_sdk_api.orbita_kinematics_pb2 import OrbitaIKRequest


class BodyControlNode(Node):
    def __init__(self, controllers_file):
        super().__init__(node_name='body_control_server_node')
        self.logger = self.get_logger()

        self.forward_controllers = self._parse_controller(controllers_file)

        self.joints = {}
        self.joint_uids = {}

        self.torques = {}
        self.requested_torques = {}

        # Subscribe to: 
        #  - /dynamic_joint_states (for present_position, torque and temperature)
        #  - /neck_forward_position_controller/commands for (neck roll pitch yaw) target_position
        #  - TODO: where to get arm target_position?
        self.joint_state_ready = Event()
        self.joint_state_sub = self.create_subscription(
            msg_type=DynamicJointState, 
            topic='/dynamic_joint_states',
            qos_profile=5,
            callback=self._on_joint_state,
        )

        # Publish to each controllers
        self.forward_publishers = {
            c: self.create_publisher(
                msg_type=Float64MultiArray, 
                topic=f'/{c}/commands',
                qos_profile=5,
            )
            for c in self.forward_controllers
        }

        self.joint_state_pub_event = Event()

        self.wait_for_setup()

        t = Thread(target=self._publish_joint_command)
        t.daemon = True
        t.start()

        self.torque_need_update = Event()
        self._torque_pub_t = Thread(target=self._publish_torque_update)
        self._torque_pub_t.start()

    def wait_for_setup(self):
        while not self.joint_state_ready.is_set():
            self.logger.info('Waiting for /dynamic_joint_states...')
            rclpy.spin_once(self)

    def get_joint_state(self, uid: JointId, full=False) -> JointsState:
        """ Get update info for requested joint.

         - present_position
         - target_position
         - temperature

        And forge a JointsState message with it.
        """
        name = self._get_joint_name(uid)
        values = self.joints[name]

        kwargs = {
            'present_position': FloatValue(value=values['present_position']),
            'temperature': FloatValue(value=values['present_temperature']),
            'goal_position': FloatValue(value=values['target_position']),
        }

        if full:
            kwargs['name'] = name
            kwargs['uid'] = UInt32Value(value=uid)

        return JointState(**kwargs)

    def _on_joint_state(self, state: DynamicJointState):
        """ Retreive the joint state from /dynamic_joint_states.

            Update present_position and temperature.
        """
        # The first time we got the cb
        # There is some specific preparation we need to do
        #   - we create the dict entry
        #   - we set the target_position to the current_position
        if not self.torques:
            self.torques = {joint: True for joint in self.forward_controllers['forward_torque_controller'].keys()}

        if not self.joints:
            for uid, (name, kv) in enumerate(zip(state.joint_names, state.interface_values)):
                if 'position' in kv.interface_names:
                    self.joints[name] = {}
                    self.joints[name]['uid'] = uid
                    self.joint_uids[uid] = name

                    for k, v in zip(kv.interface_names, kv.values):
                        if k == 'position':
                            self.joints[name]['present_position'] = v
                            self.joints[name]['target_position'] = v

                        elif k == 'temperature':
                            self.joints[name]['present_temperature'] = v

            self.joint_state_ready.set()

        # Normal use case
        for name, kv in zip(state.joint_names, state.interface_values):
            for k, v in zip(kv.interface_names, kv.values):
                if k == 'position':
                    self.joints[name]['present_position'] = v
                elif k == 'temperature':
                    self.joints[name]['present_temperature'] = v
                elif k == 'torque':
                    self.torques[name] = (v == 0.0)

        self.joint_state_pub_event.set()
        
    def _get_joint_name(self, joint_id: JointId) -> str:
        if joint_id.HasField('uid'):
            return self.joint_uids[joint_id.uid]
        else:
            return joint_id.name
    
    def _get_joint_uid(self, joint_id: JointId) -> int:
        if joint_id.HasField('uid'):
            return joint_id.uid
        else:
            return self.joints[joint_id.name]['uid']

    def _parse_controller(self, controllers_file):
        d = {}

        with open(controllers_file, 'r') as f:
            config = yaml.safe_load(f)

            controller_config = config['controller_manager']['ros__parameters']
            forward_controllers = []
            for k, v in controller_config.items():
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

        return d
    
    def _update_joint_target_pos(self, req: JointsCommand):
        for cmd in req.commands:
            if cmd.HasField('goal_position'):
                self.joints[self._get_joint_name(cmd.id)]['target_position'] = cmd.goal_position.value

    def _update_torque(self, req: JointsCommand):
        need_update = False

        for cmd in req.commands:
            if cmd.HasField('compliant'):
                need_update = True
                name = self._get_joint_name(cmd.id)
                comp = cmd.compliant.value
                self.requested_torques[name] = not comp

        if need_update:
            self.torque_need_update.set()

    def handle_joint_msg(self, grpc_req: JointsCommand):
        self._update_joint_target_pos(grpc_req)
        self._update_torque(grpc_req)

    def _publish_joint_command(self):
        while rclpy.ok():
            for controller, joint_dic in self.forward_controllers.items():
                if controller == 'forward_torque_controller':
                    continue
                pos = [self.joints[joint]['target_position'] for joint in joint_dic.keys()]
                self.forward_publishers[controller].publish(Float64MultiArray(data=pos))

            time.sleep(0.01)
    
    def _publish_torque_update(self):
        while rclpy.ok():
            self.torque_need_update.wait()
            self.torque_need_update.clear()

            self.torques.update(self.requested_torques)
            self.requested_torques.clear()

            torque_data = [float(not t) for t in self.torques.values()]

            self.forward_publishers['forward_torque_controller'].publish(
                Float64MultiArray(
                    data=torque_data
                )
            )

