import time
from threading import Event, Lock, Thread
from typing import List

import yaml
import numpy as np

from google.protobuf.wrappers_pb2 import FloatValue, UInt32Value, BoolValue
from scipy.spatial.transform import Rotation

from ament_index_python.packages import get_package_share_directory 

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

from reachy_sdk_api.arm_kinematics_pb2 import ArmIKRequest, ArmSide
from reachy_sdk_api.fan_pb2 import FanId, FanState, FansCommand
from reachy_sdk_api.fullbody_cartesian_command_pb2 import FullBodyCartesianCommand
from reachy_sdk_api.joint_pb2 import JointId, JointsCommand, JointState, JointsState, JointField, PIDValue, PIDGains
from reachy_sdk_api.orbita_kinematics_pb2 import OrbitaIKRequest
from reachy_sdk_api.sensor_pb2 import SensorId, SensorState, ForceSensorState


class BodyControlNode(Node):
    def __init__(self, controllers_file):
        super().__init__(node_name='body_control_server_node')
        self.logger = self.get_logger()

        self.forward_controllers = self._parse_controller(controllers_file)

        self.joints = {}
        self.joint_uids = {}

        self.requested_torques = {}
        self.requested_torque_limit = {}
        self.requested_speed_limit = {}
        self.requested_pid = {}
        self.requested_fans = {}

        self.sensors = {}
        self.sensors_uids = {}

        self.fans = {}
        self.fans_uids = {}

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
        self.sensor_state_pub_event = Event()
        self.fan_state_pub_event = Event()

        self.wait_for_setup()

        t = Thread(target=self._publish_joint_command)
        t.daemon = True
        t.start()

        self.torque_need_update = Event()
        self._torque_pub_t = Thread(target=self._publish_torque_update)
        self._torque_pub_t.start()

        self.torque_limit_need_update = Event()
        self._torque_limit_pub_t = Thread(target=self._publish_torque_limit_update)
        self._torque_limit_pub_t.start()

        self.speed_limit_need_update = Event()
        self._speed_limit_pub_t = Thread(target=self._publish_speed_limit_update)
        self._speed_limit_pub_t.start()

        self.pid_need_update = Event()
        self._pid_pub_t = Thread(target=self._publish_pid_update)
        self._pid_pub_t.start()

        self.fan_need_update = Event()
        self._fan_pub_t = Thread(target=self._publish_fan_update)
        self._fan_pub_t.start()

    def wait_for_setup(self):
        while not self.joint_state_ready.is_set():
            self.logger.info('Waiting for /dynamic_joint_states...')
            rclpy.spin_once(self)

    def get_joint_state(self, uid: JointId, joint_fields: JointField) -> JointsState:
        """ Get update info for requested joint.

         - present_position
         - target_position
         - temperature

        And forge a JointsState message with it.
        """
        name = self._get_joint_name(uid)
        values = self.joints[name]

        kwargs = {}

        for field in joint_fields:
            if field == JointField.ALL:
                kwargs = {
                    'name': name,
                    'uid': UInt32Value(value=values['uid']),
                    'present_position': FloatValue(value=values['present_position']),
                    'temperature': FloatValue(value=values['present_temperature']),
                    'goal_position': FloatValue(value=values['target_position']),
                    'pid': PIDValue(pid=PIDGains(p=values['pid']['p'], i=values['pid']['i'], d=values['pid']['d'])),
                    'compliant': BoolValue(value=values['compliant']),
                    'torque_limit': FloatValue(value=values['torque_limit']),
                    'speed_limit': FloatValue(value=values['speed_limit']),
                }
                break

            elif field == JointField.NAME:
                kwargs['name'] = name

            elif field == JointField.PID:
                kwargs['pid'] = PIDValue(pid=PIDGains(p=values['pid']['p'], i=values['pid']['i'], d=values['pid']['d']))

            elif field == JointField.UID:
                kwargs['uid'] = UInt32Value(value=values['uid'])

            elif field == JointField.COMPLIANT:
                kwargs['compliant'] = BoolValue(value=values['compliant'])

            elif field == JointField.TEMPERATURE:
                kwargs['temperature'] = FloatValue(value=values['present_temperature'])

            elif field == JointField.GOAL_POSITION:
                kwargs['goal_position'] = FloatValue(value=values['target_position'])

            elif field == JointField.PRESENT_POSITION:
                kwargs['present_position'] = FloatValue(value=values['present_position'])

            elif field == JointField.TORQUE_LIMIT:
                kwargs['torque_limit'] = FloatValue(value=values['torque_limit'])

            elif field == JointField.SPEED_LIMIT:
                kwargs['speed_limit'] = FloatValue(value=values['speed_limit'])

        return JointState(**kwargs)

    def get_sensor_state(self, uid: SensorId) -> SensorState:
        name = self._get_sensor_name(uid)
        return SensorState(force_sensor_state=ForceSensorState(force=self.sensors[name]['force']))

    def get_fan_state(self, uid: FanId) -> FanState:
        name = self._get_fan_name(uid)
        return FanState(on=bool(self.fans[name]['state']))

    def _on_joint_state(self, state: DynamicJointState):
        """ Retreive the joint state from /dynamic_joint_states.

            Update present_position and temperature.
        """
        # The first time we got the cb
        # There is some specific preparation we need to do
        #   - we create the dict entry
        #   - we set the target_position to the current_position
        if not self.joints:
            for uid, (name, kv) in enumerate(zip(state.joint_names, state.interface_values)):
                if 'position' in kv.interface_names:
                    self.joints[name] = {}
                    self.joints[name]['name'] = name
                    self.joints[name]['uid'] = uid
                    self.joints[name]['pid'] = {}
                    self.joint_uids[uid] = name

                    for k, v in zip(kv.interface_names, kv.values):
                        if k == 'position':
                            self.joints[name]['present_position'] = v
                            self.joints[name]['target_position'] = v
                        elif k == 'temperature':
                            self.joints[name]['present_temperature'] = v
                        elif k == 'p_gain':
                            self.joints[name]['pid']['p'] = v
                        elif k == 'i_gain':
                            self.joints[name]['pid']['i'] = v
                        elif k == 'd_gain':
                            self.joints[name]['pid']['d'] = v
                        elif k == 'torque':
                            self.joints[name]['compliant'] = (v == 0.0)
                        elif k == 'torque_limit':
                            self.joints[name]['torque_limit'] = v
                        elif k == 'max_speed':
                            self.joints[name]['speed_limit'] = v

                elif 'force' in kv.interface_names:
                    self.sensors[name] = {}
                    self.sensors[name]['name'] = name
                    self.sensors[name]['uid'] = uid
                    self.sensors[name]['force'] = kv.values[0]
                    self.sensors_uids[uid] = name

                elif 'state' in kv.interface_names:
                    self.fans[name] = {}
                    self.fans[name]['name'] = name
                    self.fans[name]['uid'] = uid
                    self.fans[name]['state'] = kv.values[0]
                    self.fans_uids[uid] = name

            self.joint_state_ready.set()

        # Normal use case
        for name, kv in zip(state.joint_names, state.interface_values):
            if 'position' in kv.interface_names:
                for k, v in zip(kv.interface_names, kv.values):
                    if k == 'position':
                        self.joints[name]['present_position'] = v
                    elif k == 'temperature':
                        self.joints[name]['present_temperature'] = v
                    elif k == 'torque':
                        self.joints[name]['compliant'] = (v == 0.0)
                    elif k == 'p_gain':
                        self.joints[name]['pid']['p'] = v
                    elif k == 'i_gain':
                        self.joints[name]['pid']['i'] = v
                    elif k == 'd_gain':
                        self.joints[name]['pid']['d'] = v
                    elif k == 'torque_limit':
                        self.joints[name]['torque_limit'] = v
                    elif k == 'max_speed':
                        self.joints[name]['speed_limit'] = v

            elif 'force' in kv.interface_names:
                self.sensors[name]['force'] = kv.values[0]

            elif 'state' in kv.interface_names:
                self.fans[name]['state'] = kv.values[0]

        self.joint_state_pub_event.set()
        self.sensor_state_pub_event.set()
        self.fan_state_pub_event.set()
        
    def _get_joint_name(self, joint_id: JointId) -> str:
        if joint_id.HasField('uid'):
            return self.joint_uids[joint_id.uid]
        else:
            return joint_id.name

    def _get_sensor_name(self, sensor_id: SensorId) -> str:
        if sensor_id.HasField('uid'):
            return self.sensors_uids[sensor_id.uid]
        else:
            return sensor_id.name

    def _get_fan_name(self, fan_id: FanId) -> str:
        if fan_id.HasField('uid'):
            return self.fans_uids[fan_id.uid]
        else:
            return fan_id.name

    def _get_joint_uid(self, joint_id: JointId) -> int:
        if joint_id.HasField('uid'):
            return joint_id.uid
        else:
            return self.joints[joint_id.name]['uid']

    def _parse_controller(self, controllers_file):
        d = {}
        controllers_file_folder_path = get_package_share_directory('reachy_bringup') + '/config/'

        try: 
            with open(f'{controllers_file_folder_path+controllers_file}.yaml', 'r') as f:
                self.logger.info(f'Using reachy_description/ros2_control/{controllers_file}.yaml controller file.')
                config = yaml.safe_load(f)

                controller_config = config['controller_manager']['ros__parameters']
                forward_controllers = []
                for k, v in controller_config.items():
                    try:
                        if v['type'] == 'forward_command_controller/ForwardCommandController':
                            forward_controllers.append(k)
                        elif v['type'] == 'pid_command_controller/PIDCommandController':
                            forward_controllers.append(k)
                    except (KeyError, TypeError):
                        pass

                for c in forward_controllers:
                    joints = config[c]['ros__parameters']['joints']
                    d[c] = {
                        j: i for i, j in enumerate(joints)
                    }

            return d
        except FileNotFoundError:
            self.logger.error(f'Controller file {controllers_file}.yaml does not exist in reachy_description/ros2_control.')
            import sys
            sys.exit()

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
                self.requested_torques[name] = comp

        if need_update:
            self.torque_need_update.set()

    def _update_torque_limit(self, req: JointsCommand):
        need_update = False

        for cmd in req.commands:
            if cmd.HasField('torque_limit'):
                need_update = True
                name = self._get_joint_name(cmd.id)
                comp = cmd.torque_limit.value
                self.requested_torque_limit[name] = comp

        if need_update:
            self.torque_limit_need_update.set()

    def _update_speed_limit(self, req: JointsCommand):
        need_update = False

        for cmd in req.commands:
            if cmd.HasField('speed_limit'):
                need_update = True
                name = self._get_joint_name(cmd.id)
                comp = cmd.speed_limit.value
                self.requested_speed_limit[name] = comp

        if need_update:
            self.speed_limit_need_update.set()

    def _update_pid(self, req: JointsCommand):
        need_update = False

        for cmd in req.commands:
            if cmd.HasField('pid'):
                need_update = True
                name = self._get_joint_name(cmd.id)
                self.requested_pid[name] = {
                    'p': cmd.pid.pid.p,
                    'i': cmd.pid.pid.i,
                    'd': cmd.pid.pid.d,
                }

        if need_update:
            self.pid_need_update.set()

    def handle_fan_msg(self, grpc_req: FansCommand):
        need_update = False

        for cmd in grpc_req.commands:
            need_update = True
            name = self._get_fan_name(cmd.id)
            self.requested_fans[name] = float(cmd.on)

        if need_update:
            self.fan_need_update.set()

    def handle_joint_msg(self, grpc_req: JointsCommand):
        self._update_joint_target_pos(grpc_req)
        self._update_torque(grpc_req)
        self._update_torque_limit(grpc_req)
        self._update_speed_limit(grpc_req)
        self._update_pid(grpc_req)

    def _publish_joint_command(self):
        while rclpy.ok():
            for controller, joint_dic in self.forward_controllers.items():
                if controller in ('forward_torque_controller', 'forward_torque_limit_controller',
                                  'forward_speed_limit_controller', 'pid_controller', 'forward_fan_controller'):
                    continue
                pos = [self.joints[joint]['target_position'] for joint in joint_dic.keys()]
                self.forward_publishers[controller].publish(Float64MultiArray(data=pos))

            time.sleep(0.01)
    
    def _publish_torque_update(self):
        while rclpy.ok():
            self.torque_need_update.wait()
            self.torque_need_update.clear()

            for joint, torque in self.requested_torques.items():
                self.joints[joint]['compliant'] = torque

            self.requested_torques.clear()

            joint_dic = self.forward_controllers['forward_torque_controller']
            torque_data = [float(not self.joints[joint]['compliant']) for joint in joint_dic.keys()]

            self.forward_publishers['forward_torque_controller'].publish(
                Float64MultiArray(
                    data=torque_data
                )
            )

    def _publish_torque_limit_update(self):
        while rclpy.ok():
            self.torque_limit_need_update.wait()
            self.torque_limit_need_update.clear()

            for joint, torque_limit in self.requested_torque_limit.items():
                self.joints[joint]['torque_limit'] = torque_limit

            self.requested_torque_limit.clear()

            joint_dic = self.forward_controllers['forward_torque_limit_controller']
            torque_limit_data = [self.joints[joint]['torque_limit'] for joint in joint_dic.keys()]

            self.forward_publishers['forward_torque_limit_controller'].publish(
                Float64MultiArray(
                    data=torque_limit_data
                )
            )

    def _publish_speed_limit_update(self):

        while rclpy.ok():
            self.speed_limit_need_update.wait()
            self.speed_limit_need_update.clear()

            for joint, speed_limit in self.requested_speed_limit.items():
                self.joints[joint]['speed_limit'] = speed_limit
            self.requested_speed_limit.clear()

            joint_dic = self.forward_controllers['forward_speed_limit_controller']
            speed_limit_data = [self.joints[joint]['speed_limit'] for joint in joint_dic.keys()]

            self.forward_publishers['forward_speed_limit_controller'].publish(
                Float64MultiArray(
                    data=speed_limit_data
                )
            )

    def _publish_pid_update(self):
        while rclpy.ok():
            self.pid_need_update.wait()
            self.pid_need_update.clear()

            for joint, pid in self.requested_pid.items():
                self.joints[joint]['pid'] = pid

            self.requested_pid.clear()

            joint_dic = self.forward_controllers['pid_controller']
            pid_data = [
                [
                    self.joints[joint]['pid']['p'], 
                    self.joints[joint]['pid']['i'], 
                    self.joints[joint]['pid']['d'], 
                ]
                for joint in joint_dic.keys()
            ]

            self.forward_publishers['pid_controller'].publish(
                Float64MultiArray(
                    data=list(np.array(pid_data).ravel())
                )
            )

    def _publish_fan_update(self):
        while rclpy.ok():
            self.fan_need_update.wait()
            self.fan_need_update.clear()

            for fan, state in self.requested_fans.items():
                self.fans[fan]['state'] = state

            self.requested_fans.clear()

            fan_dic = self.forward_controllers['forward_fan_controller']
            fan_data = [self.fans[fan]['state'] for fan in fan_dic.keys()]

            self.forward_publishers['forward_fan_controller'].publish(
                Float64MultiArray(
                    data=fan_data
                )
            )
