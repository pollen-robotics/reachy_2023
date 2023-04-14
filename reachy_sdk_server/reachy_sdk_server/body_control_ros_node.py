from threading import Event, Lock
from typing import Optional

from google.protobuf.wrappers_pb2 import FloatValue, UInt32Value, BoolValue

import rclpy
from rclpy.node import Node

from control_msgs.msg import DynamicJointState, InterfaceValue
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState

from reachy_msgs.srv import GetForwardKinematics, GetInverseKinematics

from reachy_sdk_api.arm_kinematics_pb2 import (
    ArmEndEffector, ArmSide, ArmJointPosition,
    ArmFKRequest, ArmFKSolution,
    ArmIKRequest, ArmIKSolution,
)

from reachy_sdk_api.head_kinematics_pb2 import HeadIKRequest, HeadFKRequest, HeadIKSolution, HeadFKSolution

from reachy_sdk_api import (
    fan_pb2,
    fullbody_cartesian_command_pb2,
    joint_pb2,
    kinematics_pb2,
    sensor_pb2,
)

from .type_conversion import pb_matrix_from_ros_pose, ros_pose_from_pb_matrix, ros_pose_from_pb_quaternion, pb_quaternion_from_ros_pose


class BodyControlNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()

        self.joints = {}
        self.sensors = {}
        self.fans = {}

        # Subscribe to:
        #  - /dynamic_joint_states (for present_position, torque and temperature, etc.)
        #  - /joint_commands for target position
        self.joint_state_ready = Event()
        self.joint_state_sub = self.create_subscription(
            msg_type=DynamicJointState,
            topic='/dynamic_joint_states',
            qos_profile=5,
            callback=self._on_joint_state,
        )
        self.joint_commands_ready = Event()
        self.joint_commands_sub = self.create_subscription(
            msg_type=JointState,
            topic='/joint_commands',
            qos_profile=5,
            callback=self._on_joint_command,
        )      

        # Publish to:
        #  - /dynamic_joint_commands
        self.command_pub_lock = Lock()
        self.joint_command_pub = self.create_publisher(
            msg_type=DynamicJointState,
            topic='/dynamic_joint_commands',
            qos_profile=5,
        )

        # Create clients for kinematics services/pub
        self.forward_kin_client = {}
        self.inverse_kin_client = {}
        self.target_pose_publisher = {}

        for chain in ('l_arm', 'r_arm', 'head'):
            forward_srv = self.create_client(
                srv_type=GetForwardKinematics,
                srv_name=f'/{chain}/forward_kinematics',
            )
            if not forward_srv.service_is_ready():
                continue

            self.forward_kin_client[chain] = forward_srv

            inverse_srv = self.create_client(
                srv_type=GetInverseKinematics,
                srv_name=f'/{chain}/inverse_kinematics',
            )

            self.inverse_kin_client[chain] = inverse_srv

            self.target_pose_publisher[chain] = self.create_publisher(
                msg_type=PoseStamped,
                topic=f'/{chain}/averaged_target_pose',
                qos_profile=5,
            )

            self.logger.info(f'Load kinematics (forward/inverse) for "{chain}"')

        self.wait_for_setup()

        self.logger.info(f'Joints found: {list(self.joints.keys())}')
        self.joint_uids = {j['uid']: name for name, j in self.joints.items()}

        self.logger.info(f'Sensors found: {list(self.sensors.keys())}')
        self.sensor_uids = {s['uid']: name for name, s in self.sensors.items()}

        self.logger.info(f'Fans found: {list(self.fans.keys())}')
        self.fan_uids = {f['uid']: name for name, f in self.fans.items()}

    def wait_for_setup(self):
        while not self.joint_state_ready.is_set():
            self.logger.info('Waiting for /dynamic_joint_states...')
            rclpy.spin_once(self)

        while not self.joint_commands_ready.is_set():
            self.logger.info('Waiting for /joint_commands...')
            rclpy.spin_once(self)

    # Get Joint/Sensor/Fan state
    def get_joint_state(self, uid: joint_pb2.JointId, joint_fields: joint_pb2.JointField) -> joint_pb2.JointsState:
        """ Get update info for requested joint.

        And forge a JointsState message with it.
        """
        name = self._get_joint_name(uid)
        values = self.joints[name]

        kwargs = {}

        for field in joint_fields:
            if field in (joint_pb2.JointField.NAME, joint_pb2.JointField.ALL):
                kwargs['name'] = name

            if field in (joint_pb2.JointField.PID, joint_pb2.JointField.ALL):
                if 'p_gain' in values and 'i_gain' in values and 'd_gain' in values:
                    kwargs['pid'] = joint_pb2.PIDValue(pid=joint_pb2.PIDGains(p=values['p_gain'], i=values['i_gain'], d=values['d_gain']))

            if field in (joint_pb2.JointField.UID, joint_pb2.JointField.ALL):
                kwargs['uid'] = UInt32Value(value=values['uid'])

            if field in (joint_pb2.JointField.COMPLIANT, joint_pb2.JointField.ALL):
                if 'torque' in values:
                    kwargs['compliant'] = BoolValue(value=not values['torque'])

            if field in (joint_pb2.JointField.TEMPERATURE, joint_pb2.JointField.ALL):
                if 'temperature' in values:
                    kwargs['temperature'] = FloatValue(value=values['temperature'])

            if field in (joint_pb2.JointField.GOAL_POSITION, joint_pb2.JointField.ALL):
                if 'target_position' in values:
                    kwargs['goal_position'] = FloatValue(value=values['target_position'])

            if field in (joint_pb2.JointField.PRESENT_POSITION, joint_pb2.JointField.ALL):
                if 'position' in values:
                    kwargs['present_position'] = FloatValue(value=values['position'])

            if field in (joint_pb2.JointField.PRESENT_SPEED, joint_pb2.JointField.ALL):
                if 'velocity' in values:
                    kwargs['present_speed'] = FloatValue(value=values['velocity'])

            if field in (joint_pb2.JointField.PRESENT_LOAD, joint_pb2.JointField.ALL):
                if 'effort' in values:
                    kwargs['present_load'] = FloatValue(value=values['effort'])

            if field in (joint_pb2.JointField.TORQUE_LIMIT, joint_pb2.JointField.ALL):
                if 'torque_limit' in values:
                    kwargs['torque_limit'] = FloatValue(value=values['torque_limit'])

            if field in (joint_pb2.JointField.SPEED_LIMIT, joint_pb2.JointField.ALL):
                if 'speed_limit' in values:
                    kwargs['speed_limit'] = FloatValue(value=values['speed_limit'])

        return joint_pb2.JointState(**kwargs)

    def get_sensor_state(self, uid: sensor_pb2.SensorId) -> sensor_pb2.SensorState:
        name = self._get_sensor_name(uid)
        return sensor_pb2.SensorState(
            force_sensor_state=sensor_pb2.ForceSensorState(force=self.sensors[name]['force']),
        )

    def get_fan_state(self, uid: fan_pb2.FanId) -> fan_pb2.FanState:
        name = self._get_fan_name(uid)
        return fan_pb2.FanState(on=bool(self.fans[name]['state']))

    def _on_joint_state(self, state: DynamicJointState):
        """ Retreive the joint state from /dynamic_joint_states.

            Update present_position and temperature.
        """
        if not self.joint_state_ready.is_set():
            self._device_type = {}

            for uid, (name, kv) in enumerate(zip(state.joint_names, state.interface_values)):
                if 'position' in kv.interface_names:
                    self._device_type[name] = 'joints'
                elif 'force' in kv.interface_names:
                    self._device_type[name] = 'sensors'
                elif 'state' in kv.interface_names:
                    self._device_type[name] = 'fans'
                else:
                    self.logger.warning(f'Unkwnon device {name} with interfaces ({kv.interface_names})')

                d = getattr(self, self._device_type[name])

                d[name] = {}
                d[name]['name'] = name
                d[name]['uid'] = uid


        for uid, (name, kv) in enumerate(zip(state.joint_names, state.interface_values)):
            for k, v in zip(kv.interface_names, kv.values):
                d = getattr(self, self._device_type[name])
                d[name][k] = v

        self.joint_state_ready.set()

    def _on_joint_command(self, command: JointState):
        """Retrieve the up-to-date target poisiotn from /joint_commands. """
        
        # Make sure we already have the other info.
        self.joint_state_ready.wait()

        for name, target_pos in zip(command.name, command.position):
            self.joints[name]['target_position'] = target_pos

        self.joint_commands_ready.set()

    # Handle Joint/Fan commands
    def handle_joint_msg(self, grpc_req: joint_pb2.JointsCommand):
        msg = DynamicJointState()

        for cmd in grpc_req.commands:
            if not self._check_joint_exist(cmd.id):
                self.logger.warning(f'Unkown joint id {cmd.id} for JointsCommands')
                continue 
            
            iv = InterfaceValue()

            if cmd.HasField('goal_position'):
                iv.interface_names.append('position')
                iv.values.append(cmd.goal_position.value)

            if cmd.HasField('compliant'):
                iv.interface_names.append('torque')
                iv.values.append(float(not cmd.compliant.value))

            if cmd.HasField('torque_limit'):
                iv.interface_names.append('torque_limit')
                iv.values.append(cmd.torque_limit.value)

            if cmd.HasField('speed_limit'):
                iv.interface_names.append('speed_limit')
                iv.values.append(cmd.speed_limit.value)

            if cmd.HasField('pid'):
                iv.interface_names.extend(['p_gain', 'i_gain', 'd_gain'])
                iv.values.extend([cmd.pid.pid.p, cmd.pid.pid.i, cmd.pid.pid.d])

            if iv.interface_names:
                msg.joint_names.append(self._get_joint_name(cmd.id))
                msg.interface_values.append(iv)

        if msg.joint_names:
            with self.command_pub_lock:
                self.joint_command_pub.publish(msg)
    
    def handle_fan_msg(self, grpc_req: fan_pb2.FansCommand):
        msg = DynamicJointState()

        for cmd in grpc_req.commands:
            if not self._check_fan_exist(cmd.id):
                self.logger.warning(f'Unkown fan id {cmd.id} for FansCommand')
                continue

            msg.joint_names.append(self._get_fan_name(cmd.id))
            iv = InterfaceValue()
            iv.interface_names = ['state']
            iv.values = [float(cmd.on)]
            msg.interface_values.append(iv)          

        if msg.joint_names:
            with self.command_pub_lock:
                self.joint_command_pub.publish(msg)

    # UID/Name sanity checks
    def _check_joint_exist(self, joint_id: joint_pb2.JointId) -> bool:
        if joint_id.HasField('uid'):
            return joint_id.uid in self.joint_uids.keys()
        else:
            return joint_id.name in self.joints.keys()

    def _get_joint_name(self, joint_id: joint_pb2.JointId) -> str:
        if joint_id.HasField('uid'):
            return self.joint_uids[joint_id.uid]
        else:
            return joint_id.name

    def _check_fan_exist(self, fan_id: fan_pb2.FanId) -> bool:
        if fan_id.HasField('uid'):
            return fan_id.uid in self.fan_uids.keys()
        else:
            return fan_id.name in self.fans.keys()

    def _get_sensor_name(self, sensor_id: sensor_pb2.SensorId) -> str:
        if sensor_id.HasField('uid'):
            return self.sensor_uids[sensor_id.uid]
        else:
            return sensor_id.name

    def _check_sensor_exist(self, sensor_id: sensor_pb2.SensorId) -> bool:
        if sensor_id.HasField('uid'):
            return sensor_id.uid in self.sensor_uids.keys()
        else:
            return sensor_id.name in self.sensors.keys()

    def _get_fan_name(self, fan_id: fan_pb2.FanId) -> str:
        if fan_id.HasField('uid'):
            return self.fan_uids[fan_id.uid]
        else:
            return fan_id.name

    # Kinematics related methods
    def arm_forward_kinematics(self, request: ArmFKRequest) -> ArmFKSolution:
        arm = 'l_arm' if request.arm_position.side == ArmSide.LEFT else 'r_arm'

        resp = self._forward_kin(arm, request.arm_position.positions)
        
        if resp is None:
            return ArmFKSolution(success=False)

        sol = ArmFKSolution(
            success=resp.success,
            end_effector=ArmEndEffector(
                side=request.arm_position.side,
                pose=pb_matrix_from_ros_pose(resp.pose),
            ),
        )

        return sol

    def head_forward_kinematics(self, request: HeadFKRequest) -> HeadFKSolution:
        resp = self._forward_kin('head', request.neck_position)

        if resp is None:
            return HeadFKSolution(success=False)

        sol = HeadFKSolution(
            success=resp.success,
            q=pb_quaternion_from_ros_pose(resp.pose),

        )

        return sol

    def arm_inverse_kinematics(self, request: ArmIKRequest) -> ArmIKSolution:
        arm = 'l_arm' if request.target.side == ArmSide.LEFT else 'r_arm'

        resp = self._inverse_kin(arm, ros_pose_from_pb_matrix(request.target.pose), request.q0)

        if resp is None:
            return ArmFKSolution(success=False)

        sol = ArmIKSolution(
            success=resp.success,
            arm_position=ArmJointPosition(
                side=request.target.side,
                positions=kinematics_pb2.JointPosition(
                    ids=[joint_pb2.JointId(uid=self.joints[name]['uid']) for name in resp.joint_position.name],
                    positions=resp.joint_position.position,
                ),
            ),
        )

        return sol

    def head_inverse_kinematics(self, request: HeadIKRequest) -> HeadIKSolution:
        resp = self._inverse_kin('head', ros_pose_from_pb_quaternion(request.q), request.q0)

        if resp is None:
            return HeadFKSolution(success=False)

        sol = HeadIKSolution(
            success=resp.success,
            neck_position=kinematics_pb2.JointPosition(
                ids=[joint_pb2.JointId(uid=self.joints[name]['uid']) for name in resp.joint_position.name],
                positions=resp.joint_position.position,
            ),
        )

        return sol

    def _forward_kin(self, name: str, joint_positions: kinematics_pb2.JointPosition) -> Optional[GetForwardKinematics.Response]:
        if name not in self.forward_kin_client:
            self.logger.warning(f'Could not find FK service for "{name}"!')
            return

        fk_cli = self.forward_kin_client[name]

        ros_req = GetForwardKinematics.Request()
        ros_req.joint_position.name = [self._get_joint_name(id) for id in joint_positions.ids]
        ros_req.joint_position.position = joint_positions.positions

        resp = fk_cli.call(ros_req)

        return resp

    def _inverse_kin(self, name: str, ros_pose: Pose, q0: kinematics_pb2.JointPosition) -> Optional[GetInverseKinematics.Response]:
        if name not in self.inverse_kin_client:
            self.logger.warning(f'Could not find IK service for "{name}"!')
            return

        ik_cli = self.inverse_kin_client[name]

        ros_req = GetInverseKinematics.Request()
        ros_req.pose = ros_pose
        ros_req.q0.name = [self._get_joint_name(id) for id in q0.ids]
        ros_req.q0.position = q0.positions

        resp = ik_cli.call(ros_req)

        return resp

    # FullBodyCartesian related methods
    def handle_fullbody_cartesian_command(self, cmd: fullbody_cartesian_command_pb2.FullBodyCartesianCommand):
        ack = fullbody_cartesian_command_pb2.FullBodyCartesianCommandAck()

        if cmd.HasField('left_arm'):
            ack.left_arm_command_success = self.handle_arm_cartesian_request(cmd.left_arm, 'l_arm')
        if cmd.HasField('right_arm'):
            ack.right_arm_command_success = self.handle_arm_cartesian_request(cmd.right_arm, 'r_arm')
        if cmd.HasField('head'):
            ack.head_command_success = self.handle_head_cartesian_request(cmd.head, 'head')

        return ack

    def handle_arm_cartesian_request(self, request: ArmIKRequest, name) -> bool:
        if name not in self.target_pose_publisher:
            return False

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = ros_pose_from_pb_matrix(request.target.pose)

        self.target_pose_publisher[name].publish(pose)

        return True

    def handle_head_cartesian_request(self, request: HeadIKRequest, name) -> bool:
        if name not in self.target_pose_publisher:
            return False

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose = ros_pose_from_pb_quaternion(request.q)

        self.target_pose_publisher[name].publish(pose)

        return True
