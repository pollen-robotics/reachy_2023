"""Expose main Reachy ROS services/topics through gRPC allowing remote client SDK."""

from concurrent.futures import ThreadPoolExecutor
import threading
import time
from typing import Iterator

from google.protobuf.empty_pb2 import Empty
from google.protobuf.timestamp_pb2 import Timestamp
import grpc

import rclpy

from reachy_sdk_api import joint_pb2, joint_pb2_grpc
from reachy_sdk_api import sensor_pb2, sensor_pb2_grpc
from reachy_sdk_api import arm_kinematics_pb2, arm_kinematics_pb2_grpc
from reachy_sdk_api import head_kinematics_pb2, head_kinematics_pb2_grpc
from reachy_sdk_api import fullbody_cartesian_command_pb2, fullbody_cartesian_command_pb2_grpc
from reachy_sdk_api import fan_pb2, fan_pb2_grpc
from reachy_sdk_api import config_pb2, config_pb2_grpc

from reachy_sdk_server.body_control_ros_node import BodyControlNode


class ReachySDKServer(
    arm_kinematics_pb2_grpc.ArmKinematicsServicer,
    head_kinematics_pb2_grpc.HeadKinematicsServicer,
    fullbody_cartesian_command_pb2_grpc.FullBodyCartesianCommandServiceServicer,
    joint_pb2_grpc.JointServiceServicer,
    sensor_pb2_grpc.SensorServiceServicer,
    fan_pb2_grpc.FanControllerServiceServicer,
    config_pb2_grpc.ConfigServiceServicer,
):
    """Reachy SDK server node.

    Converts gRPC requests to commands sent to the dynamic_state_router node.
    """

    def __init__(self, node_name: str) -> None:
        """Set up the node."""
        rclpy.init()
        self.body_control_node = BodyControlNode(
            node_name=node_name,
        )
        threading.Thread(target=lambda: rclpy.spin(self.body_control_node)).start()

        self.logger = self.body_control_node.get_logger()
        self.clock = self.body_control_node.get_clock()

        self.logger.info('SDK ready to be served! (port 50055)')

    # Joints gRPCs
    def GetAllJointsId(self, request: Empty, context) -> joint_pb2.JointsId:
        names, uids = zip(*[
            (joint['name'], joint['uid'])
            for joint in self.body_control_node.joints.values()
        ])
        return joint_pb2.JointsId(names=names, uids=uids)

    def GetJointsState(self, request: joint_pb2.JointsStateRequest, context) -> joint_pb2.JointsState:
        """Get the requested joints id."""
        params = {}
        params['ids'] = []
        params['states'] = []

        for id in request.ids:
            if not self.body_control_node._check_joint_exist(id):
                self.logger.warning(f'Got invalid joint request for GetJointsState, received {id}')
                continue

            params['ids'].append(id)
            params['states'].append(self.body_control_node.get_joint_state(uid=id, joint_fields=request.requested_fields))

        params['timestamp'] = Timestamp()
        params['timestamp'].GetCurrentTime()

        return joint_pb2.JointsState(**params)

    def StreamJointsState(self, request: joint_pb2.StreamJointsRequest, context) -> Iterator[joint_pb2.JointsState]:
        """Continuously stream requested joints up-to-date state."""
        dt = 1.0 / request.publish_frequency if request.publish_frequency > 0 else -1.0
        last_pub = 0.0

        for id in request.request.ids:
            if not self.body_control_node._check_joint_exist(id):
                self.logger.error(f'Got invalid joint request for StreamJointsState, received {id}')
                return joint_pb2.JointsState()

        while True:
            elapsed_time = time.time() - last_pub
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

            joints_state = self.GetJointsState(request.request, context)
            joints_state.timestamp.GetCurrentTime()

            yield joints_state
            last_pub = time.time()

    def SendJointsCommands(self, request: joint_pb2.JointsCommand, context) -> joint_pb2.JointsCommandAck:
        self.body_control_node.handle_joint_msg(grpc_req=request)
        return joint_pb2.JointsCommandAck(success=True)

    def StreamJointsCommands(self, request_iterator: Iterator[joint_pb2.JointsCommand], context) -> joint_pb2.JointsCommandAck:
        for request in request_iterator:
            self.body_control_node.handle_joint_msg(grpc_req=request)

        return joint_pb2.JointsCommandAck(success=True)

    # Fans gRPCs
    def GetAllFansId(self, request: Empty, context) -> fan_pb2.FansId:
        names, uids = zip(*[
            (fan['name'], fan['uid'])
            for fan in self.body_control_node.fans.values()
        ])
        return fan_pb2.FansId(names=names, uids=uids)

    def GetFansState(self, request: fan_pb2.FansStateRequest, context) -> fan_pb2.FansState:
        params = {}
        params['ids'] = []
        params['states'] = []

        for id in request.ids:
            if not self.body_control_node._check_fan_exist(id):
                self.logger.warning(f'Got invalid fan request for GetFansState, received {id}')
                continue
            
            params['ids'].append(id)
            params['states'].append(self.body_control_node.get_fan_state(uid=id))
        
        params['timestamp'] = Timestamp()
        params['timestamp'].GetCurrentTime()

        return fan_pb2.FansState(**params)

    def SendFansCommands(self, request: fan_pb2.FansCommand, context) -> fan_pb2.FansCommandAck:
        self.body_control_node.handle_fan_msg(grpc_req=request)
        return fan_pb2.FansCommandAck(success=True)
    
    # Force Sensors gRPCs
    def GetAllForceSensorsId(self, request: Empty, context) -> sensor_pb2.SensorsId:
        if not self.body_control_node.sensors:
            return sensor_pb2.SensorsId(names=[], uids=[])

        names, uids = zip(*[
            (sensor['name'], sensor['uid'])
            for sensor in self.body_control_node.sensors.values()
        ])
        return sensor_pb2.SensorsId(names=names, uids=uids)

    def GetSensorsState(self, request: sensor_pb2.SensorsStateRequest, context) -> sensor_pb2.SensorsState:
        params = {}
        params['ids'] = []
        params['states'] = []

        for id in request.ids:
            if not self.body_control_node._check_sensor_exist(id):
                self.logger.warning(f'Got invalid sensor request for GetSensorsState, received {id}')
                continue
            
            params['ids'].append(id)
            params['states'].append(self.body_control_node.get_sensor_state(uid=id))
        
        params['timestamp'] = Timestamp()
        params['timestamp'].GetCurrentTime()

        return sensor_pb2.SensorsState(**params)

    def StreamSensorStates(self, request: Iterator[sensor_pb2.SensorsStateRequest], context) -> Iterator[sensor_pb2.SensorsState]:
        dt = 1.0 / request.publish_frequency if request.publish_frequency > 0 else -1.0
        last_pub = 0.0

        while True:
            elapsed_time = time.time() - last_pub
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

            sensors_state = self.GetSensorsState(request.request, context)
            sensors_state.timestamp.GetCurrentTime()

            yield sensors_state
            last_pub = time.time()

    # Arm kinematics gRPCs
    def ComputeArmFK(self, request: arm_kinematics_pb2.ArmFKRequest, context) -> arm_kinematics_pb2.ArmFKSolution:
        """Compute forward kinematics for requested arm."""
        return self.body_control_node.arm_forward_kinematics(request)

    def ComputeArmIK(self, request: arm_kinematics_pb2.ArmIKRequest, context) -> arm_kinematics_pb2.ArmIKSolution:
        """Compute inverse kinematics for requested arm."""
        return self.body_control_node.arm_inverse_kinematics(request)

    # Head kinematics gRPCs
    def ComputeHeadFK(self, request: head_kinematics_pb2.HeadFKRequest, context) -> head_kinematics_pb2.HeadFKSolution:
        """Compute forward kinematics for the head."""
        return self.body_control_node.head_forward_kinematics(request)

    def ComputeHeadIK(self, request: head_kinematics_pb2.HeadIKRequest, context) -> head_kinematics_pb2.HeadIKSolution:
        """Compute inverse kinematics for the head."""
        return self.body_control_node.head_inverse_kinematics(request)

    # Full Body Cartesian gRPCs
    def SendFullBodyCartesianCommands(
        self,
        request: fullbody_cartesian_command_pb2.FullBodyCartesianCommand,
        context,
    ) -> fullbody_cartesian_command_pb2.FullBodyCartesianCommandAck:
        """Compute movement given the requested commands in cartesian space."""
        return self.body_control_node.handle_fullbody_cartesian_command(request)

    def StreamFullBodyCartesianCommands(
        self,
        request_iterator: Iterator[fullbody_cartesian_command_pb2.FullBodyCartesianCommand],
        context,
    ) -> fullbody_cartesian_command_pb2.FullBodyCartesianCommandAck:
        """Compute movement from stream of commands in cartesian space."""
        for request in request_iterator:
            _ = self.SendCartesianCommand(request, context)

        return fullbody_cartesian_command_pb2.FullBodyCartesianCommandAck(
            left_arm_command_success=True,
            right_arm_command_success=True,
            head_command_success=True,
        )

    # Config gRPCs
    def GetReachyConfig(self, request: Empty, context) -> config_pb2.ConfigReachy:
        """Get Reachy generation and if there is a mobile base attached."""
        from reachy_utils.config import get_reachy_generation, get_zuuu_version, get_camera_parameters, get_reachy_model
        generation = get_reachy_generation()
        mobile_base_presence = True if get_zuuu_version() != 'none' else False
        config = get_reachy_model()

        camera_parameters = get_camera_parameters()

        left = list(camera_parameters["left"].values())
        right = list(camera_parameters["right"].values())
        camera_parameters = left + right

        return config_pb2.ConfigReachy(
            generation=generation,
            mobile_base_presence=mobile_base_presence,
            camera_parameters=camera_parameters,
            config=config,
        )


def main():
    """Run the Node and the gRPC server."""

    sdk_server = ReachySDKServer(
        node_name='reachy_sdk_server',
    )

    server = grpc.server(thread_pool=ThreadPoolExecutor(max_workers=10))

    arm_kinematics_pb2_grpc.add_ArmKinematicsServicer_to_server(sdk_server, server)
    head_kinematics_pb2_grpc.add_HeadKinematicsServicer_to_server(sdk_server, server)
    fullbody_cartesian_command_pb2_grpc.add_FullBodyCartesianCommandServiceServicer_to_server(sdk_server, server)
    joint_pb2_grpc.add_JointServiceServicer_to_server(sdk_server, server)
    sensor_pb2_grpc.add_SensorServiceServicer_to_server(sdk_server, server)
    fan_pb2_grpc.add_FanControllerServiceServicer_to_server(sdk_server, server)
    config_pb2_grpc.add_ConfigServiceServicer_to_server(sdk_server, server)

    server.add_insecure_port('[::]:50055')
    server.start()

    server.wait_for_termination()


if __name__ == '__main__':
    main()
