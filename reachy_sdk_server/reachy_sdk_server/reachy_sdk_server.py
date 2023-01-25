"""Expose main Reachy ROS services/topics through gRPC allowing remote client SDK."""

from pathlib import Path
import threading
import time
from subprocess import check_output

from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, Iterator, List

import numpy as np

from scipy.spatial.transform import Rotation

from google.protobuf.empty_pb2 import Empty
from google.protobuf.timestamp_pb2 import Timestamp
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue

import grpc

import rclpy
from rclpy.node import Node

from reachy_sdk_api import joint_pb2, joint_pb2_grpc
from reachy_sdk_api import sensor_pb2, sensor_pb2_grpc
from reachy_sdk_api import kinematics_pb2
from reachy_sdk_api import arm_kinematics_pb2, arm_kinematics_pb2_grpc
from reachy_sdk_api import fullbody_cartesian_command_pb2, fullbody_cartesian_command_pb2_grpc
from reachy_sdk_api import fan_pb2, fan_pb2_grpc
from reachy_sdk_api import mobile_platform_reachy_pb2, mobile_platform_reachy_pb2_grpc

from reachy_sdk_server.body_control_ros_node import BodyControlNode


class ReachySDKServer(
                      joint_pb2_grpc.JointServiceServicer,
                      sensor_pb2_grpc.SensorServiceServicer,
                      fan_pb2_grpc.FanControllerServiceServicer,
                      ):
    """Reachy SDK server node."""

    def __init__(self, node_name: str, timeout_sec: float = 5, pub_frequency: float = 100) -> None:
        """Set up the node.

        Subscribe to /joint_states, /joint_temperatures, /force_sensors.
        Publish new command on /joint_goals or concerned services.

        """
        self.timeout_sec = timeout_sec
        self.pub_period = 1 / pub_frequency

        rclpy.init()
        self.body_control_node = BodyControlNode(
            controllers_file='reachy_no_orbita_controllers'
        )
        threading.Thread(target=lambda: rclpy.spin(self.body_control_node)).start()

        self.logger = self.body_control_node.get_logger()
        self.clock = self.body_control_node.get_clock()

        self.logger.info('SDK ready to be served!')

    def GetJointsState(self, request: joint_pb2.JointsStateRequest, context) -> joint_pb2.JointsState:
        """Get the requested joints id."""
        params = {}

        params['ids'] = request.ids
        params['states'] = [
            self.body_control_node.get_joint_state(uid=id, joint_fields=request.requested_fields)
            for id in request.ids
        ]
        params['timestamp'] = Timestamp()
        params['timestamp'].GetCurrentTime()

        return joint_pb2.JointsState(**params)

    def StreamJointsState(self, request: joint_pb2.StreamJointsRequest, context) -> Iterator[joint_pb2.JointsState]:
        """Continuously stream requested joints up-to-date state."""
        dt = 1.0 / request.publish_frequency if request.publish_frequency > 0 else -1.0
        last_pub = 0.0

        while True:
            elapsed_time = time.time() - last_pub
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

            self.body_control_node.joint_state_pub_event.wait()
            self.body_control_node.joint_state_pub_event.clear()

            joints_state = self.GetJointsState(request.request, context)
            joints_state.timestamp.GetCurrentTime()

            yield joints_state
            last_pub = time.time()

    def SendJointsCommands(self, request: joint_pb2.JointsCommand, context) -> joint_pb2.JointsCommandAck:
        self.body_control_node.handle_joint_msg(grpc_req=request)
        return joint_pb2.JointsCommandAck(success=True)

    def SendFansCommands(self, request: fan_pb2.FansCommand, context) -> fan_pb2.FansCommandAck:
        self.body_control_node.handle_fan_msg(grpc_req=request)
        return fan_pb2.FansCommandAck(success=True)

    def StreamJointsCommands(self, request_iterator: Iterator[joint_pb2.JointsCommand], context) -> joint_pb2.JointsCommandAck:
        for request in request_iterator:
            self.body_control_node.handle_joint_msg(grpc_req=request)
        return joint_pb2.JointsCommandAck(success=True)

    def GetAllJointsId(self, request: Empty, context) -> joint_pb2.JointsId:
        names, uids = zip(*[
            (joint['name'], joint['uid']) 
            for joint in self.body_control_node.joints.values()
        ])
        return joint_pb2.JointsId(names=names, uids=uids)

    def GetAllForceSensorsId(self, request: Empty, context) -> sensor_pb2.SensorsId:
        names, uids = zip(*[
            (sensor['name'], sensor['uid']) 
            for sensor in self.body_control_node.sensors.values()
        ])
        return sensor_pb2.SensorsId(names=names, uids=uids)

    def GetAllFansId(self, request: Empty, context) -> fan_pb2.FansId:
        names, uids = zip(*[
            (fan['name'], fan['uid']) 
            for fan in self.body_control_node.fans.values()
        ])
        return fan_pb2.FansId(names=names, uids=uids)

    def GetSensorsState(self, request: sensor_pb2.SensorsStateRequest, context) -> sensor_pb2.SensorsState:
        params = {}

        params['ids'] = request.ids
        params['states'] = [
            self.body_control_node.get_sensor_state(uid=id) for id in request.ids
        ]
        params['timestamp'] = Timestamp()
        params['timestamp'].GetCurrentTime()
        return sensor_pb2.SensorsState(**params)

    def GetFansState(self, request: fan_pb2.FansStateRequest, context) -> fan_pb2.FansState:
        params = {}

        params['ids'] = request.ids
        params['states'] = [
            self.body_control_node.get_fan_state(uid=id) for id in request.ids
        ]
        params['timestamp'] = Timestamp()
        params['timestamp'].GetCurrentTime()
        return fan_pb2.FansState(**params)

    def StreamSensorStates(self, request: Iterator[sensor_pb2.SensorsStateRequest], context) -> Iterator[sensor_pb2.SensorsState]:
        dt = 1.0 / request.publish_frequency if request.publish_frequency > 0 else -1.0
        last_pub = 0.0

        while True:
            elapsed_time = time.time() - last_pub
            if elapsed_time < dt:
                time.sleep(dt - elapsed_time)

            self.body_control_node.sensor_state_pub_event.wait()
            self.body_control_node.sensor_state_pub_event.clear()

            sensors_state = self.GetSensorsState(request.request, context)
            sensors_state.timestamp.GetCurrentTime()

            yield sensors_state
            last_pub = time.time()


def main():
    """Run the Node and the gRPC server."""
    sdk_server = ReachySDKServer(node_name='reachy_sdk_server')

    server = grpc.server(thread_pool=ThreadPoolExecutor(max_workers=10))
    joint_pb2_grpc.add_JointServiceServicer_to_server(sdk_server, server)
    sensor_pb2_grpc.add_SensorServiceServicer_to_server(sdk_server, server)
    fan_pb2_grpc.add_FanControllerServiceServicer_to_server(sdk_server, server)

    server.add_insecure_port('[::]:50055')
    server.start()

    server.wait_for_termination()


if __name__ == '__main__':
    main()
