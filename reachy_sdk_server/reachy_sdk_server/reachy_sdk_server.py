"""Expose main Reachy ROS services/topics through gRPC allowing remote client SDK."""

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

from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

from geometry_msgs.msg import Point, Quaternion

from sensor_msgs.msg import JointState

from reachy_msgs.msg import JointTemperature, ForceSensor, PidGains, FanState
from reachy_msgs.srv import GetJointFullState, SetJointCompliancy, SetJointPidGains
from reachy_msgs.srv import GetArmIK, GetArmFK
from reachy_msgs.srv import GetReachyModel, SetFanState

from reachy_sdk_api import joint_pb2, joint_pb2_grpc
from reachy_sdk_api import sensor_pb2, sensor_pb2_grpc
from reachy_sdk_api import kinematics_pb2
from reachy_sdk_api import arm_kinematics_pb2, arm_kinematics_pb2_grpc
from reachy_sdk_api import fullbody_cartesian_command_pb2, fullbody_cartesian_command_pb2_grpc
from reachy_sdk_api import fan_pb2, fan_pb2_grpc
from reachy_sdk_api import mobile_platform_reachy_pb2, mobile_platform_reachy_pb2_grpc

from body_control_ros_node import BodyControlNode


class ReachySDKServer(
                      joint_pb2_grpc.JointServiceServicer,
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
            controllers_file='/home/pierre/reachy_ws/src/reachy_2023/reachy_bringup/config/reachy_controllers.yaml'
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
            self.body_control_node.get_joint_state(uid=id)
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
        self.body_control_node._update_joint_target_pos(grpc_req=request)
        return joint_pb2.JointsCommandAck(success=True)

    def StreamJointsCommands(self, request_iterator: Iterator[joint_pb2.JointsCommand], context) -> joint_pb2.JointsCommandAck:
        for request in request_iterator:
            self.body_control_node._update_joint_target_pos(grpc_req=request)
        return joint_pb2.JointsCommandAck(success=True)


def main():
    """Run the Node and the gRPC server."""
    sdk_server = ReachySDKServer(node_name='reachy_sdk_server')

    server = grpc.server(thread_pool=ThreadPoolExecutor(max_workers=10))
    joint_pb2_grpc.add_JointServiceServicer_to_server(sdk_server, server)

    server.add_insecure_port('[::]:50055')
    server.start()

    server.wait_for_termination()


if __name__ == '__main__':
    main()
