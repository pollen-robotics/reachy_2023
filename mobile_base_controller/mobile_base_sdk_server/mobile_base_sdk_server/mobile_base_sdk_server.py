"""Expose main mobile base ROS services/topics through gRPC allowing remote client SDK."""

import time
from subprocess import run, PIPE, check_output
from concurrent.futures import ThreadPoolExecutor
from queue import Empty

import grpc
from google.protobuf.wrappers_pb2 import BoolValue, FloatValue

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from reachy_sdk_api import mobile_platform_reachy_pb2, mobile_platform_reachy_pb2_grpc

from zuuu_interfaces.srv import SetZuuuMode, GetZuuuMode, GetOdometry, ResetOdometry
from zuuu_interfaces.srv import GoToXYTheta, DistanceToGoal, SetZuuuSafety
from zuuu_interfaces.srv import SetSpeed, GetBatteryVoltage


class MobileBaseServer(
                        Node,
                        mobile_platform_reachy_pb2_grpc.MobileBasePresenceServiceServicer,
                        mobile_platform_reachy_pb2_grpc.MobilityServiceServicer,
                        ):
    """Mobile base SDK server node."""

    def __init__(self, node_name: str) -> None:
        """Set up the node.

        Get mobile base basic info such as its odometry, battery level, drive mode or control mode
        from the mobile base hal.
        Send commands through the GoToXYTheta or SetSpeed services or by publishing to cmd_vel topic.
        """
        super().__init__(node_name=node_name)
        self.logger = self.get_logger()

        self.clock = self.get_clock()

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.set_speed_client = self.create_client(SetSpeed, 'SetSpeed')
        while not self.set_speed_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service SetSpeed not available, waiting again...')

        self.go_to_client = self.create_client(GoToXYTheta, 'GoToXYTheta')
        while not self.go_to_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service GoToXYTheta not available, waiting again...')

        self.distance_to_goal_client = self.create_client(DistanceToGoal, 'DistanceToGoal')
        while not self.distance_to_goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service DistanceToGoal not available, waiting again...')

        self.set_zuuu_mode_client = self.create_client(SetZuuuMode, 'SetZuuuMode')
        while not self.set_zuuu_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service SetZuuuMode not available, waiting again...')

        self.get_zuuu_mode_client = self.create_client(GetZuuuMode, 'GetZuuuMode')
        while not self.get_zuuu_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service GetZuuuMode not available, waiting again...')

        self.get_battery_voltage_client = self.create_client(GetBatteryVoltage, 'GetBatteryVoltage')
        while not self.get_battery_voltage_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service GetBatteryVoltage not available, waiting again...')

        self.get_odometry_client = self.create_client(GetOdometry, 'GetOdometry')
        while not self.get_odometry_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service GetOdometry not available, waiting again...')

        self.reset_odometry_client = self.create_client(ResetOdometry, 'ResetOdometry')
        while not self.reset_odometry_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service ResetOdometry not available, waiting again...')

        self.set_zuuu_safety = self.create_client(SetZuuuSafety, 'SetZuuuSafety')
        while not self.reset_odometry_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service ResetOdometry not available, waiting again...')

        self.logger.info('Initialized mobile base server.')

    def SendDirection(
                    self,
                    request: mobile_platform_reachy_pb2.TargetDirectionCommand,
                    context) -> mobile_platform_reachy_pb2.MobilityServiceAck:
        """Send a speed command for the mobile base expressed in SI units."""
        twist = Twist()
        twist.linear.x = request.direction.x.value
        twist.linear.y = request.direction.y.value
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = request.direction.theta.value
        self.cmd_vel_pub.publish(twist)

        return mobile_platform_reachy_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def SendSetSpeed(
                    self,
                    request: mobile_platform_reachy_pb2.SetSpeedVector,
                    context) -> mobile_platform_reachy_pb2.MobilityServiceAck:
        """Send a speed command for the mobile base expressed in SI units for a given duration."""
        req = SetSpeed.Request()
        req.duration = request.duration.value
        req.x_vel = request.x_vel.value
        req.y_vel = request.y_vel.value
        req.rot_vel = request.rot_vel.value

        self.set_speed_client.call_async(req)
        return mobile_platform_reachy_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def SendGoTo(
                self,
                request: mobile_platform_reachy_pb2.GoToVector,
                context) -> mobile_platform_reachy_pb2.MobilityServiceAck:
        """Send a target to the mobile base in the odom frame.

        The origin of the frame is initialised when the hal is started or whenever the odometry
        is reset. x and y are in meters and theta in radian.
        """
        req = GoToXYTheta.Request()
        req.x_goal = request.x_goal.value
        req.y_goal = request.y_goal.value
        req.theta_goal = request.theta_goal.value

        self.go_to_client.call_async(req)
        return mobile_platform_reachy_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def DistanceToGoal(self, request, context):
        """Return the distance left to reach the last goto target sent.

        The remaining x, y and theta to get to the target are also returned.
        """
        response = mobile_platform_reachy_pb2.DistanceToGoalVector(
            delta_x=FloatValue(value=0.0),
            delta_y=FloatValue(value=0.0),
            delta_theta=FloatValue(value=0.0),
            distance=FloatValue(value=0.0))

        req = DistanceToGoal.Request()

        future = self.distance_to_goal_client.call_async(req)
        for _ in range(1000):
            if future.done():
                ros_response = future.result()
                print(ros_response)
                response.delta_x.value = ros_response.delta_x
                response.delta_y.value = ros_response.delta_y
                response.delta_theta.value = ros_response.delta_theta
                response.distance.value = ros_response.distance
                break
            time.sleep(0.001)
        return response

    def SetControlMode(
                    self,
                    request: mobile_platform_reachy_pb2.ControlModeCommand,
                    context) -> mobile_platform_reachy_pb2.MobilityServiceAck:
        """Set mobile base control mode.

        Two valid control modes are available: OPEN_LOOP and PID.
        """
        mode = mobile_platform_reachy_pb2.ControlModePossiblities.keys()[request.mode]

        if mode == 'NONE_CONTROL_MODE':
            return mobile_platform_reachy_pb2.MobilityServiceAck(success=BoolValue(value=False))

        run(f'ros2 param set /zuuu_hal control_mode {mode}', stdout=PIPE, shell=True)
        return mobile_platform_reachy_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def GetControlMode(
                    self,
                    request: Empty,
                    context) -> mobile_platform_reachy_pb2.ControlModeCommand:
        """Get mobile base control mode."""
        output = check_output(['ros2', 'param', 'get', '/zuuu_hal', 'control_mode']).decode()

        # Response from ros2 looks like: "String value is: MODE"
        mode = output.split(': ')[-1].split()[0]

        mode_grpc = getattr(mobile_platform_reachy_pb2.ControlModePossiblities, mode)
        return mobile_platform_reachy_pb2.ControlModeCommand(mode=mode_grpc)

    def SetZuuuMode(
                self,
                request: mobile_platform_reachy_pb2.ZuuuModeCommand,
                context) -> mobile_platform_reachy_pb2.MobilityServiceAck:
        """Set mobile base drive mode.

        Six valid drive modes are available: CMD_VEL, BRAKE, FREE_WHEEL, SPEED, GOTO, EMERGENCY_STOP.
        """
        mode = mobile_platform_reachy_pb2.ZuuuModePossiblities.keys()[request.mode]

        if mode == 'NONE_ZUUU_MODE':
            return mobile_platform_reachy_pb2.MobilityServiceAck(success=BoolValue(value=False))

        req = SetZuuuMode.Request()
        req.mode = mode
        self.set_zuuu_mode_client.call_async(req)
        return mobile_platform_reachy_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def GetZuuuMode(
                    self,
                    request: Empty,
                    context) -> mobile_platform_reachy_pb2.ZuuuModeCommand:
        """Get mobile base drive mode."""
        req = GetZuuuMode.Request()

        future = self.get_zuuu_mode_client.call_async(req)
        for _ in range(1000):
            if future.done():
                mode = future.result().mode
                break
            time.sleep(0.001)
        if not future.done():
            mode = mobile_platform_reachy_pb2.ZuuuModePossiblities.NONE_ZUUU_MODE
            return mobile_platform_reachy_pb2.ZuuuModeCommand(mode=mode)

        mode_grpc = getattr(mobile_platform_reachy_pb2.ZuuuModePossiblities, mode)
        return mobile_platform_reachy_pb2.ZuuuModeCommand(mode=mode_grpc)

    def GetBatteryLevel(
                self,
                request: Empty,
                context) -> mobile_platform_reachy_pb2.BatteryLevel:
        """Get mobile base battery level in Volt."""
        req = GetBatteryVoltage.Request()

        response = mobile_platform_reachy_pb2.BatteryLevel(
            level=FloatValue(value=0.0)
            )

        future = self.get_battery_voltage_client.call_async(req)
        for _ in range(1000):
            if future.done():
                ros_response = future.result()
                response.level.value = ros_response.voltage
                break
            time.sleep(0.001)
        return response

    def GetOdometry(
                self,
                request: Empty,
                context) -> mobile_platform_reachy_pb2.OdometryVector:
        """Get mobile base odometry.

        x, y are in meters and theta is in radian.
        """
        req = GetOdometry.Request()
        response = mobile_platform_reachy_pb2.OdometryVector(
            x=FloatValue(value=0.0),
            y=FloatValue(value=0.0),
            theta=FloatValue(value=0.0),
        )

        future = self.get_odometry_client.call_async(req)
        for _ in range(1000):
            if future.done():
                ros_response = future.result()
                response.x.value = ros_response.x
                response.y.value = ros_response.y
                response.theta.value = ros_response.theta
                break
            time.sleep(0.001)
        return response

    def ResetOdometry(
                self,
                request: Empty,
                context) -> mobile_platform_reachy_pb2.MobilityServiceAck:
        """Reset mobile base odometry.

        Current position of the mobile_base is taken as new origin of the odom frame.
        """
        req = ResetOdometry.Request()
        self.reset_odometry_client.call_async(req)
        return mobile_platform_reachy_pb2.MobilityServiceAck(success=BoolValue(value=True))

    def GetMobileBasePresence(
                            self,
                            request: Empty,
                            context) -> mobile_platform_reachy_pb2.MobileBasePresence:
        """Return if a mobile base is in Reachy's config file.

        If yes, return the mobile base version.
        """
        presence = False
        version = '0.0'

        model = check_output(['reachy-identify-zuuu-model']).strip().decode()

        if model and model != 'None':
            presence = True
            version = float(model)

        response = mobile_platform_reachy_pb2.MobileBasePresence(
            presence=BoolValue(value=presence),
            model_version=FloatValue(value=version),
        )
        return response

    def SetZuuuSafety(
                    self,
                    request: mobile_platform_reachy_pb2.SetZuuuSafetyRequest,
                    context) -> mobile_platform_reachy_pb2.MobilityServiceAck:
        """Set on/off the anti-collision safety handled by the mobile base hal."""
        req = SetZuuuSafety.Request()
        req.safety_on = request.safety_on.value
        self.set_zuuu_safety.call_async(req)
        return mobile_platform_reachy_pb2.MobilityServiceAck(success=BoolValue(value=True))


def main():
    """Run the Node and the gRPC server."""
    rclpy.init()

    mobile_base_server = MobileBaseServer(node_name='mobile_base_server')

    options = [
         ('grpc.max_send_message_length', 250000),
         ('grpc.max_receive_message_length', 250000),
         ]

    server = grpc.server(thread_pool=ThreadPoolExecutor(max_workers=10), options=options)
    mobile_platform_reachy_pb2_grpc.add_MobilityServiceServicer_to_server(mobile_base_server, server)
    mobile_platform_reachy_pb2_grpc.add_MobileBasePresenceServiceServicer_to_server(mobile_base_server, server)

    server.add_insecure_port('[::]:50061')
    server.start()

    try:
        rclpy.spin(mobile_base_server)
    except KeyboardInterrupt:
        pass

    server.stop(grace=None)
    server.wait_for_termination()

    mobile_base_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
