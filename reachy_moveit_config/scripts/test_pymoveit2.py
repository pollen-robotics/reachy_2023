from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
# from pymoveit2.robots import panda
import reachy_pymoveit2 as reachy_pymoveit2


def main():

    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.36, 0.17, 0.71])  # -0.3
    node.declare_parameter("quat_xyzw", [0.0, -0.706, 0.0, 0.707])
    node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=reachy_pymoveit2.joint_names(),
        base_link_name=reachy_pymoveit2.base_link_name(),
        end_effector_name=reachy_pymoveit2.end_effector_name(),
        group_name=reachy_pymoveit2.MOVE_GROUP_ARM,
        callback_group=callback_group,
        follow_joint_trajectory_action_name='/left_arm_controller/follow_joint_trajectory',
        execute_via_moveit=True
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value
    plan = moveit2.plan(position, quat_xyzw)
    node.get_logger().info(f'PLAN: {plan}')

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )

    moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
