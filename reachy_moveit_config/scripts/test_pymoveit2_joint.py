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
    node = Node("ex_joint_goal")

    node.declare_parameter(
        "joint_positions",
        [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        ],
    )

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
        follow_joint_trajectory_action_name='/left_arm_controller/follow_joint_trajectory'
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameter
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )

    # Move to joint configuration
    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
