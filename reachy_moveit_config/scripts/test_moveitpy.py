#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""
import numpy as np
import rclpy
from rclpy.node import Node
import sys
import threading
import time

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Constraints

from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
    MultiPipelinePlanRequestParameters,
)
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = rclpy.logging.get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    moveit = MoveItPy(node_name="moveit_py")
    reachy = moveit.get_planning_component("reachy_2023")
    logger.info("MoveItPy instance created")

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    # set plan start state using predefined state
    reachy.set_start_state(configuration_name="zero")

    # set pose goal using predefined state
    reachy.set_goal_state(configuration_name="init")

    # plan to goal
    logger.info("Planning trajectory")
    plan_result = reachy.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        moveit.execute(robot_trajectory, blocking=True, controllers=[])


if __name__ == "__main__":
    main()
