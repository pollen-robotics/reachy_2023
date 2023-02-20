from typing import List


MOVE_GROUP_ARM: str = "left_arm"
MOVE_GROUP_GRIPPER: str = "left_gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [1.0]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0]


def joint_names(prefix: str = "l_") -> List[str]:
    return [
        prefix + "shoulder_pitch",
        prefix + "shoulder_roll",
        prefix + "arm_yaw",
        prefix + "elbow_pitch",
        prefix + "forearm_yaw",
        prefix + "wrist_pitch",
        prefix + "wrist_roll",
    ]


def base_link_name() -> str:
    return "world"


def end_effector_name(prefix: str = "l_") -> str:
    return prefix + "arm_tip"


def gripper_joint_names(prefix: str = "l_") -> List[str]:
    return [
        prefix + "gripper",

    ]
