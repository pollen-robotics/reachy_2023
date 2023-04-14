"""Compute the kinematics for Reachy using the KDL library and its URDF definition."""
from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose

import PyKDL as kdl

from .kdl_parser_py import urdf


def generate_solver(urdf_str: str, root: str, tip: str, L: np.ndarray = np.array([1.0, 1.0, 1.0, 0.01, 0.01, 0.01])):
    """Create an FK/IK solvers for each arm (left/right)."""
    success, urdf_tree = urdf.treeFromString(urdf_str)
    if not success:
        raise IOError('Could not parse the URDF!')

    chain = urdf_tree.getChain(root, tip)
    fk_solver = kdl.ChainFkSolverPos_recursive(chain)

    ik_solver = kdl.ChainIkSolverPos_LMA(
        chain,
        eps=1e-5,
        maxiter=500,
        eps_joints=1e-15,
        L=L
    )

    return chain, fk_solver, ik_solver


def forward_kinematics(fk_solver, joints: np.ndarray, nb_joints: int) -> Tuple[float, np.ndarray]:
    """Compute the forward kinematics of the given arm.
    The function assumes the number of joints is correct!
    """
    q = kdl.JntArray(nb_joints)
    for i, j in enumerate(joints):
        q[i] = j

    pose = kdl.Frame()
    res = fk_solver.JntToCart(q, pose)

    M = np.eye(4)
    M[:3, 3] = list(pose.p)
    for i in range(3):
        for j in range(3):
            M[i, j] = pose.M[i, j]

    return res, M


def inverse_kinematics(ik_solver, q0: np.ndarray, target_pose: np.ndarray, nb_joints: int) -> Tuple[float, np.ndarray]:
    """Compute the inverse kinematics of the given arm.
    The function assumes the number of joints is correct!
    """
    x, y, z = target_pose[:3, 3]
    R = target_pose[:3, :3].flatten().tolist()

    _q0 = kdl.JntArray(nb_joints)
    for i, q in enumerate(q0):
        _q0[i] = q

    pose = kdl.Frame()
    pose.p = kdl.Vector(x, y, z)
    pose.M = kdl.Rotation(*R)

    sol = kdl.JntArray(nb_joints)
    res = ik_solver.CartToJnt(_q0, pose, sol)
    sol = list(sol)

    return res, sol


def ros_pose_to_matrix(pose: Pose) -> np.ndarray:
    M = np.eye(4)

    M[0, 3] = pose.position.x
    M[1, 3] = pose.position.y
    M[2, 3] = pose.position.z

    q = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    M[:3, :3] = Rotation.from_quat(q).as_matrix()

    return M
