"""Compute the kinematics for Reachy using the KDL library and its URDF definition."""
from typing import Tuple
import numpy as np

import PyKDL as kdl

from .kdl_parser_py import urdf


def generate_solver(urdf_str: str, root: str, tip: str):
    """Create an FK/IK solvers for each arm (left/right)."""
    success, urdf_tree = urdf.treeFromString(urdf_str)
    if not success:
        raise IOError('Could not parse the URDF!')

    chain = urdf_tree.getChain(root, tip)
    fk_solver = kdl.ChainFkSolverPos_recursive(chain)

    ik_solver = kdl.ChainIkSolverPos_LMA(
        chain,
        eps=1e-5,
        _maxiter=500,
        _eps_joints=1e-15,
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
