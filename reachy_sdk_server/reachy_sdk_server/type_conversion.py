import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose

from reachy_sdk_api.kinematics_pb2 import Matrix4x4

def pb_matrix_from_ros_pose(pose: Pose) -> Matrix4x4:
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

    return Matrix4x4(data=M.flatten())
    