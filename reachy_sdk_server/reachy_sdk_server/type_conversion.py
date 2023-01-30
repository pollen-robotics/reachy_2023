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


def ros_pose_from_pb_matrix(m: Matrix4x4) -> Pose:
    M = np.array(m.data).reshape((4, 4))

    pose = Pose()

    pose.position.x = M[0, 3]
    pose.position.y = M[1, 3]
    pose.position.z = M[2, 3]

    q = Rotation.from_matrix(M[:3, :3]).as_quat()
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    return pose
