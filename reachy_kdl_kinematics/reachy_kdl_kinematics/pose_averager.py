from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose


class PoseAverager:
    def __init__(self, window_length):
        self.x = deque(maxlen=window_length)
        self.y = deque(maxlen=window_length)
        self.z = deque(maxlen=window_length)
        self.q = deque(maxlen=window_length)

    def append(self, pose: Pose):
        self.x.append(pose.position.x)
        self.y.append(pose.position.y)
        self.z.append(pose.position.z)
        self.q.append([
            pose.orientation.x, 
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])

    def mean(self) -> Pose:
        pose = Pose()

        pose.position.x = np.mean(self.x)
        pose.position.y = np.mean(self.y)
        pose.position.z = np.mean(self.z)

        avg_q = Rotation.from_quat(self.q).mean().as_quat()
        pose.orientation.x = avg_q[0]
        pose.orientation.y = avg_q[1]
        pose.orientation.z = avg_q[2]
        pose.orientation.w = avg_q[3]

        return pose
    
