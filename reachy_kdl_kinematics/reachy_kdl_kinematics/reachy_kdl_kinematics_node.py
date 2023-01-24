from functools import partial

from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray, String

from reachy_msgs.srv import GetForwardKinematics

from .kdl_kinematics import (
    generate_solver, 
    forward_kinematics,
)


class ReachyKdlKinematics(Node):
    def __init__(self):
        super().__init__('reachy_kdl_kinematics_node')
        self.logger = self.get_logger()

        self.urdf = self.retrieve_urdf()

        self.chain, self.fk_solver, self.ik_solver = {}, {}, {}
        self.fk_srv = {}

        for prefix in ('l', 'r'):
            arm = f'{prefix}_arm'

            chain, fk_solver, ik_solver = generate_solver(self.urdf, 'torso', f'{prefix}_arm_tip')
            
            # We automatically loads the kinematics corresponding to the config
            if chain.getNrOfJoints():
                self.fk_srv[arm] = self.create_service(
                    srv_type=GetForwardKinematics, 
                    srv_name=f'/{arm}/forward_kinematics', 
                    callback=partial(self.forward_kinematics_srv, name=arm),
                )

                self.chain[arm] = chain
                self.fk_solver[arm] = fk_solver
                self.ik_solver[arm] = ik_solver

    def forward_kinematics_srv(
        self, 
        request: GetForwardKinematics.Request, 
        response: GetForwardKinematics.Response,
        name,
    ) -> GetForwardKinematics.Response:
        error, sol = forward_kinematics(
            self.fk_solver[name], 
            request.joint_position.position, 
            self.chain[name].getNrOfJoints(),
        )

        x, y, z = sol[:3, 3]
        q = Rotation.from_matrix(sol[:3, :3]).as_quat()

        # TODO: use error?
        response.success = True

        response.pose.position.x = x
        response.pose.position.y = y
        response.pose.position.z = z

        response.pose.orientation.x = q[0]
        response.pose.orientation.y = q[1]
        response.pose.orientation.z = q[2]
        response.pose.orientation.w = q[3]

        return response

    def arm_forward_kinematics(self, joints, side):
        chain = self.r_arm_chain if side == 'right' else self.l_arm_chain
        solver = self.r_arm_fk_solver if side == 'right' else self.l_arm_fk_solver

        error, sol = forward_kinematics(solver, joints, chain.getNrOfJoints())

        return error, sol

    def retrieve_urdf(self, timeout_sec: float = 15):
        self.logger.info('Retrieving URDF from "/robot_description"...')

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.urdf = None

        def urdf_received(msg: String):
            self.urdf = msg.data

        self.create_subscription(
            msg_type=String, topic='/robot_description',
            qos_profile=qos_profile,
            callback=urdf_received,
        )
        rclpy.spin_once(self, timeout_sec=timeout_sec)

        if self.urdf is None:
            self.logger.error('Could not retrieve the URDF!')
            raise EnvironmentError('Could not retrieve the URDF!')

        self.logger.info('Done!')

        return self.urdf


def main():
    rclpy.init()
    node = ReachyKdlKinematics()
    rclpy.spin(node)


if __name__ == '__main__':
    main()