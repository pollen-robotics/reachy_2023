from functools import partial
from threading import Event
from typing import List

import numpy as np
import copy
import re
import time
import os

from scipy.spatial.transform import Rotation

from .pose_averager import PoseAverager

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from reachy_msgs.srv import (
    GetForwardKinematics,
    GetInverseKinematics,
    GetReachability,
)

from .kdl_kinematics import (
    generate_solver,
    forward_kinematics,
    inverse_kinematics,
    ros_pose_to_matrix,
)
from reachy_placo import ik_reachy_placo

USE_QP_IK = True


class ReachyKdlKinematics(LifecycleNode):
    def __init__(self):
        super().__init__("reachy_kdl_kinematics_node")
        self.logger = self.get_logger()

        self.urdf = self.retrieve_urdf()

        # Listen to /joint_state to get current position
        # used by averaged_target_pose
        self._current_pos = {}
        self.joint_state_sub = self.create_subscription(
            msg_type=JointState,
            topic="/joint_states",
            qos_profile=5,
            callback=self.on_joint_state,
        )
        self.joint_state_ready = Event()
        self.wait_for_joint_state()

        self.chain, self.fk_solver, self.ik_solver = {}, {}, {}
        self.fk_srv, self.ik_srv, self.reach_srv = {}, {}, {}
        self.target_sub, self.averaged_target_sub = {}, {}
        self.averaged_pose = {}
        self.max_joint_vel = {}

        # Removing URDF tags that Placo does not support
        new_urdf = copy.copy(self.urdf)
        new_urdf = remove_ros2_control_tags(new_urdf)
        new_urdf = fix_arm_tip_names(new_urdf)

        # Using the collision.json file from reachy_placo
        collision_file_location = os.path.join(
            os.path.dirname(ik_reachy_placo.__file__), "reachy"
        )

        self.ik_reachy_placo = ik_reachy_placo.IKReachyQP(
            viewer_on=True, collision_avoidance=True
        )
        self.ik_reachy_placo.setup(
            urdf_path=collision_file_location,
            urdf_string=new_urdf,
        )

        self.ik_reachy_placo.create_tasks()

        for prefix in ("l", "r"):
            arm = f"{prefix}_arm"

            chain, fk_solver, ik_solver = generate_solver(
                self.urdf, "torso", f"{prefix}_arm_tip"
            )

            # We automatically loads the kinematics corresponding to the config
            if chain.getNrOfJoints():
                self.logger.info(f'Found kinematics chain for "{arm}"!')

                # Create forward kinematics service
                self.fk_srv[arm] = self.create_service(
                    srv_type=GetForwardKinematics,
                    srv_name=f"/{arm}/forward_kinematics",
                    callback=partial(self.forward_kinematics_srv, name=arm),
                )
                self.logger.info(f'Adding service "{self.fk_srv[arm].srv_name}"...')

                # Create inverse kinematics service
                self.ik_srv[arm] = self.create_service(
                    srv_type=GetInverseKinematics,
                    srv_name=f"/{arm}/inverse_kinematics",
                    callback=partial(self.inverse_kinematics_srv, name=arm),
                )
                self.logger.info(f'Adding service "{self.ik_srv[arm].srv_name}"...')
                
                # Create reachability service
                self.reach_srv[arm] = self.create_service(
                    srv_type=GetReachability,
                    srv_name=f"/{arm}/reachability",
                    callback=partial(self.reachability_srv, name=arm),
                )
                self.logger.info(f'Adding service "{self.reach_srv[arm].srv_name}"...')
                
                

                # Create cartesian control pub/subscription
                forward_position_pub = self.create_publisher(
                    msg_type=Float64MultiArray,
                    topic=f"/{arm}_forward_position_controller/commands",
                    qos_profile=5,
                )

                self.target_sub[arm] = self.create_subscription(
                    msg_type=PoseStamped,
                    topic=f"/{arm}/target_pose",
                    qos_profile=5,
                    callback=partial(
                        self.on_target_pose,
                        name=arm,
                        # arm straight, with elbow at -90 (facing forward)
                        q0=[0, 0, 0, -np.pi / 2, 0, 0, 0],
                        forward_publisher=forward_position_pub,
                    ),
                )
                self.logger.info(
                    f'Adding subscription on "{self.target_sub[arm].topic}"...'
                )

                self.averaged_target_sub[arm] = self.create_subscription(
                    msg_type=PoseStamped,
                    topic=f"/{arm}/averaged_target_pose",
                    qos_profile=5,
                    callback=partial(
                        self.on_averaged_target_pose,
                        name=arm,
                        # arm straight, with elbow at -90 (facing forward)
                        q0=[0, 0, 0, -np.pi / 2, 0, 0, 0],
                        forward_publisher=forward_position_pub,
                    ),
                )
                self.averaged_pose[arm] = PoseAverager(window_length=1)
                self.max_joint_vel[arm] = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
                self.logger.info(
                    f'Adding subscription on "{self.target_sub[arm].topic}"...'
                )

                self.chain[arm] = chain
                self.fk_solver[arm] = fk_solver
                self.ik_solver[arm] = ik_solver

        # Kinematics for the head
        chain, fk_solver, ik_solver = generate_solver(
            self.urdf,
            "torso",
            "head_tip",
            L=np.array([1e-6, 1e-6, 1e-6, 1.0, 1.0, 1.0]),
        )  # L weight matrix to considere only the orientation

        # We automatically load the kinematics corresponding to the config
        if chain.getNrOfJoints():
            self.logger.info(f"Found kinematics chain for head!")

            # Create forward kinematics service
            srv = self.create_service(
                srv_type=GetForwardKinematics,
                srv_name="/head/forward_kinematics",
                callback=partial(self.forward_kinematics_srv, name="head"),
            )
            self.fk_srv["head"] = srv
            self.logger.info(f'Adding service "{srv.srv_name}"...')

            # Create inverse kinematics service
            srv = self.create_service(
                srv_type=GetInverseKinematics,
                srv_name="/head/inverse_kinematics",
                callback=partial(self.inverse_kinematics_srv, name="head"),
            )
            self.ik_srv["head"] = srv
            self.logger.info(f'Adding service "{srv.srv_name}"...')

            # Create cartesian control subscription
            head_forward_position_pub = self.create_publisher(
                msg_type=Float64MultiArray,
                topic="/neck_forward_position_controller/commands",  # need
                qos_profile=5,
            )

            sub = self.create_subscription(
                msg_type=PoseStamped,
                topic="/head/target_pose",
                qos_profile=5,
                callback=partial(
                    self.on_target_pose,
                    name="head",
                    # head straight
                    q0=[0, 0, 0],
                    forward_publisher=head_forward_position_pub,
                ),
            )
            self.target_sub["head"] = sub
            self.logger.info(f'Adding subscription on "{sub.topic}"...')

            sub = self.create_subscription(
                msg_type=PoseStamped,
                topic="/head/averaged_target_pose",
                qos_profile=5,
                callback=partial(
                    self.on_averaged_target_pose,
                    name="head",
                    # head straight
                    q0=[0, 0, 0],
                    forward_publisher=head_forward_position_pub,
                ),
            )
            self.averaged_target_sub["head"] = sub
            self.averaged_pose["head"] = PoseAverager(window_length=1)

            self.max_joint_vel["head"] = np.array([0.1, 0.1, 0.1])
            self.logger.info(f'Adding subscription on "{sub.topic}"...')

            self.chain["head"] = chain
            self.fk_solver["head"] = fk_solver
            self.ik_solver["head"] = ik_solver

        self.logger.info(f"Kinematics node ready!")
        self.trigger_configure()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Dummy state to minimize impact on current behavior
        self.logger.info(
            "Configuring state has been called, going into inactive to release event trigger"
        )
        return TransitionCallbackReturn.SUCCESS

    def forward_kinematics_srv(
        self,
        request: GetForwardKinematics.Request,
        response: GetForwardKinematics.Response,
        name,
    ) -> GetForwardKinematics.Response:
        try:
            joint_position = self.check_position(
                request.joint_position, self.chain[name]
            )
        except KeyError:
            response.success = False
            return response

        error, sol = forward_kinematics(
            self.fk_solver[name],
            joint_position,
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

    def inverse_kinematics_srv(
        self,
        request: GetInverseKinematics.Request,
        response: GetInverseKinematics.Response,
        name,
    ) -> GetInverseKinematics.Response:
        M = ros_pose_to_matrix(request.pose)
        q0 = request.q0.position

        if name == "head" or not (USE_QP_IK):
            # IK using KDL
            error, sol = inverse_kinematics(
                self.ik_solver[name],
                q0=q0,
                target_pose=M,
                nb_joints=self.chain[name].getNrOfJoints(),
            )
        else:
            # IK using Placo
            self.logger.info(f"calling IK for {name}")

            lt0 = time.time()
            sol, errors = self.ik_reachy_placo.ik_continuous(
                M,
                arm_name=name,
                q0=q0,
                max_iter=20,
                nb_stepper_solve=5,
            )
            dt = time.time() - lt0
            self.logger.info(f"IK took {dt*1000:.2f} ms")

            # self.logger.info(f"IK errors: {[round(error, 4) for error in errors]}")
            # self.logger.info(f"sol: {sol}")
            # rads to deg
            for s in sol:
                s = s * 180 / np.pi

        # TODO: use error
        response.success = True
        response.joint_position.name = self.get_chain_joints_name(self.chain[name])
        response.joint_position.position = sol

        return response
    
    def reachability_srv(
        self,
        request: GetReachability.Request,
        response: GetReachability.Response,
        name,
    ) -> GetReachability.Response:
        try :
            M = ros_pose_to_matrix(request.pose)
            q0 = request.q0.position

            if name == "head" :
                self.logger.error(f"The reachability service does not exist for {name} yet")
                raise NotImplementedError
            elif not (USE_QP_IK):
                self.logger.error(f"The reachability service needs the flag USE_QP_IK to be True" )
                raise ValueError
            else:
                # Reachability check + IK using Placo
                self.logger.info(f"calling the Reachability service for {name}")

                lt0 = time.time()
                is_reachable, sol, errors = self.ik_reachy_placo.is_pose_reachable(
                    M,
                    arm_name=name,
                    q0=q0,
                    max_iter=20,
                    nb_stepper_solve=5,
                    tolerances=[0.001, 0.001, 0.001, 0.02, 0.02, 0.02],
                )
                dt = time.time() - lt0
                self.logger.info(f"Reachability took {dt*1000:.2f} ms")

                # self.logger.info(f"IK errors: {[round(error, 4) for error in errors]}")
                # self.logger.info(f"sol: {sol}")
                # rads to deg
                for s in sol:
                    s = s * 180 / np.pi

                response.success = is_reachable
                self.logger.info(f"a")
                
                response.joint_position.name = self.get_chain_joints_name(self.chain[name])
                self.logger.info(f"b")
                
                response.joint_position.position = sol
                self.logger.info(f"c")
                
                response.errors = errors
                self.logger.info(f"Returning...")
                return response
                
        except Exception as e:
            self.logger.error(f"Error in reachability service: {e}")
            raise e    
    

    def on_target_pose(self, msg: PoseStamped, name, q0, forward_publisher):
        M = ros_pose_to_matrix(msg.pose)

        if name == "head" or not (USE_QP_IK):
            # IK using KDL
            error, sol = inverse_kinematics(
                self.ik_solver[name],
                q0=q0,
                target_pose=M,
                nb_joints=self.chain[name].getNrOfJoints(),
            )
        else:
            # IK using Placo
            sol, errors = self.ik_reachy_placo.ik_continuous(
                M,
                arm_name=name,
                q0=q0,
                max_iter=20,
                nb_stepper_solve=5,
            )
            # rads to deg
            for s in sol:
                s = s * 180 / np.pi

        # TODO: check error

        msg = Float64MultiArray()
        msg.data = sol

        forward_publisher.publish(msg)

    def on_averaged_target_pose(self, msg: PoseStamped, name, q0, forward_publisher):
        self.averaged_pose[name].append(msg.pose)
        avg_pose = self.averaged_pose[name].mean()

        M = ros_pose_to_matrix(avg_pose)

        if name == "head" or not (USE_QP_IK):
            # IK using KDL
            error, sol = inverse_kinematics(
                self.ik_solver[name],
                q0=q0,
                target_pose=M,
                nb_joints=self.chain[name].getNrOfJoints(),
            )
        else:
            # IK using Placo
            sol, errors = self.ik_reachy_placo.ik_continuous(
                M,
                arm_name=name,
                q0=q0,
                max_iter=20,
                nb_stepper_solve=5,
            )
            # rads to deg
            for s in sol:
                s = s * 180 / np.pi

        # TODO: check error

        current_position = np.array(self.get_current_position(self.chain[name]))

        vel = np.array(sol) - current_position
        # vel = np.clip(vel, -self.max_joint_vel[name], self.max_joint_vel[name])

        smoothed_sol = current_position + vel

        msg = Float64MultiArray()
        msg.data = smoothed_sol.tolist()

        forward_publisher.publish(msg)

    def get_current_position(self, chain) -> List[float]:
        joints = self.get_chain_joints_name(chain)
        return [self._current_pos[j] for j in joints]

    def wait_for_joint_state(self):
        while not self.joint_state_ready.is_set():
            self.logger.info("Waiting for /joint_states...")
            rclpy.spin_once(self)

    def on_joint_state(self, msg: JointState):
        for j, pos in zip(msg.name, msg.position):
            self._current_pos[j] = pos

        self.joint_state_ready.set()

    def retrieve_urdf(self, timeout_sec: float = 15):
        self.logger.info('Retrieving URDF from "/robot_description"...')

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.urdf = None

        def urdf_received(msg: String):
            self.urdf = msg.data

        self.create_subscription(
            msg_type=String,
            topic="/robot_description",
            qos_profile=qos_profile,
            callback=urdf_received,
        )
        rclpy.spin_once(self, timeout_sec=timeout_sec)

        if self.urdf is None:
            self.logger.error("Could not retrieve the URDF!")
            raise EnvironmentError("Could not retrieve the URDF!")

        self.logger.info("Done!")

        return self.urdf

    def check_position(self, js: JointState, chain) -> List[float]:
        pos = dict(zip(js.name, js.position))
        try:
            joints = [pos[j] for j in self.get_chain_joints_name(chain)]
            return joints
        except KeyError:
            self.logger.warning(
                f"Incorrect joints found ({js.name} vs {self.get_chain_joints_name(chain)})"
            )
            raise

    def get_chain_joints_name(self, chain):
        return [
            chain.getSegment(i).getJoint().getName()
            for i in range(chain.getNrOfJoints())
        ]


def remove_ros2_control_tags(urdf_string):
    # Use regex to find and remove everything between <ros2_control> and </ros2_control>, including the tags
    stripped_urdf_string = re.sub(
        r"<ros2_control[^>]*>.*?</ros2_control>", "", urdf_string, flags=re.DOTALL
    )
    return stripped_urdf_string


def fix_arm_tip_names(urdf_string):
    # Replace 'r_arm_tip' and 'l_arm_tip' link names with 'r_arm_tip_link' and 'l_arm_tip_link'
    modified_urdf_string = urdf_string.replace(
        '<link name="r_arm_tip">', '<link name="r_arm_tip_link">'
    )
    modified_urdf_string = modified_urdf_string.replace(
        '<link name="l_arm_tip">', '<link name="l_arm_tip_link">'
    )

    # Replace child link names 'r_arm_tip' and 'l_arm_tip' with 'r_arm_tip_link' and 'l_arm_tip_link' in <joint> tags
    modified_urdf_string = modified_urdf_string.replace(
        '<child link="r_arm_tip"/>', '<child link="r_arm_tip_link"/>'
    )
    modified_urdf_string = modified_urdf_string.replace(
        '<child link="l_arm_tip"/>', '<child link="l_arm_tip_link"/>'
    )

    return modified_urdf_string


# Does not work as expected
# def fix_arm_tip_names(urdf_string):
#     # Replace 'r_arm_tip' and 'l_arm_tip' link names with 'r_arm_tip_link' and 'l_arm_tip_link'
#     modified_urdf_string = re.sub(
#         r'(<link name=")(r|l)(_arm_tip)(">)', r"\1\2\3_link\4", urdf_string
#     )

#     # Replace child link names 'r_arm_tip' and 'l_arm_tip' with 'r_arm_tip_link' and 'l_arm_tip_link' in <joint> tags
#     modified_urdf_string = re.sub(
#         r'(<child link=")(r|l)(_arm_tip)(">)', r"\1\2\3_link\4", modified_urdf_string
#     )

#     return modified_urdf_string


def main():
    rclpy.init()
    node = ReachyKdlKinematics()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
