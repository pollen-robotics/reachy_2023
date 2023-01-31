# Reachy KDL Kinematics node

## Getting started

It is automatically loaded by the bringup launch files.

### Kinematics computation service

It exposes services for kinematics (forward and inverse) computations:

* **/r_arm/forward_kinematics** ([GetForwardKinematics.srv](../reachy_msgs/srv/GetForwardKinematics.srv)) - Compute the forward kinematics for the right arm. 7 joints should be provided (r_shoulder_pitch, r_shoulder_roll, r_arm_yaw, r_elbow_pitch, r_forearm_yaw, r_wrist_pitch, r_wrist_roll).
* **/r_arm/inverse_kinematics** ([GetInverseKinematics.srv](../reachy_msgs/srv/GetInverseKinematics.srv)) - Compute the inverse kinematics for the right arm.

* **/l_arm/forward_kinematics** ([GetForwardKinematics.srv](../reachy_msgs/srv/GetForwardKinematics.srv)) - Compute the forward kinematics for the left arm. 7 joints should be provided (l_shoulder_pitch, l_shulder_roll, l_arm_yaw, l_elbow_pitch, l_forearm_yaw, l_wrist_pitch, l_wrist_roll).
* **/l_arm/inverse_kinematics** ([GetInverseKinematics.srv](../reachy_msgs/srv/GetInverseKinematics.srv)) - Compute the inverse kinematics for the left arm.

### Cartesian control

The node can also be used as an cartesian controller. Indeed, it listens to specific cartesian targets, compute the corresponding joints commands using the inverse kinematics. Then, it publishes those joints commands directly in the corresponding forward_position controller.

For each arm, two topics can be used (for the right arm):

* **/r_arm/target_pose** ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)) - Compute the inverse kinematics of the given pose and directly send the joint solution to the corresponding forward position controller.
* **/r_arm/averaged_target_pose** ([PoseStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)) - Average the pose (over the n last), compute the inverse kinematics, clip the joint solution according to a max velocity and then publish the result to the corresponding forward posiiotn controller. This version is typically meant to be used at a rather high frequency (> 10Hz).

Similarly for the left arm: **/l_arm/target_pose** and **/l_arm/averaged_target_pose**.

## Requirements

Please note that in order to work properly, this node requires the "/robot_description" and "/joint_states" topics to be published. Depending on your URDF, only the corresponding kinematics chain and their associated services/topics will be created.

## Install

sudo apt install python3-pykdl