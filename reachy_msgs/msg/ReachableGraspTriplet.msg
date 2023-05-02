# The unique ID of the object class we're trying to grasp (same as detections3D label)
string class_id

# The unique ID of the current grasp. There can be several grasps for the same object.
string grasp_id

# 3 Reachable poses are defined: pregrasp, grasp and postgrasp. 
# For each pose, there is the 6D pose as PoseStamped (task space) and the working IK solution as JointState (joint space).
geometry_msgs/PoseStamped pregrasp_pose
sensor_msgs/JointState pregrasp_joints

geometry_msgs/PoseStamped grasp_pose
sensor_msgs/JointState grasp_joints

geometry_msgs/PoseStamped postgrasp_pose
sensor_msgs/JointState postgrasp_joints

# Grasp confidence
float64 grasp_quality

# Grasp gripper opening
float64 grasp_opening

