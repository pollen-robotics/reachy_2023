# The unique ID of the object class we're trying to grasp (same as detections3D label)
string class_id

# The unique ID of the current grasp. There can be several grasps for the same object.
string grasp_id

# 6d grasping pose
geometry_msgs/PoseStamped grasp_pose

# Grasp confidence
float64 grasp_quality

# Grasp gripper opening
float64 grasp_opening

# Defines the acceptable margins of error for the grasp pose.
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
# Units should be meters for translation and radians for rotation.
# If needed, we could use a full 6x6 covariance matrix (these are the diagonal elements).
float64[6] margins