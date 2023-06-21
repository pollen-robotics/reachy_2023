cmake_minimum_required(VERSION 3.8)
project(reachy_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gripper.msg"
  "msg/ZoomCommand.msg"
  "msg/TrajectoryFeedback.msg"
  "msg/TrajectoryResult.msg"
  "msg/GraspPose.msg"
  "msg/GraspPoseArray.msg"
  "msg/ReachableGraspTriplet.msg"
  "msg/ReachableGraspTripletArray.msg"

  "action/GotoTrajectory.action"

  "srv/GetForwardKinematics.srv"
  "srv/GetInverseKinematics.srv"
  "srv/GetCameraZoomFocus.srv"
  "srv/SetCameraZoomFocus.srv"
  "srv/GetCameraZoomLevel.srv"
  "srv/SetCameraZoomLevel.srv"
  "srv/GetCameraZoomSpeed.srv"
  "srv/SetCameraZoomSpeed.srv"
  "srv/SetFocusState.srv"
  "srv/GetDynamicState.srv"
  "srv/GetReachability.srv"
  "srv/GetReachableGraspTriplets.srv"
  "srv/GetObjectDetection.srv"
  "srv/GetSimpleGraspPose.srv"
  "srv/GetGraspCNN.srv"

  DEPENDENCIES geometry_msgs sensor_msgs std_msgs trajectory_msgs vision_msgs

)

ament_package()
